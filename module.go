// Package detectionstosegments uses a 2D segmenter and a camera that can project its images
// to 3D to project the bounding boxes to 3D in order to created a segmented point cloud.
package detectionstosegments

import (
	"context"
	"fmt"
	"image"
	"image/color"
	"os"
	"slices"

	"github.com/golang/geo/r3"
	"github.com/go-viper/mapstructure/v2"
	"github.com/pkg/errors"
	"go.opencensus.io/trace"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/rimage"
	"go.viam.com/rdk/rimage/transform"
	"go.viam.com/rdk/robot"
	"go.viam.com/rdk/robot/client"
	servicevision "go.viam.com/rdk/services/vision"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/utils"
	vision "go.viam.com/rdk/vision"
	"go.viam.com/rdk/vision/objectdetection"
	"go.viam.com/rdk/vision/segmentation"
	rpc "go.viam.com/utils/rpc"
)

var DetectionsToSegments = resource.NewModel("viam", "vision", "detections-to-segments")

// DetectionSegmenterConfig are the optional parameters to turn a detector into a segmenter.
type DetectionSegmenterConfig struct {
	DetectorName       string  `json:"detector_name"`
	ConfidenceThresh   float64 `json:"confidence_threshold_pct"`
	MeanK              int     `json:"mean_k"`
	Sigma              float64 `json:"sigma"`
	DefaultCamera      string  `json:"camera_name"`
	DepthThresholdMm   int     `json:"depth_threshold_mm"`
	MinPointsInSegment int     `json:"min_points_in_segment"`
	ClusteringRadiusMm float64 `json:"clustering_radius_mm"`
}

func init() {
	resource.RegisterService(servicevision.API, DetectionsToSegments, resource.Registration[servicevision.Service, *DetectionSegmenterConfig]{
		Constructor: func(
			ctx context.Context, deps resource.Dependencies, c resource.Config, logger logging.Logger,
		) (servicevision.Service, error) {
			attrs, err := resource.NativeConfig[*DetectionSegmenterConfig](c)
			if err != nil {
				return nil, err
			}
			return register3DSegmenterFromDetector(ctx, c.ResourceName(), attrs, deps, logger)
		},
	})
}

// register3DSegmenterFromDetector creates a 3D segmenter from a previously registered detector.
func register3DSegmenterFromDetector(
	ctx context.Context,
	name resource.Name,
	conf *DetectionSegmenterConfig,
	deps resource.Dependencies,
	logger logging.Logger,
) (servicevision.Service, error) {
	_, span := trace.StartSpan(ctx, "service::vision::register3DSegmenterFromDetector")
	defer span.End()
	if conf == nil {
		return nil, errors.New("config for 3D segmenter made from a detector cannot be nil")
	}
	detectorService, err := servicevision.FromDependencies(deps, conf.DetectorName)
	if err != nil {
		return nil, errors.Wrapf(err, "could not find necessary dependency, detector %q", conf.DetectorName)
	}
	confThresh := 0.5 // default value
	if conf.ConfidenceThresh > 0.0 {
		confThresh = conf.ConfidenceThresh
	}
	detector := func(ctx context.Context, img image.Image) ([]objectdetection.Detection, error) {
		return detectorService.Detections(ctx, img, nil)
	}
	var client robot.Robot
	if conf.DefaultCamera != "" {
		_, err = camera.FromDependencies(deps, conf.DefaultCamera)
		if err != nil {
			return nil, errors.Errorf("could not find camera %q", conf.DefaultCamera)
		}
		client, err = connectToMachineFromEnv(ctx, logger)
		if err != nil {
			logger.Warnf("could not connect to machine for frame transforms, point clouds will be in camera frame: %v", err)
		}
	}
	clusterRadius := conf.ClusteringRadiusMm
	segmenter, err := DetectionSegmenter(objectdetection.Detector(detector), conf.MeanK, conf.Sigma, confThresh, conf.DepthThresholdMm, conf.MinPointsInSegment, clusterRadius, client, conf.DefaultCamera)
	if err != nil {
		return nil, errors.Wrap(err, "cannot create 3D segmenter from detector")
	}
	return servicevision.NewService(name, deps, logger, nil, nil, detector, segmenter, conf.DefaultCamera)
}

func (conf *DetectionSegmenterConfig) Validate(path string) ([]string, []string, error) {
	var requiredDeps []string
	var optionalDeps []string

	if conf.DefaultCamera != "" {
		requiredDeps = append(requiredDeps, conf.DefaultCamera)
	}

	if conf.DetectorName == "" {
		return nil, optionalDeps, errors.Errorf("expected a detector to be specified")
	}
	requiredDeps = append(requiredDeps, conf.DetectorName)

	if conf.MeanK <= 0 {
		return nil, optionalDeps, errors.Errorf("expected a mean k to be specified")
	}

	if conf.Sigma <= 0 {
		return nil, optionalDeps, errors.Errorf("expected a sigma to be specified")
	}

	return requiredDeps, optionalDeps, nil
}

// ConvertAttributes changes the AttributeMap input into a DetectionSegmenterConfig.
func (dsc *DetectionSegmenterConfig) ConvertAttributes(am utils.AttributeMap) error {
	decoder, err := mapstructure.NewDecoder(&mapstructure.DecoderConfig{TagName: "json", Result: dsc})
	if err != nil {
		return err
	}
	return decoder.Decode(am)
}

func cameraToProjector(
	ctx context.Context,
	source camera.Camera,
) (transform.Projector, error) {
	if source == nil {
		return nil, errors.New("cannot have a nil source")
	}
	props, err := source.Properties(ctx)
	if err != nil {
		return nil, camera.NewPropertiesError("source camera")
	}
	if props.IntrinsicParams == nil {
		return &transform.ParallelProjection{}, nil
	}
	cameraModel := transform.PinholeCameraModel{}
	cameraModel.PinholeCameraIntrinsics = props.IntrinsicParams

	if props.DistortionParams != nil {
		cameraModel.Distortion = props.DistortionParams
	}

	return &cameraModel, nil
}

// DetectionSegmenter will take an objectdetector.Detector and turn it into a Segmenter.
// The params for the segmenter are "mean_k" and "sigma" for the statistical filter on the point clouds.
// depthThresholdMm filters out background points that deviate from the median depth in the bounding box.
// minPoints sets the minimum number of points required for a valid segment.
func DetectionSegmenter(
	detector objectdetection.Detector,
	meanK int, sigma, confidenceThresh float64,
	depthThresholdMm, minPoints int,
	clusteringRadiusMm float64,
	client robot.Robot, cameraName string,
) (segmentation.Segmenter, error) {
	var err error
	if detector == nil {
		return nil, errors.New("detector cannot be nil")
	}
	var filter func(in, out pointcloud.PointCloud) error
	if meanK > 0 && sigma > 0.0 {
		filter, err = pointcloud.StatisticalOutlierFilter(meanK, sigma)
		if err != nil {
			return nil, err
		}
	}
	seg := func(ctx context.Context, src camera.Camera) ([]*vision.Object, error) {
		proj, err := cameraToProjector(ctx, src)
		if err != nil {
			return nil, err
		}
		imgs, _, err := src.Images(ctx, nil, nil)
		if err != nil {
			return nil, errors.Wrapf(err, "detection segmenter")
		}
		var img *rimage.Image
		var dmimg image.Image
		for _, i := range imgs {
			thisI := i
			if thisI.SourceName == "color" {
				colorImg, err := thisI.Image(ctx)
				if err != nil {
					return nil, errors.Wrap(err, "failed to get color image")
				}
				img = rimage.ConvertImage(colorImg)
			}
			if thisI.SourceName == "depth" {
				dmimg, err = thisI.Image(ctx)
				if err != nil {
					return nil, errors.Wrap(err, "failed to get depth image")
				}
			}
		}
		if img == nil || dmimg == nil {
			return nil, errors.New("source camera's getImages method did not have 'color' and 'depth' images")
		}
		dm, err := rimage.ConvertImageToDepthMap(ctx, dmimg)
		if err != nil {
			return nil, err
		}
		im := rimage.CloneImage(img)
		dets, err := detector(ctx, im) // detector may modify the input image
		if err != nil {
			return nil, err
		}

		objects := make([]*vision.Object, 0, len(dets))
		for _, d := range dets {
			if d.Score() < confidenceThresh {
				continue
			}
			pc, err := detectionToPointCloud(d, img, dm, proj, depthThresholdMm)
			if err != nil {
				return nil, err
			}
			if filter != nil {
				out := pc.CreateNewRecentered(spatialmath.NewZeroPose())
				err = filter(pc, out)
				if err != nil {
					return nil, err
				}
				pc = out
			}
			if pc.Size() == 0 {
				continue
			}
			// Remove the dominant plane (e.g. table surface) via RANSAC.
			if pc.Size() > 3 {
				_, nonPlane, planeErr := segmentation.SegmentPlane(ctx, pc, 1000, 10.0)
				if planeErr == nil && nonPlane.Size() > 0 {
					pc = nonPlane
				}
			}
			if minPoints > 0 && pc.Size() < minPoints {
				continue
			}
			// Split into clusters using radius-based nearest neighbors.
			var segments []pointcloud.PointCloud
			if clusteringRadiusMm > 0 {
				segments = clusterPointCloud(pc, clusteringRadiusMm, minPoints)
			} else {
				segments = []pointcloud.PointCloud{pc}
			}
			for _, cluster := range segments {
				if client != nil && cameraName != "" {
					cluster, err = client.TransformPointCloud(ctx, cluster, cameraName, "world")
					if err != nil {
						return nil, err
					}
				}
				obj, err := vision.NewObjectWithLabel(cluster, d.Label(), nil)
				if err != nil {
					return nil, err
				}
				objects = append(objects, obj)
			}
		}
		return objects, nil
	}
	return seg, nil
}

func detectionToPointCloud(
	d objectdetection.Detection,
	im *rimage.Image, dm *rimage.DepthMap,
	proj transform.Projector,
	depthThresholdMm int,
) (pointcloud.PointCloud, error) {
	bb := d.BoundingBox()
	if bb == nil {
		return nil, errors.New("detection bounding box cannot be nil")
	}
	// Use custom deprojection for pinhole cameras to skip zero-depth pixels
	// and optionally filter background points by depth threshold.
	switch p := proj.(type) {
	case *transform.PinholeCameraModel:
		return depthFilteredPointCloud(*bb, im, dm, p.PinholeCameraIntrinsics, depthThresholdMm)
	default:
		// ParallelProjection already skips zero-depth pixels
		return proj.RGBDToPointCloud(im, dm, *bb)
	}
}

// depthFilteredPointCloud projects pixels within a bounding box to 3D, skipping zero-depth pixels
// and optionally filtering out background points that deviate from the median depth.
func depthFilteredPointCloud(
	bb image.Rectangle,
	img *rimage.Image,
	dm *rimage.DepthMap,
	intrinsics *transform.PinholeCameraIntrinsics,
	depthThresholdMm int,
) (pointcloud.PointCloud, error) {
	if img == nil {
		return nil, errors.New("no rgb image for deprojection")
	}
	if dm == nil {
		return nil, errors.New("no depth map for deprojection")
	}
	bounds := bb.Intersect(img.Bounds())
	startX, startY := bounds.Min.X, bounds.Min.Y
	endX, endY := bounds.Max.X, bounds.Max.Y

	// First pass: collect non-zero depths to compute the median.
	depths := make([]int, 0, (endX-startX)*(endY-startY))
	for y := startY; y < endY; y++ {
		for x := startX; x < endX; x++ {
			z := int(dm.GetDepth(x, y))
			if z > 0 {
				depths = append(depths, z)
			}
		}
	}
	if len(depths) == 0 {
		return pointcloud.NewBasicEmpty(), nil
	}

	slices.Sort(depths)
	medianDepth := depths[len(depths)/2]
	maxDepth := findDepthCutoff(depths)

	// Second pass: project pixels to 3D, filtering by depth.
	pc := pointcloud.NewBasicEmpty()
	for y := startY; y < endY; y++ {
		for x := startX; x < endX; x++ {
			z := int(dm.GetDepth(x, y))
			if z == 0 {
				continue
			}
			if z > maxDepth {
				continue
			}
			if depthThresholdMm > 0 {
				diff := z - medianDepth
				if diff < 0 {
					diff = -diff
				}
				if diff > depthThresholdMm {
					continue
				}
			}
			px, py, pz := intrinsics.PixelToPoint(float64(x), float64(y), float64(z))
			r, g, b := img.GetXY(x, y).RGB255()
			err := pc.Set(
				pointcloud.NewVector(px, py, pz),
				pointcloud.NewColoredData(color.NRGBA{R: r, G: g, B: b, A: 255}),
			)
			if err != nil {
				return nil, err
			}
		}
	}
	return pc, nil
}

// clusterPointCloud splits a point cloud into separate clusters using
// radius-based nearest neighbors. Each connected group of points (within
// the given radius) becomes its own point cloud. Clusters smaller than
// minPoints are discarded.
func clusterPointCloud(cloud pointcloud.PointCloud, radius float64, minPoints int) []pointcloud.PointCloud {
	if cloud.Size() == 0 {
		return nil
	}
	kdt := pointcloud.ToKDTree(cloud)
	clusters := segmentation.NewSegments()
	c := 0
	kdt.Iterate(0, 0, func(v r3.Vector, d pointcloud.Data) bool {
		if _, ok := clusters.Indices[v]; ok {
			return true
		}
		nn := kdt.RadiusNearestNeighbors(v, radius, false)
		for _, neighbor := range nn {
			nv := neighbor.P
			ptIndex, ptOk := clusters.Indices[v]
			neighborIndex, neighborOk := clusters.Indices[nv]
			switch {
			case ptOk && neighborOk:
				if ptIndex != neighborIndex {
					clusters.MergeClusters(ptIndex, neighborIndex) //nolint:errcheck
				}
			case !ptOk && neighborOk:
				clusters.AssignCluster(v, d, neighborIndex) //nolint:errcheck
			case ptOk && !neighborOk:
				clusters.AssignCluster(neighbor.P, neighbor.D, ptIndex) //nolint:errcheck
			case !ptOk && !neighborOk:
				clusters.AssignCluster(v, d, c)           //nolint:errcheck
				clusters.AssignCluster(nv, neighbor.D, c) //nolint:errcheck
				c++
			}
		}
		if _, ok := clusters.Indices[v]; !ok {
			clusters.AssignCluster(v, d, c) //nolint:errcheck
			c++
		}
		return true
	})
	clouds := clusters.PointClouds()
	return pointcloud.PrunePointClouds(clouds, minPoints)
}

// findDepthCutoff takes a sorted slice of depths and finds a max depth cutoff
// by looking for a rapid gap in the depth distribution. Points closest to the
// camera belong to the object; a sudden jump in depth indicates background
// (walls, floors). The heuristic uses the interquartile range (IQR) of the
// depths seen so far: if a gap between consecutive sorted depths exceeds 2x
// the IQR, everything beyond that gap is background.
func findDepthCutoff(sortedDepths []int) int {
	n := len(sortedDepths)
	if n < 4 {
		return sortedDepths[n-1]
	}
	q1 := sortedDepths[n/4]
	q3 := sortedDepths[3*n/4]
	iqr := q3 - q1
	if iqr < 1 {
		iqr = 1
	}
	threshold := 2 * iqr
	for i := 1; i < n; i++ {
		gap := sortedDepths[i] - sortedDepths[i-1]
		if gap > threshold {
			return sortedDepths[i-1]
		}
	}
	return sortedDepths[n-1]
}

func connectToMachineFromEnv(ctx context.Context, logger logging.Logger) (robot.Robot, error) {
	host := os.Getenv(utils.MachineFQDNEnvVar)
	apiKeyID := os.Getenv(utils.APIKeyIDEnvVar)
	apiKey := os.Getenv(utils.APIKeyEnvVar)
	if host == "" || apiKeyID == "" || apiKey == "" {
		return nil, fmt.Errorf("missing required environment variables for machine connection")
	}
	return client.New(
		ctx,
		host,
		logger,
		client.WithDialOptions(rpc.WithEntityCredentials(
			apiKeyID,
			rpc.Credentials{
				Type:    rpc.CredentialsTypeAPIKey,
				Payload: apiKey,
			},
		)),
	)
}

