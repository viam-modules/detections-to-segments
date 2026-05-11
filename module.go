// Package detectionstosegments uses a 2D segmenter and a camera that can project its images
// to 3D to project the bounding boxes to 3D in order to created a segmented point cloud.
package detectionstosegments

import (
	"context"
	"image"
	"math"
	"sync"

	"github.com/golang/geo/r3"
	"github.com/pkg/errors"
	"go.opencensus.io/trace"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/data"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/rimage"
	"go.viam.com/rdk/rimage/transform"
	servicevision "go.viam.com/rdk/services/vision"
	"go.viam.com/rdk/spatialmath"
	vision "go.viam.com/rdk/vision"
	"go.viam.com/rdk/vision/objectdetection"
	"go.viam.com/rdk/vision/segmentation"
)

var DetectionsToSegments = resource.NewModel("viam", "vision", "detections-to-segments")

// DetectionSegmenterConfig are the optional parameters to turn a detector into a segmenter.
type DetectionSegmenterConfig struct {
	DetectorName     string  `json:"detector_name"`
	ConfidenceThresh float64 `json:"confidence_threshold_pct"`
	MeanK            int     `json:"mean_k"`
	Sigma            float64 `json:"sigma"`
	DefaultCamera    string  `json:"camera_name"`
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
	detectorService, err := servicevision.FromProvider(deps, conf.DetectorName)
	if err != nil {
		return nil, errors.Wrapf(err, "could not find necessary dependency, detector %q", conf.DetectorName)
	}
	confThresh := 0.5 // default value
	if conf.ConfidenceThresh > 0.0 {
		confThresh = conf.ConfidenceThresh
	}
	detector := func(ctx context.Context, img image.Image) ([]objectdetection.Detection, error) {
		namedImg, err := camera.NamedImageFromImage(img, "", "", data.Annotations{})
		if err != nil {
			return nil, err
		}
		return detectorService.Detections(ctx, &namedImg, nil)
	}
	segmenter, err := DetectionSegmenter(objectdetection.Detector(detector), conf.MeanK, conf.Sigma, confThresh, logger)
	if err != nil {
		return nil, errors.Wrap(err, "cannot create 3D segmenter from detector")
	}
	if conf.DefaultCamera != "" {
		_, err = camera.FromProvider(deps, conf.DefaultCamera)
		if err != nil {
			return nil, errors.Errorf("could not find camera %q", conf.DefaultCamera)
		}
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

// DetectionSegmenter will take an objectdetector.Detector and turn it into a Segementer.
// The params for the segmenter are "mean_k" and "sigma" for the statistical filter on the point clouds.
//
// The native-pointcloud path (used when the camera advertises SupportsPCD) assumes the cloud returned by
// NextPointCloud is already registered to the color sensor's frame; extrinsics from the camera properties
// are deliberately not applied to avoid double-transforming pre-registered clouds (the common case for
// current Viam drivers like the RealSense). A one-shot INFO log surfaces this assumption at runtime.
func DetectionSegmenter(
	detector objectdetection.Detector,
	meanK int,
	sigma, confidenceThresh float64,
	logger logging.Logger,
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
	// nativeAssumptionOnce fires the first time the native-pointcloud path runs, so the
	// "cloud is assumed already in color frame" assumption is visible in machine logs.
	var nativeAssumptionOnce sync.Once
	// return the segmenter
	seg := func(ctx context.Context, src camera.Camera) ([]*vision.Object, error) {
		props, err := src.Properties(ctx)
		if err != nil {
			return nil, camera.NewPropertiesError("source camera")
		}
		// Use the camera's native point cloud when it supports one and we have intrinsics
		// to project bbox membership; otherwise reconstruct from RGB+depth.
		useNative := props.SupportsPCD && props.IntrinsicParams != nil

		imgs, _, err := src.Images(ctx, nil, nil)
		if err != nil {
			return nil, errors.Wrapf(err, "detection segmenter")
		}
		var img *rimage.Image
		var dmimg image.Image
		for _, i := range imgs {
			thisI := i
			switch thisI.SourceName {
			case "color":
				colorImg, err := thisI.Image(ctx)
				if err != nil {
					return nil, errors.Wrap(err, "decoding color image from camera")
				}
				img = rimage.ConvertImage(colorImg)
			case "depth":
				depthImg, err := thisI.Image(ctx)
				if err != nil {
					return nil, errors.Wrap(err, "decoding depth image from camera")
				}
				dmimg = depthImg
			}
		}
		if img == nil {
			return nil, errors.New("source camera's getImages method did not return a 'color' image")
		}

		var dm *rimage.DepthMap
		var proj transform.Projector
		var sourceCloud pointcloud.PointCloud
		if useNative {
			nativeAssumptionOnce.Do(func() {
				logger.Info("native point cloud path: assuming the cloud is already registered to the color frame; camera extrinsic parameters are not applied when projecting points back to the image plane")
			})
			sourceCloud, err = src.NextPointCloud(ctx, nil)
			if err != nil {
				return nil, errors.Wrap(err, "detection segmenter")
			}
		} else {
			if dmimg == nil {
				return nil, errors.New("source camera's getImages method did not have 'color' and 'depth' images")
			}
			dm, err = rimage.ConvertImageToDepthMap(ctx, dmimg)
			if err != nil {
				return nil, err
			}
			proj, err = cameraToProjector(ctx, src)
			if err != nil {
				return nil, err
			}
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
			var pc pointcloud.PointCloud
			if useNative {
				pc, err = filterPointCloudByDetection(sourceCloud, d, props.IntrinsicParams)
			} else {
				pc, err = detectionToPointCloud(d, img, dm, proj)
			}
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
			// if object was filtered away, skip it
			if pc.Size() == 0 {
				continue
			}
			obj, err := vision.NewObjectWithLabel(pc, d.Label(), nil)
			if err != nil {
				return nil, err
			}
			objects = append(objects, obj)
		}
		return objects, nil
	}
	return seg, nil
}

func detectionToPointCloud(
	d objectdetection.Detection,
	im *rimage.Image, dm *rimage.DepthMap,
	proj transform.Projector,
) (pointcloud.PointCloud, error) {
	bb := d.BoundingBox()
	if bb == nil {
		return nil, errors.New("detection bounding box cannot be nil")
	}
	pc, err := proj.RGBDToPointCloud(im, dm, *bb)
	if err != nil {
		return nil, err
	}
	return pc, nil
}

// filterPointCloudByDetection returns the subset of points whose projection onto the
// camera's image plane falls inside the detection's bounding box. Per-point Data is
// carried through unchanged, so color is preserved when the source cloud has it.
//
// Projection goes through the color intrinsics only — extrinsics are intentionally not
// applied here. We assume the source cloud is already registered to the color sensor's
// frame, which is the convention for current Viam camera drivers that produce native
// point clouds (e.g. RealSense). Applying extrinsics in that case would double-transform
// and shift projections by the depth↔color baseline. If a future driver returns a cloud
// in the depth frame, callers must transform it into the color frame before passing in.
func filterPointCloudByDetection(
	src pointcloud.PointCloud,
	d objectdetection.Detection,
	intrinsics *transform.PinholeCameraIntrinsics,
) (pointcloud.PointCloud, error) {
	bb := d.BoundingBox()
	if bb == nil {
		return nil, errors.New("detection bounding box cannot be nil")
	}
	if intrinsics == nil {
		return nil, errors.New("intrinsic parameters are required to filter a point cloud by 2D detection")
	}
	out := pointcloud.NewBasicEmpty()
	if src == nil {
		return out, nil
	}
	var iterErr error
	src.Iterate(0, 0, func(p r3.Vector, dat pointcloud.Data) bool {
		px, py := intrinsics.PointToPixel(p.X, p.Y, p.Z)
		pt := image.Point{X: int(math.Round(px)), Y: int(math.Round(py))}
		if !pt.In(*bb) {
			return true
		}
		if err := out.Set(p, dat); err != nil {
			iterErr = err
			return false
		}
		return true
	})
	if iterErr != nil {
		return nil, iterErr
	}
	return out, nil
}
