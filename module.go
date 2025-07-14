// Package detectionstosegments uses a 2D segmenter and a camera that can project its images
// to 3D to project the bounding boxes to 3D in order to created a segmented point cloud.
package detectionstosegments

import (
	"context"
	"image"

	"github.com/go-viper/mapstructure/v2"
	"github.com/pkg/errors"
	"go.opencensus.io/trace"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/rimage"
	"go.viam.com/rdk/rimage/transform"
	servicevision "go.viam.com/rdk/services/vision"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/utils"
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
	segmenter, err := DetectionSegmenter(objectdetection.Detector(detector), conf.MeanK, conf.Sigma, confThresh)
	if err != nil {
		return nil, errors.Wrap(err, "cannot create 3D segmenter from detector")
	}
	if conf.DefaultCamera != "" {
		_, err = camera.FromDependencies(deps, conf.DefaultCamera)
		if err != nil {
			return nil, errors.Errorf("could not find camera %q", conf.DefaultCamera)
		}
	}
	return servicevision.NewService(name, deps, logger, nil, nil, detector, segmenter, conf.DefaultCamera)
}

func (conf *DetectionSegmenterConfig) Validate(path string) ([]string, []string, error) {
	var deps []string
	var warnings []string

	if conf.DefaultCamera != "" {
		deps = append(deps, conf.DefaultCamera)
	}

	if conf.DetectorName == "" {
		return nil, warnings, errors.Errorf("expected a detector to be specified")
	}
	deps = append(deps, conf.DetectorName)

	if conf.MeanK <= 0 {
		return nil, warnings, errors.Errorf("expected a mean k to be specified")
	}

	if conf.Sigma <= 0 {
		return nil, warnings, errors.Errorf("expected a sigma to be specified")
	}

	return deps, warnings, nil
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

// DetectionSegmenter will take an objectdetector.Detector and turn it into a Segementer.
// The params for the segmenter are "mean_k" and "sigma" for the statistical filter on the point clouds.
func DetectionSegmenter(detector objectdetection.Detector, meanK int, sigma, confidenceThresh float64) (segmentation.Segmenter, error) {
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
	// return the segmenter
	seg := func(ctx context.Context, src camera.Camera) ([]*vision.Object, error) {
		proj, err := cameraToProjector(ctx, src)
		if err != nil {
			return nil, err
		}
		// get the 3D detections, and turn them into 2D image and depthmap
		imgs, _, err := src.Images(ctx)
		if err != nil {
			return nil, errors.Wrapf(err, "detection segmenter")
		}
		var img *rimage.Image
		var dmimg image.Image
		for _, i := range imgs {
			thisI := i
			if i.SourceName == "color" {
				img = rimage.ConvertImage(thisI.Image)
			}
			if i.SourceName == "depth" {
				dmimg = thisI.Image
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
			// TODO(bhaney): Is there a way to just project the detection boxes themselves?
			pc, err := detectionToPointCloud(d, img, dm, proj)
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
