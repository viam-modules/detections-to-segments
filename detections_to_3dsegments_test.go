package detectionstosegments

import (
	"context"
	"image"
	"image/color"
	"testing"
	"time"

	"github.com/pkg/errors"
	"go.viam.com/test"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/logging"
	pc "go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/rimage"
	"go.viam.com/rdk/services/vision"
	"go.viam.com/rdk/testutils/inject"
	"go.viam.com/rdk/vision/objectdetection"
)

type simpleDetector struct{}

func (s *simpleDetector) Detect(context.Context, image.Image) ([]objectdetection.Detection, error) {
	det1 := objectdetection.NewDetection(image.Rect(0, 0, 50, 50), image.Rect(10, 10, 20, 20), 0.5, "yes")
	return []objectdetection.Detection{det1}, nil
}

func Test3DSegmentsFromDetector(t *testing.T) {
	cam := &inject.Camera{}
	deps := make(resource.Dependencies)
	deps[camera.Named("fakeCamera")] = cam
	logger := logging.NewTestLogger(t)
	m := &simpleDetector{}

	detectorService, err := vision.NewService(vision.Named("testDetector"), deps, logger, nil, nil, m.Detect, nil, "fakeCamera")
	test.That(t, err, test.ShouldBeNil)
	deps[vision.Named("testDetector")] = detectorService

	cam.NextPointCloudFunc = func(ctx context.Context) (pc.PointCloud, error) {
		return nil, errors.New("no pointcloud")
	}
	cam.ImagesFunc = func(ctx context.Context) ([]camera.NamedImage, resource.ResponseMetadata, error) {
		return nil, resource.ResponseMetadata{}, errors.New("no images")
	}
	cam.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
		return camera.Properties{}, nil
	}

	params := &DetectionSegmenterConfig{
		DetectorName:     "testDetector",
		ConfidenceThresh: 0.2,
	}
	// bad registration, no parameters
	name2 := vision.Named("test_seg")
	_, err = register3DSegmenterFromDetector(context.Background(), name2, nil, deps, logger)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "cannot be nil")
	// bad registration, no such detector
	params.DetectorName = "noDetector"
	_, err = register3DSegmenterFromDetector(context.Background(), name2, params, deps, logger)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "could not find necessary dependency")
	// successful registration
	params.DetectorName = "testDetector"
	name3 := vision.Named("test_rcs")
	seg, err := register3DSegmenterFromDetector(context.Background(), name3, params, deps, logger)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, seg.Name(), test.ShouldResemble, name3)
	// successful registration, valid default camera
	params.DefaultCamera = "fakeCamera"
	seg, err = register3DSegmenterFromDetector(context.Background(), name3, params, deps, logger)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, seg.Name(), test.ShouldResemble, name3)
	// bad registration, invalid default camera
	params.DefaultCamera = "not-camera"
	_, err = register3DSegmenterFromDetector(context.Background(), name3, params, deps, logger)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "could not find camera \"not-camera\"")

	// fails on not finding camera
	_, err = seg.GetObjectPointClouds(context.Background(), "no_camera", map[string]interface{}{})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "Resource missing")

	// fails since camera cannot return images
	_, err = seg.GetObjectPointClouds(context.Background(), "fakeCamera", map[string]interface{}{})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "no images")

	// successful, creates one object with some points in it
	cam.ImagesFunc = func(ctx context.Context) ([]camera.NamedImage, resource.ResponseMetadata, error) {
		img := rimage.NewImage(150, 150)
		dm := rimage.NewEmptyDepthMap(150, 150)
		dm.Set(0, 0, rimage.Depth(5))
		dm.Set(0, 100, rimage.Depth(6))
		dm.Set(50, 0, rimage.Depth(8))
		dm.Set(50, 100, rimage.Depth(4))
		dm.Set(15, 15, rimage.Depth(3))
		dm.Set(16, 14, rimage.Depth(10))
		imgs := []camera.NamedImage{{img, "color"}, {dm, "depth"}}
		return imgs, resource.ResponseMetadata{CapturedAt: time.Now()}, nil
	}
	cam.NextPointCloudFunc = func(ctx context.Context) (pc.PointCloud, error) {
		cloud := pc.NewBasicEmpty()
		err = cloud.Set(pc.NewVector(0, 0, 5), pc.NewColoredData(color.NRGBA{255, 0, 0, 255}))
		test.That(t, err, test.ShouldBeNil)
		err = cloud.Set(pc.NewVector(0, 100, 6), pc.NewColoredData(color.NRGBA{255, 0, 0, 255}))
		test.That(t, err, test.ShouldBeNil)
		err = cloud.Set(pc.NewVector(50, 0, 8), pc.NewColoredData(color.NRGBA{255, 0, 0, 255}))
		test.That(t, err, test.ShouldBeNil)
		err = cloud.Set(pc.NewVector(50, 100, 4), pc.NewColoredData(color.NRGBA{255, 0, 0, 255}))
		test.That(t, err, test.ShouldBeNil)
		err = cloud.Set(pc.NewVector(15, 15, 3), pc.NewColoredData(color.NRGBA{255, 0, 0, 255}))
		test.That(t, err, test.ShouldBeNil)
		err = cloud.Set(pc.NewVector(16, 14, 10), pc.NewColoredData(color.NRGBA{255, 0, 0, 255}))
		test.That(t, err, test.ShouldBeNil)
		return cloud, nil
	}
	objects, err := seg.GetObjectPointClouds(context.Background(), "fakeCamera", map[string]interface{}{})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(objects), test.ShouldEqual, 1)
	test.That(t, objects[0].Size(), test.ShouldEqual, 2)
	// does  implement detector
	dets, err := seg.Detections(context.Background(), nil, nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(dets), test.ShouldEqual, 1)
	// does not implement classifier
	_, err = seg.Classifications(context.Background(), nil, 1, nil)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "does not implement")
}
