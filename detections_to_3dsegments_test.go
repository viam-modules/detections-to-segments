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
	"go.viam.com/rdk/data"
	"go.viam.com/rdk/logging"
	pc "go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/rimage"
	"go.viam.com/rdk/rimage/transform"
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

	cam.NextPointCloudFunc = func(ctx context.Context, extra map[string]interface{}) (pc.PointCloud, error) {
		return nil, errors.New("no pointcloud")
	}
	cam.ImagesFunc = func(ctx context.Context, filterSourceNames []string, extra map[string]interface{}) ([]camera.NamedImage, resource.ResponseMetadata, error) {
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
	cam.ImagesFunc = func(ctx context.Context, filterSourceNames []string, extra map[string]interface{}) ([]camera.NamedImage, resource.ResponseMetadata, error) {
		img := rimage.NewImage(150, 150)
		dm := rimage.NewEmptyDepthMap(150, 150)
		dm.Set(0, 0, rimage.Depth(5))
		dm.Set(0, 100, rimage.Depth(6))
		dm.Set(50, 0, rimage.Depth(8))
		dm.Set(50, 100, rimage.Depth(4))
		dm.Set(15, 15, rimage.Depth(3))
		dm.Set(16, 14, rimage.Depth(10))
		colorNI, err := camera.NamedImageFromImage(img, "color", "", data.Annotations{})
		test.That(t, err, test.ShouldBeNil)
		depthNI, err := camera.NamedImageFromImage(dm, "depth", "", data.Annotations{})
		test.That(t, err, test.ShouldBeNil)
		imgs := []camera.NamedImage{colorNI, depthNI}
		return imgs, resource.ResponseMetadata{CapturedAt: time.Now()}, nil
	}
	cam.NextPointCloudFunc = func(ctx context.Context, extra map[string]interface{}) (pc.PointCloud, error) {
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
	// does implement detector
	dets, err := seg.Detections(context.Background(), nil, nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(dets), test.ShouldEqual, 1)
	// does not implement classifier
	_, err = seg.Classifications(context.Background(), nil, 1, nil)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "does not implement")
}

func TestDepthFilteredPointCloud(t *testing.T) {
	intrinsics := &transform.PinholeCameraIntrinsics{
		Width:  100,
		Height: 100,
		Fx:     100,
		Fy:     100,
		Ppx:    50,
		Ppy:    50,
	}

	t.Run("skips zero-depth pixels", func(t *testing.T) {
		img := rimage.NewImage(100, 100)
		dm := rimage.NewEmptyDepthMap(100, 100)
		// Only set depth for 2 pixels, rest are zero
		dm.Set(25, 25, rimage.Depth(500))
		dm.Set(30, 30, rimage.Depth(510))

		bb := image.Rect(10, 10, 50, 50)
		cloud, err := depthFilteredPointCloud(bb, img, dm, intrinsics, 0)
		test.That(t, err, test.ShouldBeNil)
		// Only 2 points should be projected (zero-depth pixels are skipped)
		test.That(t, cloud.Size(), test.ShouldEqual, 2)
		// Verify no point at origin (0,0,0)
		_, found := cloud.At(0, 0, 0)
		test.That(t, found, test.ShouldBeFalse)
	})

	t.Run("filters background by depth threshold", func(t *testing.T) {
		img := rimage.NewImage(100, 100)
		dm := rimage.NewEmptyDepthMap(100, 100)
		// Foreground object at ~500mm
		dm.Set(20, 20, rimage.Depth(490))
		dm.Set(21, 21, rimage.Depth(500))
		dm.Set(22, 22, rimage.Depth(510))
		// Background at 3000mm
		dm.Set(30, 30, rimage.Depth(3000))
		dm.Set(31, 31, rimage.Depth(3100))

		bb := image.Rect(10, 10, 50, 50)
		cloud, err := depthFilteredPointCloud(bb, img, dm, intrinsics, 200)
		test.That(t, err, test.ShouldBeNil)
		// Only the 3 foreground points should remain (median ~500, threshold 200)
		test.That(t, cloud.Size(), test.ShouldEqual, 3)
	})

	t.Run("no filtering when threshold is zero", func(t *testing.T) {
		img := rimage.NewImage(100, 100)
		dm := rimage.NewEmptyDepthMap(100, 100)
		dm.Set(20, 20, rimage.Depth(500))
		dm.Set(30, 30, rimage.Depth(3000))

		bb := image.Rect(10, 10, 50, 50)
		cloud, err := depthFilteredPointCloud(bb, img, dm, intrinsics, 0)
		test.That(t, err, test.ShouldBeNil)
		// Both points kept when threshold is disabled
		test.That(t, cloud.Size(), test.ShouldEqual, 2)
	})

	t.Run("returns empty cloud when all depths are zero", func(t *testing.T) {
		img := rimage.NewImage(100, 100)
		dm := rimage.NewEmptyDepthMap(100, 100)

		bb := image.Rect(10, 10, 50, 50)
		cloud, err := depthFilteredPointCloud(bb, img, dm, intrinsics, 0)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, cloud.Size(), test.ShouldEqual, 0)
	})
}

func TestMinPointsFiltering(t *testing.T) {
	detector := func(_ context.Context, _ image.Image) ([]objectdetection.Detection, error) {
		det := objectdetection.NewDetection(image.Rect(0, 0, 50, 50), image.Rect(10, 10, 20, 20), 0.8, "obj")
		return []objectdetection.Detection{det}, nil
	}
	// minPoints=100 should filter out segments with fewer than 100 points
	seg, err := DetectionSegmenter(objectdetection.Detector(detector), 1, 1.0, 0.5, 0, 100, 5.0, nil, "")
	test.That(t, err, test.ShouldBeNil)

	cam := &inject.Camera{}
	cam.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
		return camera.Properties{}, nil // no intrinsics → ParallelProjection
	}
	cam.ImagesFunc = func(ctx context.Context, filterSourceNames []string, extra map[string]interface{}) ([]camera.NamedImage, resource.ResponseMetadata, error) {
		img := rimage.NewImage(50, 50)
		dm := rimage.NewEmptyDepthMap(50, 50)
		// Only 2 points with depth — fewer than minPoints=100
		dm.Set(15, 15, rimage.Depth(500))
		dm.Set(16, 14, rimage.Depth(510))
		colorNI, err := camera.NamedImageFromImage(img, "color", "", data.Annotations{})
		test.That(t, err, test.ShouldBeNil)
		depthNI, err := camera.NamedImageFromImage(dm, "depth", "", data.Annotations{})
		test.That(t, err, test.ShouldBeNil)
		return []camera.NamedImage{colorNI, depthNI}, resource.ResponseMetadata{CapturedAt: time.Now()}, nil
	}

	objects, err := seg(context.Background(), cam)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(objects), test.ShouldEqual, 0) // filtered out by minPoints
}

func TestPinholeDeprojection(t *testing.T) {
	intrinsics := &transform.PinholeCameraIntrinsics{
		Width:  100,
		Height: 100,
		Fx:     100,
		Fy:     100,
		Ppx:    50,
		Ppy:    50,
	}

	detector := func(_ context.Context, _ image.Image) ([]objectdetection.Detection, error) {
		det := objectdetection.NewDetection(image.Rect(0, 0, 100, 100), image.Rect(10, 10, 40, 40), 0.9, "thing")
		return []objectdetection.Detection{det}, nil
	}
	// Use meanK=1, sigma=10.0 (very lenient filter) so points aren't filtered away
	seg, err := DetectionSegmenter(objectdetection.Detector(detector), 1, 10.0, 0.5, 0, 0, 0, nil, "")
	test.That(t, err, test.ShouldBeNil)

	cam := &inject.Camera{}
	cam.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
		return camera.Properties{IntrinsicParams: intrinsics}, nil
	}
	cam.ImagesFunc = func(ctx context.Context, filterSourceNames []string, extra map[string]interface{}) ([]camera.NamedImage, resource.ResponseMetadata, error) {
		img := rimage.NewImage(100, 100)
		dm := rimage.NewEmptyDepthMap(100, 100)
		// Set depth values inside the detection box; most pixels have zero depth.
		// With pinhole deprojection, only these 5 non-zero pixels should be projected.
		dm.Set(20, 20, rimage.Depth(1000))
		dm.Set(21, 21, rimage.Depth(1000))
		dm.Set(22, 22, rimage.Depth(1000))
		dm.Set(25, 25, rimage.Depth(1000))
		dm.Set(30, 30, rimage.Depth(1000))
		colorNI, err := camera.NamedImageFromImage(img, "color", "", data.Annotations{})
		test.That(t, err, test.ShouldBeNil)
		depthNI, err := camera.NamedImageFromImage(dm, "depth", "", data.Annotations{})
		test.That(t, err, test.ShouldBeNil)
		return []camera.NamedImage{colorNI, depthNI}, resource.ResponseMetadata{CapturedAt: time.Now()}, nil
	}

	objects, err := seg(context.Background(), cam)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(objects), test.ShouldEqual, 1)
	// Only 5 points should be in the cloud (zero-depth pixels skipped)
	test.That(t, objects[0].Size(), test.ShouldEqual, 5)
}
