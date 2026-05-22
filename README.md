# Detection to Segments Module

This module provides a vision service model that turns 2D detections into 3D point cloud segments.

For each call, it runs the configured detector on the camera's color image to obtain 2D bounding boxes, then produces a 3D point cloud per detection using one of two paths chosen automatically based on the camera's `Properties`:

- **Native point cloud (preferred):** if the camera advertises `SupportsPCD` and exposes intrinsic parameters, the module calls `NextPointCloud` and keeps the points whose projection onto the image plane (using the color camera's intrinsics) falls inside the detection's bounding box. This preserves the fidelity of cameras that produce their own registered/fused point clouds. Per-point color is carried through when the native cloud has it; segments will be uncolored otherwise. **Assumption:** the cloud returned by the driver is already registered to the color sensor's frame — extrinsic parameters from the camera properties are deliberately not applied, to avoid double-transforming pre-aligned clouds (the common case for current Viam drivers like the RealSense). This assumption is logged once per service instance the first time the native path runs. If a driver returns the cloud in the depth frame instead, segments will be shifted by the depth↔color baseline.
- **RGBD reconstruction (fallback):** if the camera doesn't support `NextPointCloud` (or has no intrinsics), the module falls back to the original behavior — fetching the color and depth images and reprojecting the pixels inside each bounding box to 3D via the camera's intrinsics. Segments are always colored from the RGB frame in this path.

The label of each segment is the label of the originating detection. If the chosen camera supports neither path (no `NextPointCloud` and no usable depth + intrinsics), this vision model will fail.

### Configuration
The following attribute template can be used to configure this model:

```json
{
  "sigma": 1.25,
  "camera_name": "realsense-camera",
  "detector_name": "color-detector-1",
  "confidence_threshold_pct": 0.5,
  "mean_k": 5
}
```

#### Attributes

The following attributes are available for this model:

| Name          | Type   | Inclusion | Description                |
|---------------|--------|-----------|----------------------------|
| `detector_name` | string  | Required  | The name of a registered detector vision service. The segmenter vision service uses the detections from "detector_name" to create the 3D segments. |
| `confidence_threshold_pct` | float | Optional  | A number between 0 and 1 which represents a filter on object confidence scores. Detections that score below the threshold will be filtered out in the segmenter. The default is 0.5. |
| `mean_k` | int | Required  | 	An integer parameter used in a subroutine to eliminate the noise in the point clouds. It should be set to be 5-10% of the minimum segment size. Start with 5% and go up if objects are still too noisy. If you don’t want to use the filtering, set the number to 0 or less. |
| `sigma` | float | Required  | A floating point parameter used in a subroutine to eliminate the noise in the point clouds. It should usually be set between 1.0 and 2.0. 1.25 is usually a good default. If you want the object result to be less noisy (at the risk of losing some data around its edges) set sigma to be lower. |
| `camera_name` | string | Required  | Name of the camera to use |

#### Example Camera Configuration
* Note that for a color-detector, the color string must come first in the sensors array.
```json
{
  "name": "realsense-camera",
  "api": "rdk:component:camera",
  "model": "viam:camera:realsense",
  "attributes": {
    "serial_number": "",
    "sensors": [
      "color",
      "depth"
    ],
    "width_px": 640,
    "height_px": 480,
    "little_endian_depth": false
  }
}
```

#### Example Module Configuration
```json
{
  "name": "detections-to-segments-new",
  "api": "rdk:service:vision",
  "model": "viam:vision:detections-to-segments",
  "attributes": {
    "sigma": 1.25,
    "camera_name": "realsense-camera",
    "detector_name": "color-detector-1",
    "confidence_threshold_pct": 0.5,
    "mean_k": 5
  }
}
```
