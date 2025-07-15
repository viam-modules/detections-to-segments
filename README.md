# Detection to Segments Module

This module provides a vision service model takes 2D bounding boxes from an object detector, and, using the intrinsic parameters of the chosen camera, projects the pixels in the bounding box to points in 3D space. If the chosen camera is not equipped to do projections from 2D to 3D, then this vision model will fail. The label and the pixels associated with the 2D detections become the label and point cloud associated with the 3D segmenter.

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
