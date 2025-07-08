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
| `attribute_1` | float  | Required  | Description of attribute 1 |
| `attribute_2` | string | Optional  | Description of attribute 2 |

#### Example Configuration

```json
{
  "attribute_1": 1.0,
  "attribute_2": "foo"
}
```

### DoCommand

If your model implements DoCommand, provide an example payload of each command that is supported and the arguments that can be used. If your model does not implement DoCommand, remove this section.

#### Example DoCommand

```json
{
  "command_name": {
    "arg1": "foo",
    "arg2": 1
  }
}
```
