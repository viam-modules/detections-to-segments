# Detection to Segments Module

This module provides a vision service that takes 2D bounding boxes from an object detector and, using the intrinsic parameters of the chosen camera, projects the pixels in the bounding box to points in 3D space. If the chosen camera is not equipped to do projections from 2D to 3D, then this vision model will fail. The label and the pixels associated with the 2D detections become the label and point cloud associated with the 3D segmenter.

The module also applies several filtering steps to produce clean 3D segments:
- **Depth gap cutoff**: removes far-away background points (walls, floors) by detecting large gaps in the depth distribution.
- **RANSAC plane removal**: detects and removes the dominant flat surface (e.g. a table) from each segment.
- **Clustering**: optionally splits disconnected point clusters into separate objects with the same label.
- **Frame transform**: if connected to a machine, transforms point clouds from the camera frame to the world frame using the camera's extrinsic parameters.

### Configuration
The following attribute template can be used to configure this model:

```json
{
  "detector_name": "color-detector-1",
  "camera_name": "realsense-camera",
  "mean_k": 5,
  "sigma": 1.25,
  "confidence_threshold_pct": 0.5,
  "depth_threshold_mm": 0,
  "min_points_in_segment": 0,
  "clustering_radius_mm": 0
}
```

#### Example Camera Configuration
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
  "name": "detections-to-segments",
  "api": "rdk:service:vision",
  "model": "viam:vision:detections-to-segments",
  "attributes": {
    "detector_name": "color-detector-1",
    "camera_name": "realsense-camera",
    "mean_k": 5,
    "sigma": 1.25,
    "confidence_threshold_pct": 0.5,
    "depth_threshold_mm": 100,
    "min_points_in_segment": 50,
    "clustering_radius_mm": 10
  }
}
```

##### Attributes

| Name | Type | Required | Default | Description |
|------|------|----------|---------|-------------|
| `detector_name` | string | Yes | - | The name of a registered detector vision service. The segmenter uses the detections from this detector to create the 3D segments. |
| `camera_name` | string | Yes | - | Name of the camera to use. Must provide both color and depth images, and must have intrinsic parameters configured. |
| `mean_k` | int | Yes | - | Parameter for the statistical outlier filter on point clouds. Should be set to 5-10% of the minimum segment size. Set to 0 or less to disable filtering. |
| `sigma` | float | Yes | - | Parameter for the statistical outlier filter. Usually set between 1.0 and 2.0. Lower values produce less noisy results at the risk of losing edge data. 1.25 is a good starting point. |
| `confidence_threshold_pct` | float | No | `0.5` | A number between 0 and 1. Detections scoring below this threshold are filtered out. |
| `depth_threshold_mm` | int | No | `0` | Maximum allowed deviation (in mm) from the median depth within a bounding box. Points outside this range are filtered out. Set to 0 to disable. |
| `min_points_in_segment` | int | No | `0` | Minimum number of points required for a valid segment. Segments (and clusters, if clustering is enabled) with fewer points are discarded. Set to 0 to disable. |
| `clustering_radius_mm` | float | No | `0` | Radius (in mm) for nearest-neighbor clustering. After plane removal, disconnected groups of points are split into separate objects (each keeping the same detection label). Clusters smaller than `min_points_in_segment` are discarded. Set to 0 to disable clustering and return one object per detection. |
