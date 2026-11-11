# Local Gazebo model archive

This directory is installed under `share/yolo_siaf_sim/models`. The launch file
prepends it to `GAZEBO_MODEL_PATH`, so every `model://` URI in the world resolves
locally.

| Directory | Role | Used by default world |
|---|---|---|
| `night_road` | 120 m road, edge lines, center line and sidewalks | yes |
| `street_lamp` | lamp geometry and local point-light source | yes, three instances |
| `yolo_siaf_car` | differential-drive car and synchronized RGB/IR-proxy sensors | yes |
| `rgb_ir_camera_rig` | standalone sensor model for topic and calibration tests | no |
| `pedestrian_actor` | local walking animation and preview mannequin | yes, two actors |

The vehicle is spawned from `yolo_siaf_car/model.sdf`. The Xacro file in the
package is retained as an editable robot-description reference, but it is not
spawned concurrently, which avoids duplicate Gazebo entities and TF parents.
