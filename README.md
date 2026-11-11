# YOLO-SIAF ROS 2 nighttime pedestrian simulation

This workspace replaces the ROS 1 + Darknet YOLOv2 demonstration with a ROS 2
Humble perception-control loop for the local YOLO-SIAF implementation.

## System boundary

The simulation contains a differential-drive vehicle, a registered RGB/IR
camera pair, a straight night road, and two crossing pedestrians located at
approximately 20 m and 50 m from the initial vehicle pose. The detector fuses
the synchronized images into the six-channel layout expected by YOLO-SIAF. The
nearest pedestrian distance is estimated from the detected bounding-box height
and the calibrated RGB focal length. A separate controller maps that distance
to a smooth longitudinal-speed command.

Gazebo Classic does not simulate thermal radiation. The `/sensors/ir/image_raw`
stream is therefore a co-registered monochrome camera used as an **IR proxy**.
It is sufficient for testing ROS synchronization, six-channel inference and
closed-loop braking, but it is not evidence of performance on a physical
thermal camera. A thermal-valid experiment should replace this stream with a
Gazebo Sim thermal sensor or real registered RGB/thermal data.

## Target environment

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11
- Python environment containing this modified Ultralytics repository
- The bundled RGB+IR YOLO-SIAF checkpoint (`weights/best.pt`)

Install ROS dependencies:

```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-xacro \
  ros-humble-robot-state-publisher ros-humble-cv-bridge \
  ros-humble-message-filters ros-humble-image-transport
```

All scene resources are stored in `yolo_siaf_sim/models`; no online model
database is required at launch. The local model library contains:

- `night_road`: 120 m road, sidewalks and lane markings;
- `street_lamp`: pole, luminaire and point light;
- `yolo_siaf_car`: driveable vehicle with RGB/IR-proxy sensors;
- `rgb_ir_camera_rig`: standalone camera-pair model for sensor debugging;
- `pedestrian_actor`: locally archived animated walking mesh.

The pedestrian animation is attributed in
`models/pedestrian_actor/ATTRIBUTION.md` and is licensed under CC BY 4.0.

## Build

```bash
mkdir -p ~/yolo_siaf_ros2_ws/src
cp -r ros2_yolo_siaf_sim/yolo_siaf_sim ~/yolo_siaf_ros2_ws/src/
cd ~/yolo_siaf_ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Make the modified Ultralytics package importable before launch:

```bash
export PYTHONPATH=/absolute/path/to/YOLOv11-RGBT-master:$PYTHONPATH
```

## Run

```bash
ros2 launch yolo_siaf_sim simulation.launch.py
```

The launch file uses the bundled checkpoint by default. To test another
checkpoint, override it with `model_path:=/absolute/path/to/best.pt`.

The bundled checkpoint comes from `runs/detect/yolo11-GC10`, trained directly
with the complete `yolo11-6.yaml` RGB+IR architecture. Its highest recorded
LLVIP validation results are mAP50 = 0.97124 and mAP50-95 = 0.66280.

Important topics:

| Topic | Type | Purpose |
|---|---|---|
| `/sensors/rgb/image_raw` | `sensor_msgs/Image` | RGB image |
| `/sensors/ir/image_raw` | `sensor_msgs/Image` | monochrome IR proxy |
| `/yolo_siaf/detections` | `std_msgs/String` | JSON detection records |
| `/yolo_siaf/nearest_pedestrian_distance` | `std_msgs/Float32` | estimated nearest range |
| `/yolo_siaf/annotated_image` | `sensor_msgs/Image` | RGB detection overlay |
| `/cmd_vel` | `geometry_msgs/Twist` | vehicle speed command |

Visualize:

```bash
rqt_image_view /yolo_siaf/annotated_image
ros2 topic echo /yolo_siaf/nearest_pedestrian_distance
ros2 topic echo /cmd_vel
```

## Speed policy

The default controller cruises at 1.5 m/s. It begins proportional deceleration
inside 30 m, reaches the configured crawl speed near 8 m, and stops inside 5 m.
Commands are acceleration-limited and detections older than 0.5 s are ignored.
These values are simulation parameters rather than a certified braking policy.

## Experimental cautions

1. Keep RGB and IR topics synchronized and geometrically registered. The
   six-channel tensor order is RGB followed by IR converted to three channels.
2. Use the same resize and normalization as model training.
3. Bounding-box ranging assumes a 1.70 m pedestrian height; partial occlusion
   and unusual poses introduce error. Log Gazebo ground truth separately when
   evaluating range error, but do not feed it to the controller.
4. Record detection latency, range error, minimum clearance and stopping time.
   A successful visual demonstration alone is not a quantitative validation.

