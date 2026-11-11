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
- Python environment with the bundled modified Ultralytics package installed
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

## Install the bundled YOLO-SIAF runtime

From the repository root, install the included modified Ultralytics package:

```bash
python3 -m pip install -r requirements.txt
python3 -m pip install -e .
```

The bundled runtime contains the custom `SPDConv`, `SIAF_EA` (AIFM) and
`C3_LTC` (C3k2-CAL) implementations required to deserialize and run the
checkpoint. Do not replace it with an unmodified PyPI Ultralytics package.

## Build the ROS 2 package

```bash
mkdir -p ~/yolo_siaf_ros2_ws/src
cp -r yolo_siaf_sim ~/yolo_siaf_ros2_ws/src/
cd ~/yolo_siaf_ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
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

## 中文运行指令（带注释）

以下指令适用于 Ubuntu 22.04、ROS 2 Humble 和 Gazebo Classic 11。所有
命令均在终端中执行，`#` 后面的内容是说明，不需要单独输入。

### 1. 下载项目

```bash
# 克隆GitHub仓库
git clone https://github.com/penny-xiong/ros2_car_yolo11.git

# 进入仓库根目录
cd ros2_car_yolo11
```

### 2. 安装ROS 2与Gazebo依赖

```bash
# 加载ROS 2 Humble环境
source /opt/ros/humble/setup.bash

# 更新软件包索引
sudo apt update

# 安装Gazebo接口、图像转换、消息同步和ROS 2编译工具
sudo apt install -y ros-humble-gazebo-ros-pkgs \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-cv-bridge \
  ros-humble-message-filters \
  ros-humble-image-transport \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-venv
```

### 3. 安装YOLO-SIAF运行环境

```bash
# 在仓库根目录创建虚拟环境；保留系统ROS 2 Python包的访问能力
python3 -m venv --system-site-packages .venv

# 激活虚拟环境
source .venv/bin/activate

# 更新pip并安装项目依赖
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements.txt

# 以可编辑方式安装仓库内修改后的Ultralytics
python3 -m pip install -e .
```

不要用普通PyPI版Ultralytics覆盖本项目代码，否则模型可能无法找到
`SPDConv`、`SIAF_EA` 和 `C3_LTC` 等自定义模块。

### 4. 创建并编译ROS 2工作空间

```bash
# 创建ROS 2工作空间
mkdir -p ~/yolo_siaf_ros2_ws/src

# 将功能包复制到工作空间；该命令在仓库根目录执行
cp -r yolo_siaf_sim ~/yolo_siaf_ros2_ws/src/

# 进入工作空间
cd ~/yolo_siaf_ros2_ws

# 加载ROS 2环境
source /opt/ros/humble/setup.bash

# 安装package.xml中声明的依赖
rosdep install --from-paths src --ignore-src -r -y

# 编译功能包
colcon build --symlink-install

# 加载本次编译结果
source install/setup.bash
```

### 5. 启动完整仿真

```bash
# 如果打开了新终端，先依次加载Python、ROS 2和工作空间环境
source ~/ros2_car_yolo11/.venv/bin/activate
source /opt/ros/humble/setup.bash
source ~/yolo_siaf_ros2_ws/install/setup.bash

# 启动夜间场景、小车、双相机、YOLO-SIAF和速度控制节点
ros2 launch yolo_siaf_sim simulation.launch.py
```

启动文件默认使用功能包内的 `weights/best.pt`。若需要测试其他权重：

```bash
# 用指定的模型权重替换默认权重
ros2 launch yolo_siaf_sim simulation.launch.py \
  model_path:=/absolute/path/to/best.pt
```

### 6. 检查传感器与检测结果

另开一个终端，并加载ROS 2工作空间：

```bash
source /opt/ros/humble/setup.bash
source ~/yolo_siaf_ros2_ws/install/setup.bash

# 查看当前ROS 2话题是否正常发布
ros2 topic list

# 查看RGB相机图像
rqt_image_view /sensors/rgb/image_raw

# 查看IR代理相机图像
rqt_image_view /sensors/ir/image_raw

# 查看带检测框、置信度和距离的结果图
rqt_image_view /yolo_siaf/annotated_image

# 查看最近行人的估算距离，单位为米
ros2 topic echo /yolo_siaf/nearest_pedestrian_distance

# 查看结构化检测记录
ros2 topic echo /yolo_siaf/detections

# 查看发送给小车的速度指令
ros2 topic echo /cmd_vel
```

### 7. 停止与重新运行

```bash
# 在启动仿真的终端中按Ctrl+C，停止全部节点

# 修改源码或参数后，在工作空间中重新编译
cd ~/yolo_siaf_ros2_ws
colcon build --symlink-install
source install/setup.bash

# 再次启动仿真
ros2 launch yolo_siaf_sim simulation.launch.py
```

### 8. 常见检查

```bash
# 确认ROS 2能够找到功能包
ros2 pkg prefix yolo_siaf_sim

# 确认GPU和驱动状态
nvidia-smi

# 确认PyTorch能够访问CUDA
python3 -c "import torch; print(torch.__version__, torch.cuda.is_available())"

# 确认加载的是仓库内的Ultralytics代码
python3 -c "import ultralytics; print(ultralytics.__file__)"
```

Gazebo Classic中的IR图像是与RGB相机同位置的灰度代理图像，不包含真实
热辐射信息。该配置用于验证双路同步、六通道推理和车辆减速闭环；真实
热红外部署时，应将 `/sensors/ir/image_raw` 替换为完成标定和配准的热成像
相机话题。

