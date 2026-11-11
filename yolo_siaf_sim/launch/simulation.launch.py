import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("yolo_siaf_sim")
    gazebo_share = get_package_share_directory("gazebo_ros")
    world = os.path.join(package_share, "worlds", "night_straight_road.world")
    models_dir = os.path.join(package_share, "models")
    car_sdf = os.path.join(models_dir, "yolo_siaf_car", "model.sdf")
    params_file = os.path.join(package_share, "config", "sim_params.yaml")
    default_model_path = os.path.join(package_share, "weights", "best.pt")

    model_path = LaunchConfiguration("model_path")
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model_path",
                default_value=default_model_path,
                description="Absolute path to the trained YOLO-SIAF best.pt checkpoint",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            SetEnvironmentVariable(
                "GAZEBO_MODEL_PATH",
                models_dir + os.pathsep + os.environ.get("GAZEBO_MODEL_PATH", ""),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gazebo_share, "launch", "gazebo.launch.py")
                ),
                launch_arguments={"world": world, "gui": gui}.items(),
            ),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-entity",
                    "yolo_siaf_car",
                    "-file",
                    car_sdf,
                    "-x",
                    "0.0",
                    "-y",
                    "0.0",
                    "-z",
                    "0.0",
                ],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="rgb_camera_static_tf",
                arguments=[
                    "--x", "0.48", "--y", "0.0", "--z", "1.16",
                    "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                    "--frame-id", "base_link", "--child-frame-id", "rgb_camera_link",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="ir_camera_static_tf",
                arguments=[
                    "--x", "0.48", "--y", "0.0", "--z", "1.16",
                    "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                    "--frame-id", "base_link", "--child-frame-id", "ir_camera_link",
                ],
            ),
            Node(
                package="yolo_siaf_sim",
                executable="detector_node",
                name="yolo_siaf_detector",
                output="screen",
                parameters=[
                    params_file,
                    {"model_path": model_path, "use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="yolo_siaf_sim",
                executable="speed_controller",
                name="pedestrian_speed_controller",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            ),
        ]
    )
