import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import launch

################### user configure parameters for ros2 start ###################
xfer_format   = 0    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 'livox_frame'

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
rviz_config_path = os.path.join(cur_config_path, 'display_point_cloud_ROS2.rviz')
user_config_path = os.path.join(cur_config_path, 'MID360_config.json')

# Added by the idmind review. Each default reproduces the driver's previous
# hardcoded behaviour, so leaving them alone changes nothing.
imu_frame_id     = ''        # '' means "same as frame_id"
imu_publish_freq = 0.0       # 0.0 publishes every IMU sample (~200Hz)
lidar_topic      = 'livox/lidar'
imu_topic        = 'livox/imu'
qos_reliability  = 'reliable'    # 'reliable' or 'best_effort'
qos_depth        = 0             # 0 keeps the 64 (per-lidar) / 256 (shared) defaults
timestamp_source = 'lidar'       # 'lidar' or 'ros'
################### user configure parameters for ros2 end #####################

# Every entry below is both a launch argument and a node parameter, so any of
# them can be overridden on the command line without editing this file:
#
#   ros2 launch livox_ros_driver2 rviz_MID360_launch.py \
#       user_config_path:=/absolute/path/to/my_lidar.json frame_id:=livox_a
#
# The values above are only the defaults.
default_params = {
    'xfer_format': xfer_format,
    'multi_topic': multi_topic,
    'data_src': data_src,
    'publish_freq': publish_freq,
    'output_data_type': output_type,
    'frame_id': frame_id,
    'user_config_path': user_config_path,
    'imu_frame_id': imu_frame_id,
    'imu_publish_freq': imu_publish_freq,
    'lidar_topic': lidar_topic,
    'imu_topic': imu_topic,
    'qos_reliability': qos_reliability,
    'qos_depth': qos_depth,
    'timestamp_source': timestamp_source,
}

# One-line help for each, shown by 'ros2 launch <file> -s'.
param_help = {
    'xfer_format': '0 = PointCloud2 (PointXYZRTLT), 1 = Livox CustomMsg',
    'multi_topic': '0 = all LiDARs share one topic, 1 = one topic per LiDAR',
    'data_src': 'only 0 (raw lidar) is supported',
    'publish_freq': 'point cloud publish rate in Hz, 0.5 to 100',
    'output_data_type': 'only 0 (publish to ROS) is supported',
    'frame_id': 'frame_id on the point cloud messages',
    'user_config_path': 'absolute path to the LiDAR JSON config',
    'imu_frame_id': "frame_id on the IMU messages; empty means same as frame_id",
    'imu_publish_freq': 'max IMU rate in Hz; 0.0 publishes every sample (~200Hz)',
    'lidar_topic': 'base topic for point cloud data',
    'imu_topic': 'base topic for IMU data',
    'qos_reliability': "'reliable' or 'best_effort'",
    'qos_depth': 'publisher queue depth; 0 keeps the 64/256 defaults',
    'timestamp_source': "'lidar' for the device clock, 'ros' for the node clock",
}

livox_ros2_params = [
    {name: ParameterValue(LaunchConfiguration(name), value_type=type(default))}
    for name, default in default_params.items()
]

launch_args = [
    DeclareLaunchArgument(name, default_value=str(default),
                          description=param_help[name])
    for name, default in default_params.items()
]


def generate_launch_description():
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
        )

    livox_rviz = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['--display-config', rviz_config_path]
        )

    return LaunchDescription(launch_args + [
        livox_driver,
        livox_rviz,
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=livox_rviz,
        #         on_exit=[
        #             launch.actions.EmitEvent(event=launch.events.Shutdown()),
        #         ]
        #     )
        # )
    ])
