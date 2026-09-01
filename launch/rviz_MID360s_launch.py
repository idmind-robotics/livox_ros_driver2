import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
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
user_config_path = os.path.join(cur_config_path, 'MID360s_config.json')

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

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"user_config_path": user_config_path},
    {"imu_frame_id": imu_frame_id},
    {"imu_publish_freq": imu_publish_freq},
    {"lidar_topic": lidar_topic},
    {"imu_topic": imu_topic},
    {"qos_reliability": qos_reliability},
    {"qos_depth": qos_depth},
    {"timestamp_source": timestamp_source},
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

    return LaunchDescription([
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
