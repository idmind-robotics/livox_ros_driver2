# Livox ROS Driver 2

Livox ROS Driver 2 is the 2nd-generation driver package used to connect LiDAR products produced by Livox. This branch is ROS2-only (humble recommended).

  **Note :**

  As a debugging tool, Livox ROS Driver is not recommended for mass production but limited to test scenarios. You should optimize the code based on the original source to meet your various needs.

## 1. Preparation

### 1.1 OS requirements

  * Ubuntu 20.04 for ROS2 Foxy;
  * Ubuntu 22.04 for ROS2 Humble;
  * Ubuntu 24.04 for ROS2 Jazzy;

  **Tips:**

  Colcon is a build tool used in ROS2.

  How to install colcon: [Colcon installation instructions](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)

### 1.2 Install ROS2

For ROS Foxy installation, please refer to:
[ROS Foxy installation instructions](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

For ROS Humble installation, please refer to:
[ROS Humble installation instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

For ROS Jazzy installation, please refer to:
[ROS Jazzy installation instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html)

Desktop-Full installation is recommend.

## 2. Build & Run Livox ROS Driver 2

### 2.1 Clone Livox ROS Driver 2 source code:

```shell
git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2
```

  **Note :**

  Be sure to clone the source code in a '[work_space]/src/' folder (as shown above), otherwise compilation errors will occur due to the compilation tool restriction.

### 2.2 Build & install the Livox-SDK2

  **Note :**

  Please follow the guidance of installation in the [Livox-SDK2/README.md](https://github.com/Livox-SDK/Livox-SDK2/blob/master/README.md)

### 2.3 Build the Livox ROS Driver 2:

Build it like any other ament package, from the workspace root:

```shell
source /opt/ros/humble/setup.bash
colcon build --packages-select livox_ros_driver2
```

### 2.4 Run Livox ROS Driver 2:

```shell
source install/setup.bash
ros2 launch livox_ros_driver2 [launch file]
```

in which,

* **livox_ros_driver2** : is the ROS2 package name of Livox ROS Driver 2;
* **[launch file]** : is the launch file you want to use; the 'launch' folder contains several launch samples for your reference.

An rviz launch example for HAP LiDAR would be:

```shell
ros2 launch livox_ros_driver2 rviz_HAP_launch.py
```

## 3. Launch file and livox_ros_driver2 internal parameter configuration instructions

### 3.1 Launch file configuration instructions

Launch files are in the "ws_livox/src/livox_ros_driver2/launch" directory. Different launch files have different configuration parameter values and are used in different scenarios:

| launch file name           | Description                                                  |
| -------------------------- | ------------------------------------------------------------ |
| rviz_HAP_launch.py         | Connect to HAP LiDAR device<br>Publish pointcloud2 format data<br>Autoload rviz |
| msg_HAP_launch.py          | Connect to HAP LiDAR device<br>Publish livox customized pointcloud data |
| rviz_MID360_launch.py      | Connect to MID360 LiDAR device<br>Publish pointcloud2 format data<br>Autoload rviz |
| msg_MID360_launch.py       | Connect to MID360 LiDAR device<br>Publish livox customized pointcloud data |
| rviz_MID360s_launch.py     | Connect to MID360s LiDAR device<br>Publish pointcloud2 format data<br>Autoload rviz |
| msg_MID360s_launch.py      | Connect to MID360s LiDAR device<br>Publish livox customized pointcloud data |
| rviz_AVIA2_launch.py       | Connect to Avia2 LiDAR device<br>Publish pointcloud2 format data<br>Autoload rviz |
| msg_AVIA2_launch.py        | Connect to Avia2 LiDAR device<br>Publish livox customized pointcloud data |
| rviz_mixed_launch.py       | Connect to HAP and MID360 LiDAR device<br>Publish pointcloud2 format data<br>Autoload rviz |

### 3.2 Livox ros driver 2 internal main parameter configuration instructions

All internal parameters of Livox_ros_driver2 are in the launch file. Below are detailed descriptions of the three commonly used parameters :

| Parameter    | Detailed description                                         | Default |
| ------------ | ------------------------------------------------------------ | ------- |
| publish_freq | Set the frequency of point cloud publish <br>Floating-point data type, recommended values 5.0, 10.0, 20.0, 50.0, etc. The maximum publish frequency is 100.0 Hz.| 10.0    |
| multi_topic  | If the LiDAR device has an independent topic to publish pointcloud data<br>0 -- All LiDAR devices use the same topic to publish pointcloud data<br>1 -- Each LiDAR device has its own topic to publish point cloud data | 0       |
| xfer_format  | Set pointcloud format<br>0 -- Livox pointcloud2(PointXYZRTLT) pointcloud format<br>1 -- Livox customized pointcloud format | 0       |

#### Parameters added in this fork

Every one of these defaults to the driver's previous hardcoded behaviour, so an
existing launch file that does not set them behaves exactly as before.

| Parameter | Detailed description | Default |
| --- | --- | --- |
| imu_frame_id | `frame_id` for the IMU messages. Empty means "same as `frame_id`". Previously the IMU frame was the literal `livox_frame` regardless of `frame_id`, which put the cloud and the IMU in different frames. | `''` |
| imu_publish_freq | Maximum IMU publish rate in Hz. `0.0` publishes every sample the device sends (~200 Hz). | 0.0 |
| lidar_topic | Base topic for point cloud data. With `multi_topic:=1` the device IP is appended, e.g. `livox/lidar_192_168_1_12`. | `livox/lidar` |
| imu_topic | Base topic for IMU data, same suffix rule. | `livox/imu` |
| qos_reliability | `reliable` or `best_effort`. `best_effort` matches RViz's default subscription QoS and suits high-rate cloud over Ethernet. | `reliable` |
| qos_depth | Publisher queue depth. `0` keeps the historical depths (64 per-lidar, 256 shared). | 0 |
| timestamp_source | `lidar` stamps messages with the device timestamp. `ros` stamps with the node clock, which is what you want under `use_sim_time`, or when the lidar is not PTP/GPS synchronised (the driver then falls back to a non-ROS clock). | `lidar` |

  **Note :**

`xfer_format:=2` (pcl::PointCloud), `output_data_type:=1` (rosbag) and
`data_src` other than `0` were ROS1-only. They are now rejected at startup with
an explanatory message, rather than being accepted and then publishing nothing.
Use `ros2 bag record` for recording.

  **Note :**

Other parameters not mentioned in these tables are not suggested to be changed unless fully understood.

&ensp;&ensp;&ensp;&ensp;***Livox_ros_driver2 pointcloud data detailed description :***

1. Livox pointcloud2 (PointXYZRTLT) point cloud format, as follows :

```c
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
float32 intensity       # the value is reflectivity, 0.0~255.0
uint8   tag             # livox tag
uint8   line            # laser number in lidar
float64 timestamp       # Timestamp of point
```
  **Note :**

  The number of points in the frame may be different, but each point provides a timestamp.

2. Livox customized data package format, as follows :

```c
std_msgs/Header header     # ROS standard message header
uint64          timebase   # The time of first point
uint32          point_num  # Total number of pointclouds
uint8           lidar_id   # Lidar device id number
uint8[3]        rsvd       # Reserved use
CustomPoint[]   points     # Pointcloud data
```

&ensp;&ensp;&ensp;&ensp;Customized Point Cloud (CustomPoint) format in the above customized data package :

```c
uint32  offset_time     # offset time relative to the base time
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
uint8   reflectivity    # reflectivity, 0~255
uint8   tag             # livox tag
uint8   line            # laser number in lidar
```

## 4. LiDAR config

LiDAR Configurations (such as ip, port, data type... etc.) can be set via a json-style config file. Config files for single HAP, Mid360, Mid360s, Avia2 and mixed-LiDARs are in the "config" folder. The parameter naming *'user_config_path'* in launch files indicates such json file path.

1. Follow is a configuration example for HAP LiDAR (located in config/HAP_config.json):

```json
{
  "lidar_summary_info" : {
    "lidar_type": 8  # protocol type index, please don't revise this value
  },
  "HAP": {
    "device_type" : "HAP",
    "lidar_ipaddr": "",
    "lidar_net_info" : {
      "cmd_data_port": 56000,  # command port
      "push_msg_port": 0,
      "point_data_port": 57000,
      "imu_data_port": 58000,
      "log_data_port": 59000
    },
    "host_net_info" : {
      "cmd_data_ip" : "192.168.1.5",  # host ip (it can be revised)
      "cmd_data_port": 56000,
      "push_msg_ip": "",
      "push_msg_port": 0,
      "point_data_ip": "192.168.1.5",  # host ip
      "point_data_port": 57000,
      "imu_data_ip" : "192.168.1.5",  # host ip
      "imu_data_port": 58000,
      "log_data_ip" : "",
      "log_data_port": 59000
    }
  },
  "lidar_configs" : [
    {
      "ip" : "192.168.1.100",  # ip of the LiDAR you want to config
      "pcl_data_type" : 1,
      "pattern_mode" : 0,
      "blind_spot_set" : 50,
      "extrinsic_parameter" : {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    }
  ]
}
```

The parameter attributes in the above json file are described in the following table :

**LiDAR configuration parameter**
| Parameter                  | Type    | Description                                                  | Default         |
| :------------------------- | ------- | ------------------------------------------------------------ | --------------- |
| ip             | String  | Ip of the LiDAR you want to config | 192.168.1.100 |
| pcl_data_type             | Int | Choose the resolution of the point cloud data to send<br>1 -- Cartesian coordinate data (32 bits)<br>2 -- Cartesian coordinate data (16 bits) <br>3 --Spherical coordinate data| 1           |
| pattern_mode                | Int     | Space scan pattern<br>0 -- non-repeating scanning pattern mode<br>1 -- repeating scanning pattern mode <br>2 -- repeating scanning pattern mode (low scanning rate) | 0               |
| blind_spot_set (Only for HAP LiDAR)                 | Int     | Set blind spot<br>Range from 50 cm to 200 cm               | 50               |
| extrinsic_parameter |      | Set extrinsic parameter<br> The data types of "roll" "picth" "yaw" are float <br>  The data types of "x" "y" "z" are int<br>               |

For more infomation about the HAP config, please refer to:
[HAP Config File Description](https://github.com/Livox-SDK/Livox-SDK2/wiki/hap-config-file-description)

2. Follow is a configuration example for Avia2 LiDAR (located in config/AVIA2_config.json). The Mid360 and Mid360s LiDARs use the same port numbers and config structure, only with a different device key ("MID360" / "Mid360s", see config/MID360_config.json and config/MID360s_config.json):

```json
{
  "lidar_summary_info" : {
    "lidar_type": 8
  },
  "Avia2": {
    "lidar_net_info" : {
      "cmd_data_port"  : 56100,
      "push_msg_port"  : 56200,
      "point_data_port": 56300,
      "imu_data_port"  : 56400,
      "log_data_port"  : 56500
    },
    "host_net_info" : [
      {
        "host_ip"        : "192.168.1.5",
        "cmd_data_port"  : 56101,
        "push_msg_port"  : 56201,
        "point_data_port": 56301,
        "imu_data_port"  : 56401,
        "log_data_port"  : 56501
      }
    ]
  },
  "lidar_configs" : [
    {
      "ip" : "192.168.1.12",
      "pcl_data_type" : 1,
      "extrinsic_parameter" : {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    }
  ]
}
```

3. When connecting multiple LiDARs, add objects corresponding to different LiDARs to the "lidar_configs" array. Examples of mixed-LiDARs config file contents are as follows :

```json
{
  "lidar_summary_info" : {
    "lidar_type": 8  # protocol type index, please don't revise this value
  },
  "HAP": {
    "lidar_net_info" : {  # HAP ports, please don't revise these values
      "cmd_data_port": 56000,  # HAP command port
      "push_msg_port": 0,
      "point_data_port": 57000,
      "imu_data_port": 58000,
      "log_data_port": 59000
    },
    "host_net_info" : {
      "cmd_data_ip" : "192.168.1.5",  # host ip
      "cmd_data_port": 56000,
      "push_msg_ip": "",
      "push_msg_port": 0,
      "point_data_ip": "192.168.1.5",  # host ip
      "point_data_port": 57000,
      "imu_data_ip" : "192.168.1.5",  # host ip
      "imu_data_port": 58000,
      "log_data_ip" : "",
      "log_data_port": 59000
    }
  },
  "MID360": {
    "lidar_net_info" : {  # Mid360 ports, please don't revise these values
      "cmd_data_port": 56100,  # Mid360 command port
      "push_msg_port": 56200,
      "point_data_port": 56300,
      "imu_data_port": 56400,
      "log_data_port": 56500
    },
    "host_net_info" : {
      "cmd_data_ip" : "192.168.1.5",  # host ip
      "cmd_data_port": 56101,
      "push_msg_ip": "192.168.1.5",  # host ip
      "push_msg_port": 56201,
      "point_data_ip": "192.168.1.5",  # host ip
      "point_data_port": 56301,
      "imu_data_ip" : "192.168.1.5",  # host ip
      "imu_data_port": 56401,
      "log_data_ip" : "",
      "log_data_port": 56501
    }
  },
  "lidar_configs" : [
    {
      "ip" : "192.168.1.100",  # ip of the HAP you want to config
      "pcl_data_type" : 1,
      "pattern_mode" : 0,
      "blind_spot_set" : 50,
      "extrinsic_parameter" : {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    },
    {
      "ip" : "192.168.1.12",  # ip of the Mid360 you want to config
      "pcl_data_type" : 1,
      "pattern_mode" : 0,
      "extrinsic_parameter" : {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    }
  ]
}
```
4. when multiple nics on the host connect to multiple LiDARs, you need to add objects corresponding to different LiDARs to the lidar_configs array. Run different luanch files separately, and the following is an example of mixing lidar configuration file contents:

**MID360_config1:**
```json
{
  "lidar_summary_info" : {
    "lidar_type": 8  # protocol type index，please don't revise this value
  },
    "MID360": {
        "lidar_net_info": {
            "cmd_data_port": 56100, # command port
            "push_msg_port": 56200, 
            "point_data_port": 56300,
            "imu_data_port": 56400,
            "log_data_port": 56500
        },
        "host_net_info": [
            {
                "lidar_ip": ["192.168.1.100"], # Lidar ip
                "host_ip": "192.168.1.5", # host ip
                "cmd_data_port": 56101,
                "push_msg_port": 56201,
                "point_data_port": 56301,
                "imu_data_port": 56401,
                "log_data_port": 56501
            }
        ]
    },
    "lidar_configs": [
        {
            "ip": "192.168.1.100", # ip of the LiDAR you want to config
            "pcl_data_type": 1,
            "pattern_mode": 0,
            "extrinsic_parameter": {
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "x": 0,
                "y": 0,
                "z": 0
            }
        }
    ]
}
```
**MID360_config2:**
```json
{
  "lidar_summary_info" : {
    "lidar_type": 8  # protocol type index，please don't revise this value
  },
    "MID360": {
        "lidar_net_info": {
            "cmd_data_port": 56100, # command port
            "push_msg_port": 56200, 
            "point_data_port": 56300,
            "imu_data_port": 56400,
            "log_data_port": 56500
        },
        "host_net_info": [
            {
                "lidar_ip": ["192.168.2.100"], # Lidar ip
                "host_ip": "192.168.2.5", # host ip
                "cmd_data_port": 56101,
                "push_msg_port": 56201,
                "point_data_port": 56301,
                "imu_data_port": 56401,
                "log_data_port": 56501
            }
        ]
    },
    "lidar_configs": [
        {
            "ip": "192.168.2.100", # ip of the LiDAR you want to config
            "pcl_data_type": 1,
            "pattern_mode": 0,
            "extrinsic_parameter": {
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "x": 0,
                "y": 0,
                "z": 0
            }
        }
    ]
}
```
Then start one driver node per config, each with its own `user_config_path` and node name
(see `launch/rviz_mixed_launch.py` for the pattern), setting `multi_topic` to 1 so each LiDAR
publishes on its own topic.

### 4.1 Extrinsic parameters

The `extrinsic_parameter` block of the LiDAR config is applied by the **driver**,
in software, when it converts raw points. The driver explicitly zeroes the
device's own install attitude at startup so the transform is not applied twice -
once on the device and once here - which is what happened previously. A
consequence is that the driver clears any install attitude a previous run, or
the vendor driver, left stored on the device.

### 4.2 Composition

The node is registered as a `rclcpp_components` component, but `LdsLidar` is a
process-wide singleton, so **only one driver node per process is supported**.
Loading two into one component container will silently share a single SDK
instance. Run separate processes for multiple LiDARs, or use `multi_topic:=1`
with one node and several entries in `lidar_configs`.

## 5. Supported LiDAR list

* HAP
* Mid360
* Mid360s
* Avia2
* (more types are comming soon...)

## 6. FAQ

### 6.1 launch with "rviz_HAP_launch.py" but no point cloud display on the grid?

Please check the "Global Options - Fixed Frame" field in the RViz "Display" pannel. Set the field value to "livox_frame" and check the "PointCloud2" option in the pannel.

### 6.2 launch with command "ros2 launch rviz_HAP_launch.py" but cannot open shared object file "liblivox_sdk_shared.so" ?

Please add '/usr/local/lib' to the env LD_LIBRARY_PATH.

* If you want to add to current terminal:

  ```shell
  export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib
  ```

* If you want to add to current user:

  ```shell
  vim ~/.bashrc
  export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib
  source ~/.bashrc
  ```
