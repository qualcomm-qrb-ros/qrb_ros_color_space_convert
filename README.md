# QRB Color Space Conversion ROS Node

## Overview
The `qrb-ros-color-space-convert` sample application converts between NV12 and RGB888 formats. 

Qualcomm's smart devices, such as the Qualcomm Dragonwing™ RB3 Gen 2 development kits, use NV12 as the default image color space conversion format. However, the more common color space format is RGB888.

The `qrb-ros-color-space-convert` sample application implements the following:

- Provides ROS nodes
  - API to convert NV12 to RGB888
  - API to convert RGB888 to NV12
- Supports `dmabuf` fd as input and output
- Input and output image receive and send with QRB ROS transport
- Hardware acceleration with GPU by OpenGL ES

## Getting Started

### Build
For the Qualcomm QCLinux platform, we provide two ways to build this package.

<details>
<summary>On-Device Compilation with Docker</summary>


1. Set up the QCLinux Docker environment following the [QRB ROS Docker Setup](https://github.com/qualcomm-qrb-ros/qrb_ros_docker?tab=readme-ov-file#quickstart).

2. Clone and build the source code:

    ```bash
    git clone https://github.com/qualcomm-qrb-ros/lib_mem_dmabuf.git
    git clone https://github.com/qualcomm-qrb-ros/qrb_ros_transport.git
    git clone https://github.com/qualcomm-qrb-ros/qrb_ros_color_space_convert.git

    colcon build --packages-skip qrb_ros_colorspace_convert
    ```

</details>


<details>
<summary>Cross Compilation with QIRP SDK</summary>

1. Set up the QIRP SDK environment: Refer to [QRB ROS Documents: Getting Started](https://qualcomm-qrb-ros.github.io/main/getting_started/environment_setup.html).

2. Create a workspace and clone the source code:
    ```bash
    mkdir -p <qirp_decompressed_workspace>/qirp-sdk/ros_ws
    cd <qirp_decompressed_workspace>/qirp-sdk/ros_ws

    git clone https://github.com/qualcomm-qrb-ros/lib_mem_dmabuf.git
    git clone https://github.com/qualcomm-qrb-ros/qrb_ros_transport.git
    git clone https://github.com/qualcomm-qrb-ros/qrb_ros_color_space_convert.git
    ```

3. Build the source code with QIRP SDK:

    ```bash
    colcon build --merge-install --cmake-args ${CMAKE_ARGS}
    ```
4. Install ROS package to device
    ```bash
    cd install
    tar -czvf sample_colorspace_convert.tar.gz lib share
    scp sample_colorspace_convert.tar.gz root@[ip-addr]:/opt/
    ssh root@[ip-addr]
    (ssh) mount -o remount rw /usr
    (ssh) tar --no-same-owner -zxf /opt/sample_colorspace_convert.tar.gz -C /usr/
    ```

</details>

<br>

### Usage

This section shows how to use `qrb-ros-color-space-convert` in your projects.
- You have **Set up the device** according to [Set up the environment for running sample applications](https://docs.qualcomm.com/bundle/publicresource/topics/80-70020-265/quick_start.html?state=preview#setup-demo-qs).
    
- The customer ROS node as the input is ready.
    
- You have connected the device to the display monitor.

Set up the runtime environment.
```

# Set the HOME variable
(ssh) export HOME=/opt

# Set up the runtime environment
(ssh) source /usr/share/qirp-setup.sh
(ssh) export ROS_DOMAIN_ID=xx # Value range of ROS_DOMAIN_ID: [0, 232]

# Set up the Weston environment
(ssh) export XDG_RUNTIME_DIR=/dev/socket/weston/
(ssh) mkdir -p $XDG_RUNTIME_DIR
(ssh) export WAYLAND_DISPLAY=wayland-1

# Set up the shared memory
(ssh) export FASTRTPS_DEFAULT_PROFILES_FILE=/usr/share/qrb_ros_colorspace_convert/config/large_message_profile.xml
```

Run the sample `qrb-ros-color-space-convert`
```
# If you want to convert color space from NV12 to RGB888, run the command below
(ssh) ros2 launch qrb_ros_colorspace_convert colorspace_convert.launch.py 'conversion_type:=nv12_to_rgb8' 'latency_fps_test:=False'

# If you want to convert color space from RGB888 to NV12, run the command below
(ssh) ros2 launch qrb_ros_colorspace_convert colorspace_convert.launch.py 'conversion_type:=rgb8_to_nv12' 'latency_fps_test:=False'
```
Use the following command to confirm the color space conversion Result.
```
(ssh) ros2 topic echo /image_raw | grep "encoding"
(ssh) ros2 topic echo /image | grep "encoding"
```

For more details, check out the documentation at [qualcomm-qrb-ros.github.io](https://qualcomm-qrb-ros.github.io/).

## Supported Types

| QRB ROS Transport Type          | ROS Interfaces          |
| ------------------------------- | ----------------------- |
| [qrb_ros::transport::type::Image](./qrb_ros_transport_image_type/include/qrb_ros_transport_image_type/image.hpp) | [sensor_msgs::msg::Image](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Image.msg) |

## Contributing

We would love to have you as a part of the QRB ROS community. Whether you are helping us fix bugs, proposing new features, improving our documentation, or spreading the word, please refer to our [contribution guidelines](https://github.qualcomm.com/jiaxshi/QRB-ROS-repository-template/blob/QRBROS/CONTRIBUTING.md) and [code of conduct](https://github.qualcomm.com/jiaxshi/QRB-ROS-repository-template/blob/QRBROS/CODE_OF_CONDUCT.md).

- Bug report: If you see an error message or encounter failures, please create a [bug report](https://github.qualcomm.com/jiaxshi/QRB-ROS-repository-template/issues)
- Feature Request: If you have an idea or if there is a capability that is missing and would make development easier and more robust, please submit a [feature request](https://github.qualcomm.com/jiaxshi/QRB-ROS-repository-template/issues)


## Authors

- **Vito Wang** - *Initial work* - [violet227 (Vito Wang)](https://github.com/violet227)

See also the list of [Contributors to qualcomm-qrb-ros/qrb_ros_color_space_convert](https://github.com/qualcomm-qrb-ros/qrb_ros_color_space_convert/graphs/contributors) who participated in this project.

## License

Project is licensed under the [BSD-3-clause License](https://spdx.org/licenses/BSD-3-Clause.html). See [LICENSE](https://github.qualcomm.com/jiaxshi/QRB-ROS-repository-template/blob/QRBROS/LICENSE) for the full license text.
