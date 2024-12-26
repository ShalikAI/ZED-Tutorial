# All-About-ZED
This repository is all about zed and it's implementation in my PhD.

# System Requirements
For this demo, we are using Ubuntu 20.04 / 22.04, ROS2 (ROS Humble), Nvidia Driver 535, Cuda 12.1 and ZED SDK 4.1.4 (pyzed-4.1) and python 3.10. 

# ZED Camera SDK Installation:
Download the Latest SDK version from the site: https://www.stereolabs.com/developers/release/.
The default download should be in your "Downloads" directory. To check, run in a terminal:
```
cd ~/Downloads
ls
```
You should see a file in the name of ZED_SDK_Ubuntu22_cuda12.1_v4.1.4.run or something similar. Change the permission of the file so that you can execute it from the terminal.
```
chmod +x ZED_SDK_Ubuntu22_cuda12.1_v4.1.4.run
```
Run the following command to install zed sdk:
```
./ZED_SDK_Ubuntu22_cuda12.1_v4.1.4.run
```
Follow the instruction & the necessary files and libraries will automatically be installed in to the system. To check installation, run the following command:
```
cd /usr/local/zed/tools
ls
```
You should see some executables. Try to run them one by one. After starting an executable, to stop it, hit ctrl+c. Now run the following command:
```

./ZED_Explorer      # To start playing, recording and loading sensor data (svo2)
ctrl+c              # To stop it
./ZED_Depth_Viewer  # To start playing and loading data (svo2) for depth and point cloud visualization
ctrl+c              # To stop it
./ZEDfu             # To start playing and loading data (svo2) for trajectory and map visualization
ctrl+c              # To stop it
```
# ZED Camera Setup
## Clone this repo:
```bash
git clone https://github.com/ArghyaChatterjee/zed-zed2-zedm-zed-x-camera-setup-in-ros2.git
cd zed-zed2-zedm-zed-x-camera-setup-in-ros2/
```

## Create a Virtualenv:
```bash
python3 -m venv zed_zed2_zedm_zed_x_camera_setup_in_ros2_venv
source venv zed_zed2_zedm_zed_x_camera_setup_in_ros2_venv/bin/activate
pip3 install --upgrade pip
pip3 install -r requirements.txt
```

## Install python api for ZED
```bash
cd scripts
python3 get_python_api.py
```

## Monitor System Performance
- To monitor system performance during the task allocated, you can open Activities Overview--->System Monitor. Alternatively, you can also install 'htop' to monitor system parameters in a unique way. In a new terminal, run:
```
sudo snap install nvtop
nvtop
```
## Monitor nvidia gpu memory usage
- To monitor your nvidia gpu memory usage during a certain gpu intensive task, install 'glances' & check by running the following command in a new terminal:
```
sudo python3 -m pip install glances[gpu]
sudo glances
```

# ROS1 topics
## ZED2
Launch the node:
```
$ roslaunch zed_wrapper zed2.launch
```
List the topics:
```
$ rostopic list
/tf
/tf_static
/zed2/joint_states
/zed2/zed_node/atm_press
/zed2/zed_node/confidence/confidence_map
/zed2/zed_node/depth/camera_info
/zed2/zed_node/depth/depth_registered
/zed2/zed_node/depth/depth_registered/compressed
/zed2/zed_node/depth/depth_registered/compressed/parameter_descriptions
/zed2/zed_node/depth/depth_registered/compressed/parameter_updates
/zed2/zed_node/depth/depth_registered/compressedDepth
/zed2/zed_node/depth/depth_registered/compressedDepth/parameter_descriptions
/zed2/zed_node/depth/depth_registered/compressedDepth/parameter_updates
/zed2/zed_node/depth/depth_registered/theora
/zed2/zed_node/depth/depth_registered/theora/parameter_descriptions
/zed2/zed_node/depth/depth_registered/theora/parameter_updates
/zed2/zed_node/disparity/disparity_image
/zed2/zed_node/imu/data
/zed2/zed_node/imu/data_raw
/zed2/zed_node/imu/mag
/zed2/zed_node/left/camera_info
/zed2/zed_node/left/image_rect_color
/zed2/zed_node/left/image_rect_color/compressed
/zed2/zed_node/left/image_rect_color/compressed/parameter_descriptions
/zed2/zed_node/left/image_rect_color/compressed/parameter_updates
/zed2/zed_node/left/image_rect_color/compressedDepth
/zed2/zed_node/left/image_rect_color/compressedDepth/parameter_descriptions
/zed2/zed_node/left/image_rect_color/compressedDepth/parameter_updates
/zed2/zed_node/left/image_rect_color/theora
/zed2/zed_node/left/image_rect_color/theora/parameter_descriptions
/zed2/zed_node/left/image_rect_color/theora/parameter_updates
/zed2/zed_node/left/image_rect_gray
/zed2/zed_node/left/image_rect_gray/compressed
/zed2/zed_node/left/image_rect_gray/compressed/parameter_descriptions
/zed2/zed_node/left/image_rect_gray/compressed/parameter_updates
/zed2/zed_node/left/image_rect_gray/compressedDepth
/zed2/zed_node/left/image_rect_gray/compressedDepth/parameter_descriptions
/zed2/zed_node/left/image_rect_gray/compressedDepth/parameter_updates
/zed2/zed_node/left/image_rect_gray/theora
/zed2/zed_node/left/image_rect_gray/theora/parameter_descriptions
/zed2/zed_node/left/image_rect_gray/theora/parameter_updates
/zed2/zed_node/left_cam_imu_transform
/zed2/zed_node/left_raw/camera_info
/zed2/zed_node/left_raw/image_raw_color
/zed2/zed_node/left_raw/image_raw_color/compressed
/zed2/zed_node/left_raw/image_raw_color/compressed/parameter_descriptions
/zed2/zed_node/left_raw/image_raw_color/compressed/parameter_updates
/zed2/zed_node/left_raw/image_raw_color/compressedDepth
/zed2/zed_node/left_raw/image_raw_color/compressedDepth/parameter_descriptions
/zed2/zed_node/left_raw/image_raw_color/compressedDepth/parameter_updates
/zed2/zed_node/left_raw/image_raw_color/theora
/zed2/zed_node/left_raw/image_raw_color/theora/parameter_descriptions
/zed2/zed_node/left_raw/image_raw_color/theora/parameter_updates
/zed2/zed_node/left_raw/image_raw_gray
/zed2/zed_node/left_raw/image_raw_gray/compressed
/zed2/zed_node/left_raw/image_raw_gray/compressed/parameter_descriptions
/zed2/zed_node/left_raw/image_raw_gray/compressed/parameter_updates
/zed2/zed_node/left_raw/image_raw_gray/compressedDepth
/zed2/zed_node/left_raw/image_raw_gray/compressedDepth/parameter_descriptions
/zed2/zed_node/left_raw/image_raw_gray/compressedDepth/parameter_updates
/zed2/zed_node/left_raw/image_raw_gray/theora
/zed2/zed_node/left_raw/image_raw_gray/theora/parameter_descriptions
/zed2/zed_node/left_raw/image_raw_gray/theora/parameter_updates
/zed2/zed_node/odom
/zed2/zed_node/odom/status
/zed2/zed_node/parameter_descriptions
/zed2/zed_node/parameter_updates
/zed2/zed_node/path_map
/zed2/zed_node/path_odom
/zed2/zed_node/plane
/zed2/zed_node/plane_marker
/zed2/zed_node/point_cloud/cloud_registered
/zed2/zed_node/pose
/zed2/zed_node/pose/status
/zed2/zed_node/pose_with_covariance
/zed2/zed_node/rgb/camera_info
/zed2/zed_node/rgb/image_rect_color
/zed2/zed_node/rgb/image_rect_color/compressed
/zed2/zed_node/rgb/image_rect_color/compressed/parameter_descriptions
/zed2/zed_node/rgb/image_rect_color/compressed/parameter_updates
/zed2/zed_node/rgb/image_rect_color/compressedDepth
/zed2/zed_node/rgb/image_rect_color/compressedDepth/parameter_descriptions
/zed2/zed_node/rgb/image_rect_color/compressedDepth/parameter_updates
/zed2/zed_node/rgb/image_rect_color/theora
/zed2/zed_node/rgb/image_rect_color/theora/parameter_descriptions
/zed2/zed_node/rgb/image_rect_color/theora/parameter_updates
/zed2/zed_node/rgb/image_rect_gray
/zed2/zed_node/rgb/image_rect_gray/compressed
/zed2/zed_node/rgb/image_rect_gray/compressed/parameter_descriptions
/zed2/zed_node/rgb/image_rect_gray/compressed/parameter_updates
/zed2/zed_node/rgb/image_rect_gray/compressedDepth
/zed2/zed_node/rgb/image_rect_gray/compressedDepth/parameter_descriptions
/zed2/zed_node/rgb/image_rect_gray/compressedDepth/parameter_updates
/zed2/zed_node/rgb/image_rect_gray/theora
/zed2/zed_node/rgb/image_rect_gray/theora/parameter_descriptions
/zed2/zed_node/rgb/image_rect_gray/theora/parameter_updates
/zed2/zed_node/rgb_raw/camera_info
/zed2/zed_node/rgb_raw/image_raw_color
/zed2/zed_node/rgb_raw/image_raw_color/compressed
/zed2/zed_node/rgb_raw/image_raw_color/compressed/parameter_descriptions
/zed2/zed_node/rgb_raw/image_raw_color/compressed/parameter_updates
/zed2/zed_node/rgb_raw/image_raw_color/compressedDepth
/zed2/zed_node/rgb_raw/image_raw_color/compressedDepth/parameter_descriptions
/zed2/zed_node/rgb_raw/image_raw_color/compressedDepth/parameter_updates
/zed2/zed_node/rgb_raw/image_raw_color/theora
/zed2/zed_node/rgb_raw/image_raw_color/theora/parameter_descriptions
/zed2/zed_node/rgb_raw/image_raw_color/theora/parameter_updates
/zed2/zed_node/rgb_raw/image_raw_gray
/zed2/zed_node/rgb_raw/image_raw_gray/compressed
/zed2/zed_node/rgb_raw/image_raw_gray/compressed/parameter_descriptions
/zed2/zed_node/rgb_raw/image_raw_gray/compressed/parameter_updates
/zed2/zed_node/rgb_raw/image_raw_gray/compressedDepth
/zed2/zed_node/rgb_raw/image_raw_gray/compressedDepth/parameter_descriptions
/zed2/zed_node/rgb_raw/image_raw_gray/compressedDepth/parameter_updates
/zed2/zed_node/rgb_raw/image_raw_gray/theora
/zed2/zed_node/rgb_raw/image_raw_gray/theora/parameter_descriptions
/zed2/zed_node/rgb_raw/image_raw_gray/theora/parameter_updates
/zed2/zed_node/right/camera_info
/zed2/zed_node/right/image_rect_color
/zed2/zed_node/right/image_rect_color/compressed
/zed2/zed_node/right/image_rect_color/compressed/parameter_descriptions
/zed2/zed_node/right/image_rect_color/compressed/parameter_updates
/zed2/zed_node/right/image_rect_color/compressedDepth
/zed2/zed_node/right/image_rect_color/compressedDepth/parameter_descriptions
/zed2/zed_node/right/image_rect_color/compressedDepth/parameter_updates
/zed2/zed_node/right/image_rect_color/mouse_click
/zed2/zed_node/right/image_rect_color/theora
/zed2/zed_node/right/image_rect_color/theora/parameter_descriptions
/zed2/zed_node/right/image_rect_color/theora/parameter_updates
/zed2/zed_node/right/image_rect_gray
/zed2/zed_node/right/image_rect_gray/compressed
/zed2/zed_node/right/image_rect_gray/compressed/parameter_descriptions
/zed2/zed_node/right/image_rect_gray/compressed/parameter_updates
/zed2/zed_node/right/image_rect_gray/compressedDepth
/zed2/zed_node/right/image_rect_gray/compressedDepth/parameter_descriptions
/zed2/zed_node/right/image_rect_gray/compressedDepth/parameter_updates
/zed2/zed_node/right/image_rect_gray/theora
/zed2/zed_node/right/image_rect_gray/theora/parameter_descriptions
/zed2/zed_node/right/image_rect_gray/theora/parameter_updates
/zed2/zed_node/right_raw/camera_info
/zed2/zed_node/right_raw/image_raw_color
/zed2/zed_node/right_raw/image_raw_color/compressed
/zed2/zed_node/right_raw/image_raw_color/compressed/parameter_descriptions
/zed2/zed_node/right_raw/image_raw_color/compressed/parameter_updates
/zed2/zed_node/right_raw/image_raw_color/compressedDepth
/zed2/zed_node/right_raw/image_raw_color/compressedDepth/parameter_descriptions
/zed2/zed_node/right_raw/image_raw_color/compressedDepth/parameter_updates
/zed2/zed_node/right_raw/image_raw_color/theora
/zed2/zed_node/right_raw/image_raw_color/theora/parameter_descriptions
/zed2/zed_node/right_raw/image_raw_color/theora/parameter_updates
/zed2/zed_node/right_raw/image_raw_gray
/zed2/zed_node/right_raw/image_raw_gray/compressed
/zed2/zed_node/right_raw/image_raw_gray/compressed/parameter_descriptions
/zed2/zed_node/right_raw/image_raw_gray/compressed/parameter_updates
/zed2/zed_node/right_raw/image_raw_gray/compressedDepth
/zed2/zed_node/right_raw/image_raw_gray/compressedDepth/parameter_descriptions
/zed2/zed_node/right_raw/image_raw_gray/compressedDepth/parameter_updates
/zed2/zed_node/right_raw/image_raw_gray/theora
/zed2/zed_node/right_raw/image_raw_gray/theora/parameter_descriptions
/zed2/zed_node/right_raw/image_raw_gray/theora/parameter_updates
/zed2/zed_node/stereo/image_rect_color
/zed2/zed_node/stereo/image_rect_color/compressed
/zed2/zed_node/stereo/image_rect_color/compressed/parameter_descriptions
/zed2/zed_node/stereo/image_rect_color/compressed/parameter_updates
/zed2/zed_node/stereo/image_rect_color/compressedDepth
/zed2/zed_node/stereo/image_rect_color/compressedDepth/parameter_descriptions
/zed2/zed_node/stereo/image_rect_color/compressedDepth/parameter_updates
/zed2/zed_node/stereo/image_rect_color/theora
/zed2/zed_node/stereo/image_rect_color/theora/parameter_descriptions
/zed2/zed_node/stereo/image_rect_color/theora/parameter_updates
/zed2/zed_node/stereo_raw/image_raw_color
/zed2/zed_node/stereo_raw/image_raw_color/compressed
/zed2/zed_node/stereo_raw/image_raw_color/compressed/parameter_descriptions
/zed2/zed_node/stereo_raw/image_raw_color/compressed/parameter_updates
/zed2/zed_node/stereo_raw/image_raw_color/compressedDepth
/zed2/zed_node/stereo_raw/image_raw_color/compressedDepth/parameter_descriptions
/zed2/zed_node/stereo_raw/image_raw_color/compressedDepth/parameter_updates
/zed2/zed_node/stereo_raw/image_raw_color/theora
/zed2/zed_node/stereo_raw/image_raw_color/theora/parameter_descriptions
/zed2/zed_node/stereo_raw/image_raw_color/theora/parameter_updates
/zed2/zed_node/temperature/imu
/zed2/zed_node/temperature/left
/zed2/zed_node/temperature/right
```
## ZEDMini
Launch the node:
```
$ roslaunch zed_wrapper zedm.launch
```
List the topics:
```
$ rostopic list
/tf
/tf_static
/zed2/zed_node/right/image_rect_color
/zed2/zed_node/right/image_rect_color/mouse_click
/zedm/joint_states
/zedm/zed_node/confidence/confidence_map
/zedm/zed_node/depth/camera_info
/zedm/zed_node/depth/depth_registered
/zedm/zed_node/depth/depth_registered/compressed
/zedm/zed_node/depth/depth_registered/compressed/parameter_descriptions
/zedm/zed_node/depth/depth_registered/compressed/parameter_updates
/zedm/zed_node/depth/depth_registered/compressedDepth
/zedm/zed_node/depth/depth_registered/compressedDepth/parameter_descriptions
/zedm/zed_node/depth/depth_registered/compressedDepth/parameter_updates
/zedm/zed_node/depth/depth_registered/theora
/zedm/zed_node/depth/depth_registered/theora/parameter_descriptions
/zedm/zed_node/depth/depth_registered/theora/parameter_updates
/zedm/zed_node/disparity/disparity_image
/zedm/zed_node/imu/data
/zedm/zed_node/imu/data_raw
/zedm/zed_node/left/camera_info
/zedm/zed_node/left/image_rect_color
/zedm/zed_node/left/image_rect_color/compressed
/zedm/zed_node/left/image_rect_color/compressed/parameter_descriptions
/zedm/zed_node/left/image_rect_color/compressed/parameter_updates
/zedm/zed_node/left/image_rect_color/compressedDepth
/zedm/zed_node/left/image_rect_color/compressedDepth/parameter_descriptions
/zedm/zed_node/left/image_rect_color/compressedDepth/parameter_updates
/zedm/zed_node/left/image_rect_color/theora
/zedm/zed_node/left/image_rect_color/theora/parameter_descriptions
/zedm/zed_node/left/image_rect_color/theora/parameter_updates
/zedm/zed_node/left/image_rect_gray
/zedm/zed_node/left/image_rect_gray/compressed
/zedm/zed_node/left/image_rect_gray/compressed/parameter_descriptions
/zedm/zed_node/left/image_rect_gray/compressed/parameter_updates
/zedm/zed_node/left/image_rect_gray/compressedDepth
/zedm/zed_node/left/image_rect_gray/compressedDepth/parameter_descriptions
/zedm/zed_node/left/image_rect_gray/compressedDepth/parameter_updates
/zedm/zed_node/left/image_rect_gray/theora
/zedm/zed_node/left/image_rect_gray/theora/parameter_descriptions
/zedm/zed_node/left/image_rect_gray/theora/parameter_updates
/zedm/zed_node/left_cam_imu_transform
/zedm/zed_node/left_raw/camera_info
/zedm/zed_node/left_raw/image_raw_color
/zedm/zed_node/left_raw/image_raw_color/compressed
/zedm/zed_node/left_raw/image_raw_color/compressed/parameter_descriptions
/zedm/zed_node/left_raw/image_raw_color/compressed/parameter_updates
/zedm/zed_node/left_raw/image_raw_color/compressedDepth
/zedm/zed_node/left_raw/image_raw_color/compressedDepth/parameter_descriptions
/zedm/zed_node/left_raw/image_raw_color/compressedDepth/parameter_updates
/zedm/zed_node/left_raw/image_raw_color/theora
/zedm/zed_node/left_raw/image_raw_color/theora/parameter_descriptions
/zedm/zed_node/left_raw/image_raw_color/theora/parameter_updates
/zedm/zed_node/left_raw/image_raw_gray
/zedm/zed_node/left_raw/image_raw_gray/compressed
/zedm/zed_node/left_raw/image_raw_gray/compressed/parameter_descriptions
/zedm/zed_node/left_raw/image_raw_gray/compressed/parameter_updates
/zedm/zed_node/left_raw/image_raw_gray/compressedDepth
/zedm/zed_node/left_raw/image_raw_gray/compressedDepth/parameter_descriptions
/zedm/zed_node/left_raw/image_raw_gray/compressedDepth/parameter_updates
/zedm/zed_node/left_raw/image_raw_gray/theora
/zedm/zed_node/left_raw/image_raw_gray/theora/parameter_descriptions
/zedm/zed_node/left_raw/image_raw_gray/theora/parameter_updates
/zedm/zed_node/odom
/zedm/zed_node/odom/status
/zedm/zed_node/parameter_descriptions
/zedm/zed_node/parameter_updates
/zedm/zed_node/path_map
/zedm/zed_node/path_odom
/zedm/zed_node/plane
/zedm/zed_node/plane_marker
/zedm/zed_node/point_cloud/cloud_registered
/zedm/zed_node/pose
/zedm/zed_node/pose/status
/zedm/zed_node/pose_with_covariance
/zedm/zed_node/rgb/camera_info
/zedm/zed_node/rgb/image_rect_color
/zedm/zed_node/rgb/image_rect_color/compressed
/zedm/zed_node/rgb/image_rect_color/compressed/parameter_descriptions
/zedm/zed_node/rgb/image_rect_color/compressed/parameter_updates
/zedm/zed_node/rgb/image_rect_color/compressedDepth
/zedm/zed_node/rgb/image_rect_color/compressedDepth/parameter_descriptions
/zedm/zed_node/rgb/image_rect_color/compressedDepth/parameter_updates
/zedm/zed_node/rgb/image_rect_color/theora
/zedm/zed_node/rgb/image_rect_color/theora/parameter_descriptions
/zedm/zed_node/rgb/image_rect_color/theora/parameter_updates
/zedm/zed_node/rgb/image_rect_gray
/zedm/zed_node/rgb/image_rect_gray/compressed
/zedm/zed_node/rgb/image_rect_gray/compressed/parameter_descriptions
/zedm/zed_node/rgb/image_rect_gray/compressed/parameter_updates
/zedm/zed_node/rgb/image_rect_gray/compressedDepth
/zedm/zed_node/rgb/image_rect_gray/compressedDepth/parameter_descriptions
/zedm/zed_node/rgb/image_rect_gray/compressedDepth/parameter_updates
/zedm/zed_node/rgb/image_rect_gray/theora
/zedm/zed_node/rgb/image_rect_gray/theora/parameter_descriptions
/zedm/zed_node/rgb/image_rect_gray/theora/parameter_updates
/zedm/zed_node/rgb_raw/camera_info
/zedm/zed_node/rgb_raw/image_raw_color
/zedm/zed_node/rgb_raw/image_raw_color/compressed
/zedm/zed_node/rgb_raw/image_raw_color/compressed/parameter_descriptions
/zedm/zed_node/rgb_raw/image_raw_color/compressed/parameter_updates
/zedm/zed_node/rgb_raw/image_raw_color/compressedDepth
/zedm/zed_node/rgb_raw/image_raw_color/compressedDepth/parameter_descriptions
/zedm/zed_node/rgb_raw/image_raw_color/compressedDepth/parameter_updates
/zedm/zed_node/rgb_raw/image_raw_color/theora
/zedm/zed_node/rgb_raw/image_raw_color/theora/parameter_descriptions
/zedm/zed_node/rgb_raw/image_raw_color/theora/parameter_updates
/zedm/zed_node/rgb_raw/image_raw_gray
/zedm/zed_node/rgb_raw/image_raw_gray/compressed
/zedm/zed_node/rgb_raw/image_raw_gray/compressed/parameter_descriptions
/zedm/zed_node/rgb_raw/image_raw_gray/compressed/parameter_updates
/zedm/zed_node/rgb_raw/image_raw_gray/compressedDepth
/zedm/zed_node/rgb_raw/image_raw_gray/compressedDepth/parameter_descriptions
/zedm/zed_node/rgb_raw/image_raw_gray/compressedDepth/parameter_updates
/zedm/zed_node/rgb_raw/image_raw_gray/theora
/zedm/zed_node/rgb_raw/image_raw_gray/theora/parameter_descriptions
/zedm/zed_node/rgb_raw/image_raw_gray/theora/parameter_updates
/zedm/zed_node/right/camera_info
/zedm/zed_node/right/image_rect_color
/zedm/zed_node/right/image_rect_color/compressed
/zedm/zed_node/right/image_rect_color/compressed/parameter_descriptions
/zedm/zed_node/right/image_rect_color/compressed/parameter_updates
/zedm/zed_node/right/image_rect_color/compressedDepth
/zedm/zed_node/right/image_rect_color/compressedDepth/parameter_descriptions
/zedm/zed_node/right/image_rect_color/compressedDepth/parameter_updates
/zedm/zed_node/right/image_rect_color/theora
/zedm/zed_node/right/image_rect_color/theora/parameter_descriptions
/zedm/zed_node/right/image_rect_color/theora/parameter_updates
/zedm/zed_node/right/image_rect_gray
/zedm/zed_node/right/image_rect_gray/compressed
/zedm/zed_node/right/image_rect_gray/compressed/parameter_descriptions
/zedm/zed_node/right/image_rect_gray/compressed/parameter_updates
/zedm/zed_node/right/image_rect_gray/compressedDepth
/zedm/zed_node/right/image_rect_gray/compressedDepth/parameter_descriptions
/zedm/zed_node/right/image_rect_gray/compressedDepth/parameter_updates
/zedm/zed_node/right/image_rect_gray/theora
/zedm/zed_node/right/image_rect_gray/theora/parameter_descriptions
/zedm/zed_node/right/image_rect_gray/theora/parameter_updates
/zedm/zed_node/right_raw/camera_info
/zedm/zed_node/right_raw/image_raw_color
/zedm/zed_node/right_raw/image_raw_color/compressed
/zedm/zed_node/right_raw/image_raw_color/compressed/parameter_descriptions
/zedm/zed_node/right_raw/image_raw_color/compressed/parameter_updates
/zedm/zed_node/right_raw/image_raw_color/compressedDepth
/zedm/zed_node/right_raw/image_raw_color/compressedDepth/parameter_descriptions
/zedm/zed_node/right_raw/image_raw_color/compressedDepth/parameter_updates
/zedm/zed_node/right_raw/image_raw_color/theora
/zedm/zed_node/right_raw/image_raw_color/theora/parameter_descriptions
/zedm/zed_node/right_raw/image_raw_color/theora/parameter_updates
/zedm/zed_node/right_raw/image_raw_gray
/zedm/zed_node/right_raw/image_raw_gray/compressed
/zedm/zed_node/right_raw/image_raw_gray/compressed/parameter_descriptions
/zedm/zed_node/right_raw/image_raw_gray/compressed/parameter_updates
/zedm/zed_node/right_raw/image_raw_gray/compressedDepth
/zedm/zed_node/right_raw/image_raw_gray/compressedDepth/parameter_descriptions
/zedm/zed_node/right_raw/image_raw_gray/compressedDepth/parameter_updates
/zedm/zed_node/right_raw/image_raw_gray/theora
/zedm/zed_node/right_raw/image_raw_gray/theora/parameter_descriptions
/zedm/zed_node/right_raw/image_raw_gray/theora/parameter_updates
/zedm/zed_node/stereo/image_rect_color
/zedm/zed_node/stereo/image_rect_color/compressed
/zedm/zed_node/stereo/image_rect_color/compressed/parameter_descriptions
/zedm/zed_node/stereo/image_rect_color/compressed/parameter_updates
/zedm/zed_node/stereo/image_rect_color/compressedDepth
/zedm/zed_node/stereo/image_rect_color/compressedDepth/parameter_descriptions
/zedm/zed_node/stereo/image_rect_color/compressedDepth/parameter_updates
/zedm/zed_node/stereo/image_rect_color/theora
/zedm/zed_node/stereo/image_rect_color/theora/parameter_descriptions
/zedm/zed_node/stereo/image_rect_color/theora/parameter_updates
/zedm/zed_node/stereo_raw/image_raw_color
/zedm/zed_node/stereo_raw/image_raw_color/compressed
/zedm/zed_node/stereo_raw/image_raw_color/compressed/parameter_descriptions
/zedm/zed_node/stereo_raw/image_raw_color/compressed/parameter_updates
/zedm/zed_node/stereo_raw/image_raw_color/compressedDepth
/zedm/zed_node/stereo_raw/image_raw_color/compressedDepth/parameter_descriptions
/zedm/zed_node/stereo_raw/image_raw_color/compressedDepth/parameter_updates
/zedm/zed_node/stereo_raw/image_raw_color/theora
/zedm/zed_node/stereo_raw/image_raw_color/theora/parameter_descriptions
/zedm/zed_node/stereo_raw/image_raw_color/theora/parameter_updates
```

# Setup ZED ROS1 workspace

This package lets you use the ZED stereo camera with ROS. It outputs the camera's left and right images, depth map, point cloud, and pose information and supports the use of multiple ZED cameras.

[More information](https://www.stereolabs.com/documentation/guides/using-zed-with-ros/introduction.html)

**Note:** The `zed_interfaces` package has been removed from this repository and moved to its own [`zed-ros-interfaces` repository](https://github.com/stereolabs/zed-ros-interfaces) to allow better integration of the ZED Wrapper on remote ground stations that do not require the full package to be installed. To update your repository please follow the [new update instructions](https://github.com/stereolabs/zed-ros-wrapper#update-the-repository). For more information please read issue [#750](https://github.com/stereolabs/zed-ros-wrapper/issues/750).

## Getting started

- First, download the latest version of the ZED SDK on [stereolabs.com](https://www.stereolabs.com/developers/)
- [Install](#build-the-program) the ZED ROS wrapper
- For more information, check out our [ROS documentation](https://www.stereolabs.com/documentation/guides/using-zed-with-ros/introduction.html). If you want to customize the wrapper, check the [ZED API documentation](https://www.stereolabs.com/developers/documentation/API/)

### Prerequisites

- Ubuntu 20.04
- [ZED SDK **v4.1**](https://www.stereolabs.com/developers/) and its dependency [CUDA](https://developer.nvidia.com/cuda-downloads)
- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)

### Build the repository

The zed_ros_wrapper is a catkin package. It depends on the following ROS packages:

- roscpp
- image_transport
- rosconsole
- sensor_msgs
- stereo_msgs
- std_msgs
- std_srvs
- message_filters
- tf2_ros
- nodelet
- tf2_geometry_msgs
- message_generation
- diagnostic_updater    
- dynamic_reconfigure
- zed_interfaces

Open a terminal, clone the repository, update the dependencies and build the packages:

    $ cd ~/zed_ros1_ws/src
    $ git clone --recursive https://github.com/stereolabs/zed-ros-wrapper.git
    $ git clone https://github.com/stereolabs/zed-ros-interfaces.git
    $ cd ../
    $ rosdep install --from-paths src --ignore-src -r -y
    $ catkin_make -DCMAKE_BUILD_TYPE=Release
    $ source ./devel/setup.bash

### Run the ZED wrapper

To launch the ZED node use

ZED camera:

    $ roslaunch zed_wrapper zed.launch
   
ZED Mini camera:

    $ roslaunch zed_wrapper zedm.launch
   
ZED 2 camera:

    $ roslaunch zed_wrapper zed2.launch

ZED 2i camera:

    $ roslaunch zed_wrapper zed2i.launch

ZED X camera:

    $ roslaunch zed_wrapper zedx.launch  

ZED X Mini camera:

    $ roslaunch zed_wrapper zedxm.launch  

To select the camera from its serial number:
 
     $ roslaunch zed_wrapper zed.launch serial_number:=1010 #replace 1010 with the actual SN

### Rviz visualization
Example launch files to start a pre-configured Rviz environment to visualize the data of ZED, ZED Mini, ZED 2, ZED X, and ZED X Mini cameras are provided in the [`zed-ros-examples` repository](https://github.com/stereolabs/zed-ros-examples/tree/master/zed_display_rviz)
    
### SVO recording
[SVO recording](https://www.stereolabs.com/docs/video/#video-recording) can be started and stopped while the ZED node is running using the service `start_svo_recording` and the service `stop_svo_recording`.
[More information](https://www.stereolabs.com/docs/ros/zed_node/#services)

### Object Detection
The SDK v3.0 introduces the Object Detection and Tracking module. **The Object Detection module is available only with a ZED 2 camera**. 

The Object Detection can be enabled *automatically* when the node start setting the parameter `object_detection/od_enabled` to `true` in the file `common.yaml`.

The Object Detection can be enabled/disabled *manually* calling the service `enable_object_detection`.

### Body Tracking
The Body Tracking module is not available for the ZED ROS Wrapper. Please consider migrating to the [ZED ROS2 Wrapper](https://github.com/stereolabs/zed-ros2-wrapper) if you need it.

### Spatial Mapping
The Spatial Mapping can be enabled automatically when the node start setting the parameter `mapping/mapping_enabled` to `true` in the file `common.yaml`.
The Spatial Mapping can be enabled/disabled manually calling the services `start_3d_mapping` and `stop_3d_mapping`.

### Geo Tracking (GNSS Fusion)
The Geo tracking module is not available for the ZED ROS Wrapper. Please consider migrating to the [ZED ROS2 Wrapper](https://github.com/stereolabs/zed-ros2-wrapper) if you need it.

### Diagnostic
The ZED node publishes diagnostic information that can be used by the robotic system using a [diagnostic_aggregator node](http://wiki.ros.org/diagnostic_aggregator).

With the `rqt` plugin `Runtime monitor`, it is possible to retrieve all the diagnostic information, checking that the node 
is working as expected.

### 2D mode
For robots moving on a planar surface it is possible to activate the "2D mode" (parameter `tracking/two_d_mode` in `common.yaml`). 
The value of the coordinate Z for odometry and pose will have a fixed value (parameter `tracking/fixed_z_value` in `common.yaml`). 
Roll and pitch and relative velocities will be fixed to zero.

## Examples and Tutorials
Examples and tutorials are provided to better understand how to use the ZED wrapper and how to integrate it in the ROS framework.
See the [`zed-ros-examples` repository](https://github.com/stereolabs/zed-ros-examples)

### Examples
Alongside the wrapper itself and the Rviz display, a few examples are provided to interface the ZED with other ROS packages :

- [RTAB-Map](http://introlab.github.io/rtabmap/): See [zed_rtabmap_example](https://github.com/stereolabs/zed-ros-examples/tree/master/examples/zed_rtabmap_example/README.md)
- ROS Nodelet, `depthimage_to_laserscan`: See [zed_nodelet_example](https://github.com/stereolabs/zed-ros-examples/tree/master/examples/zed_nodelet_example/README.md)
- AR Track Alvar: See [zed_ar_track_alvar_example](https://github.com/stereolabs/zed-ros-examples/tree/master/examples/zed_ar_track_alvar_example/README.md)

### Tutorials

A few tutorials are provided to understand how to use the ZED node in the ROS environment :

 - [Image subscription tutorial](https://github.com/stereolabs/zed-ros-examples/tree/master/tutorials/zed_video_sub_tutorial/README.md)
 - [Depth subscription tutorial](https://github.com/stereolabs/zed-ros-examples/tree/master/tutorials/zed_depth_sub_tutorial/README.md)
 - [Tracking subscription tutorial](https://github.com/stereolabs/zed-ros-examples/tree/master/tutorials/zed_tracking_sub_tutorial/README.md) 
 - [Sensors data subscription tutorial](https://github.com/stereolabs/zed-ros-examples/blob/master/tutorials/zed_sensors_sub_tutorial/README.md) 
 - [Object detection subscription tutorial](https://github.com/stereolabs/zed-ros-examples/blob/master/tutorials/zed_obj_det_sub_tutorial/README.md) 

## Stereolabs ZED Camera - ROS Interfaces

The `zed-ros-interfaces` repository install the `zed_interfaces` ROS package which defines the custom topics, services and actions used by the [ZED ROS Wrapper](https://github.com/stereolabs/zed-ros-wrapper) to interface with ROS.

If you already installed the [ZED ROS Wrapper](https://github.com/stereolabs/zed-ros-wrapper) or you plan to install it on this machine, this package is not required because it is automatically integrated by `zed-ros-wrapper` as a git submodule to satisfy all the required dependencies.

You must instead install this package on a remote system that must retrieve the topics sent by the ZED Wrapper (e.g. the list of detected objects obtained with the Object Detection module) or call services and actions to control the status of the ZED Wrapper.

**Note:** this package does not require CUDA, hence it can be used to receive the ZED data also on machines not equipped with an Nvidia GPU.

### Prerequisites

- Ubuntu 20.04
- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)

### Build the repository

The `zed_interfaces` is a catkin package. It depends on the following ROS packages:

- catkin
- std_msgs
- sensor_msgs
- actionlib_msgs
- geometry_msgs
- message_generation

## Custom Topics

 - BoundingBox2Df
 - BoundingBox2Di
 - BoundingBox3D
 - Keypoint2Df
 - Keypoint2Di
 - Keypoint3D
 - Object
 - ObjectsStamped
 - RGBDSensors
 - Skeleton2D
 - Skeleton3D
 - PlaneStamped

You can get more information reading the [Stereolabs online documentation](https://www.stereolabs.com/docs/ros/zed-node/)

## Custom Services

 - reset_odometry
 - reset_tracking
 - set_led_status
 - set_pose
 - save_3d_map
 - save_area_memory
 - start_3d_mapping
 - start_object_detection
 - start_remote_stream
 - start_svo_recording
 - stop_3d_mapping
 - stop_object_detection
 - stop_remote_stream
 - stop_svo_recording
 - toggle_led

You can get more information reading the [Stereolabs online documentation](https://www.stereolabs.com/docs/ros/zed-node/#services)


# ROS2 topics 
## ZED2
Launch the node:
```
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2
```
List the topics:
```
$ ros2 topic list
/tf
/tf_static
/zed/joint_states
/zed/plane
/zed/plane_marker
/zed/robot_description
/zed/zed_node/atm_press
/zed/zed_node/confidence/confidence_map
/zed/zed_node/depth/camera_info
/zed/zed_node/depth/depth_info
/zed/zed_node/depth/depth_registered
/zed/zed_node/depth/depth_registered/compressed
/zed/zed_node/depth/depth_registered/compressedDepth
/zed/zed_node/depth/depth_registered/theora
/zed/zed_node/disparity/disparity_image
/zed/zed_node/imu/data
/zed/zed_node/imu/data_raw
/zed/zed_node/imu/mag
/zed/zed_node/left/camera_info
/zed/zed_node/left/image_rect_color
/zed/zed_node/left/image_rect_color/compressed
/zed/zed_node/left/image_rect_color/compressedDepth
/zed/zed_node/left/image_rect_color/theora
/zed/zed_node/left_cam_imu_transform
/zed/zed_node/left_gray/camera_info
/zed/zed_node/left_gray/image_rect_gray
/zed/zed_node/left_gray/image_rect_gray/compressed
/zed/zed_node/left_gray/image_rect_gray/compressedDepth
/zed/zed_node/left_gray/image_rect_gray/theora
/zed/zed_node/left_raw/camera_info
/zed/zed_node/left_raw/image_raw_color
/zed/zed_node/left_raw/image_raw_color/compressed
/zed/zed_node/left_raw/image_raw_color/compressedDepth
/zed/zed_node/left_raw/image_raw_color/theora
/zed/zed_node/left_raw_gray/camera_info
/zed/zed_node/left_raw_gray/image_raw_gray
/zed/zed_node/left_raw_gray/image_raw_gray/compressed
/zed/zed_node/left_raw_gray/image_raw_gray/compressedDepth
/zed/zed_node/left_raw_gray/image_raw_gray/theora
/zed/zed_node/odom
/zed/zed_node/path_map
/zed/zed_node/path_odom
/zed/zed_node/point_cloud/cloud_registered
/zed/zed_node/pose
/zed/zed_node/pose/status
/zed/zed_node/pose_with_covariance
/zed/zed_node/rgb/camera_info
/zed/zed_node/rgb/image_rect_color
/zed/zed_node/rgb/image_rect_color/compressed
/zed/zed_node/rgb/image_rect_color/compressedDepth
/zed/zed_node/rgb/image_rect_color/theora
/zed/zed_node/rgb_gray/camera_info
/zed/zed_node/rgb_gray/image_rect_gray
/zed/zed_node/rgb_gray/image_rect_gray/compressed
/zed/zed_node/rgb_gray/image_rect_gray/compressedDepth
/zed/zed_node/rgb_gray/image_rect_gray/theora
/zed/zed_node/rgb_raw/camera_info
/zed/zed_node/rgb_raw/image_raw_color
/zed/zed_node/rgb_raw/image_raw_color/compressed
/zed/zed_node/rgb_raw/image_raw_color/compressedDepth
/zed/zed_node/rgb_raw/image_raw_color/theora
/zed/zed_node/rgb_raw_gray/camera_info
/zed/zed_node/rgb_raw_gray/image_raw_gray
/zed/zed_node/rgb_raw_gray/image_raw_gray/compressed
/zed/zed_node/rgb_raw_gray/image_raw_gray/compressedDepth
/zed/zed_node/rgb_raw_gray/image_raw_gray/theora
/zed/zed_node/right/camera_info
/zed/zed_node/right/image_rect_color
/zed/zed_node/right/image_rect_color/compressed
/zed/zed_node/right/image_rect_color/compressedDepth
/zed/zed_node/right/image_rect_color/theora
/zed/zed_node/right_gray/camera_info
/zed/zed_node/right_gray/image_rect_gray
/zed/zed_node/right_gray/image_rect_gray/compressed
/zed/zed_node/right_gray/image_rect_gray/compressedDepth
/zed/zed_node/right_gray/image_rect_gray/theora
/zed/zed_node/right_raw/camera_info
/zed/zed_node/right_raw/image_raw_color
/zed/zed_node/right_raw/image_raw_color/compressed
/zed/zed_node/right_raw/image_raw_color/compressedDepth
/zed/zed_node/right_raw/image_raw_color/theora
/zed/zed_node/right_raw_gray/camera_info
/zed/zed_node/right_raw_gray/image_raw_gray
/zed/zed_node/right_raw_gray/image_raw_gray/compressed
/zed/zed_node/right_raw_gray/image_raw_gray/compressedDepth
/zed/zed_node/right_raw_gray/image_raw_gray/theora
/zed/zed_node/stereo/image_rect_color
/zed/zed_node/stereo/image_rect_color/compressed
/zed/zed_node/stereo/image_rect_color/compressedDepth
/zed/zed_node/stereo/image_rect_color/theora
/zed/zed_node/stereo_raw/image_raw_color
/zed/zed_node/stereo_raw/image_raw_color/compressed
/zed/zed_node/stereo_raw/image_raw_color/compressedDepth
/zed/zed_node/stereo_raw/image_raw_color/theora
/zed/zed_node/temperature/imu
/zed/zed_node/temperature/left
/zed/zed_node/temperature/right
```
# ZEDmini
Launch the node:
```
$ ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm
```
List the topics:
```
$ ros2 topic list
/tf
/tf_static
/zed/joint_states
/zed/plane
/zed/plane_marker
/zed/robot_description
/zed/zed_node/confidence/confidence_map
/zed/zed_node/depth/camera_info
/zed/zed_node/depth/depth_info
/zed/zed_node/depth/depth_registered
/zed/zed_node/depth/depth_registered/compressed
/zed/zed_node/depth/depth_registered/compressedDepth
/zed/zed_node/depth/depth_registered/theora
/zed/zed_node/disparity/disparity_image
/zed/zed_node/imu/data
/zed/zed_node/imu/data_raw
/zed/zed_node/left/camera_info
/zed/zed_node/left/image_rect_color
/zed/zed_node/left/image_rect_color/compressed
/zed/zed_node/left/image_rect_color/compressedDepth
/zed/zed_node/left/image_rect_color/theora
/zed/zed_node/left_cam_imu_transform
/zed/zed_node/left_gray/camera_info
/zed/zed_node/left_gray/image_rect_gray
/zed/zed_node/left_gray/image_rect_gray/compressed
/zed/zed_node/left_gray/image_rect_gray/compressedDepth
/zed/zed_node/left_gray/image_rect_gray/theora
/zed/zed_node/left_raw/camera_info
/zed/zed_node/left_raw/image_raw_color
/zed/zed_node/left_raw/image_raw_color/compressed
/zed/zed_node/left_raw/image_raw_color/compressedDepth
/zed/zed_node/left_raw/image_raw_color/theora
/zed/zed_node/left_raw_gray/camera_info
/zed/zed_node/left_raw_gray/image_raw_gray
/zed/zed_node/left_raw_gray/image_raw_gray/compressed
/zed/zed_node/left_raw_gray/image_raw_gray/compressedDepth
/zed/zed_node/left_raw_gray/image_raw_gray/theora
/zed/zed_node/odom
/zed/zed_node/path_map
/zed/zed_node/path_odom
/zed/zed_node/point_cloud/cloud_registered
/zed/zed_node/pose
/zed/zed_node/pose/status
/zed/zed_node/pose_with_covariance
/zed/zed_node/rgb/camera_info
/zed/zed_node/rgb/image_rect_color
/zed/zed_node/rgb/image_rect_color/compressed
/zed/zed_node/rgb/image_rect_color/compressedDepth
/zed/zed_node/rgb/image_rect_color/theora
/zed/zed_node/rgb_gray/camera_info
/zed/zed_node/rgb_gray/image_rect_gray
/zed/zed_node/rgb_gray/image_rect_gray/compressed
/zed/zed_node/rgb_gray/image_rect_gray/compressedDepth
/zed/zed_node/rgb_gray/image_rect_gray/theora
/zed/zed_node/rgb_raw/camera_info
/zed/zed_node/rgb_raw/image_raw_color
/zed/zed_node/rgb_raw/image_raw_color/compressed
/zed/zed_node/rgb_raw/image_raw_color/compressedDepth
/zed/zed_node/rgb_raw/image_raw_color/theora
/zed/zed_node/rgb_raw_gray/camera_info
/zed/zed_node/rgb_raw_gray/image_raw_gray
/zed/zed_node/rgb_raw_gray/image_raw_gray/compressed
/zed/zed_node/rgb_raw_gray/image_raw_gray/compressedDepth
/zed/zed_node/rgb_raw_gray/image_raw_gray/theora
/zed/zed_node/right/camera_info
/zed/zed_node/right/image_rect_color
/zed/zed_node/right/image_rect_color/compressed
/zed/zed_node/right/image_rect_color/compressedDepth
/zed/zed_node/right/image_rect_color/theora
/zed/zed_node/right_gray/camera_info
/zed/zed_node/right_gray/image_rect_gray
/zed/zed_node/right_gray/image_rect_gray/compressed
/zed/zed_node/right_gray/image_rect_gray/compressedDepth
/zed/zed_node/right_gray/image_rect_gray/theora
/zed/zed_node/right_raw/camera_info
/zed/zed_node/right_raw/image_raw_color
/zed/zed_node/right_raw/image_raw_color/compressed
/zed/zed_node/right_raw/image_raw_color/compressedDepth
/zed/zed_node/right_raw/image_raw_color/theora
/zed/zed_node/right_raw_gray/camera_info
/zed/zed_node/right_raw_gray/image_raw_gray
/zed/zed_node/right_raw_gray/image_raw_gray/compressed
/zed/zed_node/right_raw_gray/image_raw_gray/compressedDepth
/zed/zed_node/right_raw_gray/image_raw_gray/theora
/zed/zed_node/stereo/image_rect_color
/zed/zed_node/stereo/image_rect_color/compressed
/zed/zed_node/stereo/image_rect_color/compressedDepth
/zed/zed_node/stereo/image_rect_color/theora
/zed/zed_node/stereo_raw/image_raw_color
/zed/zed_node/stereo_raw/image_raw_color/compressed
/zed/zed_node/stereo_raw/image_raw_color/compressedDepth
/zed/zed_node/stereo_raw/image_raw_color/theora
```
Additional topics coming from zed2 (different from zedm):
```
/zed/zed_node/atm_press
/zed/zed_node/imu/mag
/zed/zed_node/temperature/imu
/zed/zed_node/temperature/left
/zed/zed_node/temperature/right
```

# Setup ZED ROS2 Workspace

### Prerequisites
  - [Foxy on Ubuntu 20.04](https://docs.ros.org/en/foxy/Installation/Linux-Install-Debians.html) -> Close to EOL
  - [Humble on Ubuntu 22.04](https://docs.ros.org/en/humble/Installation/Linux-Install-Debians.html)

### Build the package

The **zed_ros2_wrapper** is a [colcon](http://design.ros2.org/articles/build_tool.html) package. Set up your colcon workspace yet, please follow this short [tutorial](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/).

#### Instruction for ZED SDK >=4.2
To install the **zed_ros2_wrapper**, clone the package, and build it:

```bash
source /opt/ros/humble/setup.bash # for ubuntu 22.04 with ros humble installed
# source /opt/ros/foxy/setup.bash # for ubuntu 20.04 with ros foxy installed
mkdir -p ~/zed_ros2_ws/src/ 
cd ~/zed_ros2_ws/src/ 
git clone  --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
cd ..
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y # install dependencies
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc) 
echo source $(pwd)/install/local_setup.bash >> ~/.bashrc
source ~/.bashrc
```

#### Instruction for ZED SDK <=4.1.4
To install the **zed_ros2_wrapper**, clone the package, and build it:

```bash
source /opt/ros/humble/setup.bash # for ubuntu 22.04 with ros humble installed
# source /opt/ros/foxy/setup.bash # for ubuntu 20.04 with ros foxy installed
mkdir -p ~/zed_ros2_ws/src/ 
cd ~/zed_ros2_ws/src/ 
git clone  --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
cd zed-ros2-wrapper
git checkout tags/humble-v4.1.4
git submodule update --init --recursive
cd ../..
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y # install dependencies
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc) 
echo source $(pwd)/install/local_setup.bash >> ~/.bashrc
source ~/.bashrc
```

If `rosdep` is missing you can install it with:
```
sudo apt-get install python3-rosdep python3-rosinstall-generator python3-vcstool python3-rosinstall build-essential`
```
### Start ZED ROS2 Node:

To start the ZED node, open a bash terminal and use the [CLI](https://index.ros.org/doc/ros2/Tutorials/Introspection-with-command-line-tools/) command `ros2 launch`:

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=<camera_model>
```

Replace `<camera_model>` with the model of the camera that you are using: `'zed'`, `'zedm'`, `'zed2'`, `'zed2i'`, `'zedx'`, `'zedxm'`, `'virtual'`,`'zedxonegs'`,`'zedxone4k'`.

```
The `zed_camera.launch.py` is Python launch scripts that automatically start the ZED node using ["manual composition"](https://index.ros.org/doc/ros2/Tutorials/Composition/). The parameters for the indicated camera model are loaded from the relative "YAML files". A Robot State Publisher node is started to publish the camera static links and joints loaded from the URDF model associated with the camera model.

### Set Configurations and Resolutions
You can set your own configurations by modifying the parameters in the files **common_stereo.yaml**, **zed.yaml** **zedm.yaml**, **zed2.yaml**, **zed2i.yaml**, **zedx.yaml**, **zedxm.yaml**, **common_mono.yaml**, **zedxonegs.yaml**, and **zedxone4k.yaml**  available in the folder `zed_wrapper/config`.

If you want to change the resolution of the images publishing, change 2 things.
- Change `grab_resolution: 'HD720'` to `'HD2K', 'HD1080', 'VGA', 'AUTO'` any of them inside `zed_wrapper/config/<camera_model>.yaml`
- Change `pub_resolution: "CUSTOM"` to `pub_resolution: "NATIVE"` inside `zed_wrapper/config/common.yaml`

There are in total 5 depth mode which are `Performance`, `Quality`, `Ultra`, `Neural`, `Neural +`. If you want to change the default depth mode:
- Change `depth_mode: "ULTRA"` to `'PERFORMANCE', 'QUALITY', 'ULTRA', 'NEURAL', 'NEURAL_PLUS'`

# Record ZED ROS2 topics in a rosbag
Now, record the ros2 topics in a rosbag.
```bash
# record all topic
ros2 bag record -a -o ros2_bag_name
# record specific topic
ros2 bag record /tf /tf_static /zed/zed_node/depth/camera_info /zed/zed_node/depth_registered /zed/zed_node/left/camera_info /zed/zed_node/left/image_rect_color /zed/zed_node/point_cloud/cloud_registered /zed/zed_node/pose /zed/zed_node/pose_with_covariance /zed/zed_node/right/camera_info /zed/zed_node/right/image_rect_color /zed/zed_node/imu/data -o ros2_bag_name
# record mentioning compression mode
ros2 bag record -a -o ros2_bag_name --compression-mode file --compression-format zstd 
# record excluding a specific topic
ros2 bag record -a --exclude "/topic_to_exclude" -o ros2_bag_name
```

# Replay ZED ROS2 topics from a rosbag
If you want to replay the rosbag (at default rate, r=1):
```bash
ros2 bag play zed_ros2_topics.bag -r 1
```
# Echo ZED ROS2 topics
If you want to echo any topic:
```bash
ros2 topic echo /zed/zed_node/depth/depth_info
```
Here is the output:
```
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: zed_left_camera_optical_frame
min_depth: 0.6266739368438721
max_depth: 7.9091668128967285

```


