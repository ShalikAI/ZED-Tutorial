# ZED Tutorial
This repository is all about zed and it's implementation in my PhD.

# System Requirements
For this demo, we are using Ubuntu 20.04 / 22.04, ROS1 (ROS Noetic) / ROS2 (ROS Humble), Nvidia Driver 535, Cuda 12.1 and ZED SDK 4.1.4 (pyzed-4.1) and python 3.10. 

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

# Setup ZED ROS1 Workspace

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
```
$ cd ~/zed_ros1_ws/src
$ git clone --recursive https://github.com/stereolabs/zed-ros-wrapper.git
$ git clone https://github.com/stereolabs/zed-ros-interfaces.git
$ cd ../
$ rosdep install --from-paths src --ignore-src -r -y
$ catkin_make -DCMAKE_BUILD_TYPE=Release
$ source ./devel/setup.bash
```
### Run the ZED wrapper

To launch the ZED node for ZED2 camera:
```
$ roslaunch zed_wrapper zed2.launch
```
For other cameras, use `zedm`, `zed`, `zed2i`, `zedx`, `zedxm`. To select the camera from its serial number:
```
$ roslaunch zed_wrapper zed.launch serial_number:=1010 #replace 1010 with the actual SN
```

### Replay ZED ROS topics from a rosbag

If you want to replay the rosbag (at default rate, r=1):
```bash
$ rosbag play zed_ros_topics.bag -r 1
```

If you want to query the info of the rosbag:
```
$ rosbag info zed_ros_topics.bag 
path:        rosbag_record_2024-12-26-21-43-57.bag
version:     2.0
duration:    7.4s
start:       Dec 26 2024 21:43:58.03 (1735271038.03)
end:         Dec 26 2024 21:44:05.39 (1735271045.39)
size:        867.1 MB
messages:    10497
compression: none [606/606 chunks]
types:       diagnostic_msgs/DiagnosticArray         [60810da900de1dd6ddd437c3503511da]
             dynamic_reconfigure/Config              [958f16a05573709014982821e6822580]
             dynamic_reconfigure/ConfigDescription   [757ce9d44ba8ddd801bb30bc456f946f]
             geometry_msgs/PoseStamped               [d3812c3cbc69362b77dc0b19b345f8f5]
             geometry_msgs/PoseWithCovarianceStamped [953b798c0f514ff060a53a3498ce6246]
             geometry_msgs/Transform                 [ac9eff44abf714214112b05d54a3cf9b]
             nav_msgs/Odometry                       [cd5e73d190d741a2f92e81eda573aca7]
             nav_msgs/Path                           [6227e2b7e9cce15051f669a5e197bbf7]
             rosgraph_msgs/Log                       [acffd30cd6b6de30f120938c17c593fb]
             sensor_msgs/CameraInfo                  [c9a58c1b0b154e0e6da7578cb991d214]
             sensor_msgs/CompressedImage             [8f7a12909da2c9d3332d540a0977563f]
             sensor_msgs/FluidPressure               [804dc5cea1c5306d6a2eb80b9833befe]
             sensor_msgs/Image                       [060021388200f6f0f447d0fcd9c64743]
             sensor_msgs/Imu                         [6a62c6daae103f4ff57a132d6f95cec2]
             sensor_msgs/MagneticField               [2f3b0b43eed0c9501de0fa3ff89a45aa]
             sensor_msgs/PointCloud2                 [1158d486dd51d683ce2f1be655c3c181]
             sensor_msgs/Temperature                 [ff71b307acdbe7c871a5a6d7ed359100]
             stereo_msgs/DisparityImage              [04a177815f75271039fa21f16acad8c9]
             tf2_msgs/TFMessage                      [94810edda583a504dfda3829e70d7eec]
             theora_image_transport/Packet           [33ac4e14a7cff32e7e0d65f18bb410f3]
             zed_interfaces/PosTrackStatus           [16c87ef5951f2667d385cacb152a0d50]
topics:      /diagnostics                                                                          7 msgs    : diagnostic_msgs/DiagnosticArray        
             /rosout                                                                             922 msgs    : rosgraph_msgs/Log                       (2 connections)
             /rosout_agg                                                                         822 msgs    : rosgraph_msgs/Log                      
             /tf                                                                                 106 msgs    : tf2_msgs/TFMessage                     
             /tf_static                                                                            2 msgs    : tf2_msgs/TFMessage                      (2 connections)
             /zed2/zed_node/atm_press                                                            178 msgs    : sensor_msgs/FluidPressure              
             /zed2/zed_node/confidence/confidence_map                                             51 msgs    : sensor_msgs/Image                      
             /zed2/zed_node/depth/camera_info                                                     51 msgs    : sensor_msgs/CameraInfo                 
             /zed2/zed_node/depth/depth_registered                                                51 msgs    : sensor_msgs/Image                      
             /zed2/zed_node/depth/depth_registered/compressed/parameter_descriptions               1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/depth/depth_registered/compressed/parameter_updates                    1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/depth/depth_registered/compressedDepth                                51 msgs    : sensor_msgs/CompressedImage            
             /zed2/zed_node/depth/depth_registered/compressedDepth/parameter_descriptions          1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/depth/depth_registered/compressedDepth/parameter_updates               1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/depth/depth_registered/theora                                          3 msgs    : theora_image_transport/Packet          
             /zed2/zed_node/depth/depth_registered/theora/parameter_descriptions                   1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/depth/depth_registered/theora/parameter_updates                        1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/disparity/disparity_image                                             51 msgs    : stereo_msgs/DisparityImage             
             /zed2/zed_node/imu/data                                                            1422 msgs    : sensor_msgs/Imu                        
             /zed2/zed_node/imu/data_raw                                                        1421 msgs    : sensor_msgs/Imu                        
             /zed2/zed_node/imu/mag                                                              355 msgs    : sensor_msgs/MagneticField              
             /zed2/zed_node/left/camera_info                                                      88 msgs    : sensor_msgs/CameraInfo                 
             /zed2/zed_node/left/image_rect_color                                                 44 msgs    : sensor_msgs/Image                      
             /zed2/zed_node/left/image_rect_color/compressed                                      44 msgs    : sensor_msgs/CompressedImage            
             /zed2/zed_node/left/image_rect_color/compressed/parameter_descriptions                1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/left/image_rect_color/compressed/parameter_updates                     1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/left/image_rect_color/compressedDepth/parameter_descriptions           1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/left/image_rect_color/compressedDepth/parameter_updates                1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/left/image_rect_color/theora                                          47 msgs    : theora_image_transport/Packet          
             /zed2/zed_node/left/image_rect_color/theora/parameter_descriptions                    1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/left/image_rect_color/theora/parameter_updates                         1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/left/image_rect_gray                                                  54 msgs    : sensor_msgs/Image                      
             /zed2/zed_node/left/image_rect_gray/compressed                                       54 msgs    : sensor_msgs/CompressedImage            
             /zed2/zed_node/left/image_rect_gray/compressed/parameter_descriptions                 1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/left/image_rect_gray/compressed/parameter_updates                      1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/left/image_rect_gray/compressedDepth/parameter_descriptions            1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/left/image_rect_gray/compressedDepth/parameter_updates                 1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/left/image_rect_gray/theora                                           57 msgs    : theora_image_transport/Packet          
             /zed2/zed_node/left/image_rect_gray/theora/parameter_descriptions                     1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/left/image_rect_gray/theora/parameter_updates                          1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/left_cam_imu_transform                                                 1 msg     : geometry_msgs/Transform                
             /zed2/zed_node/left_raw/camera_info                                                 106 msgs    : sensor_msgs/CameraInfo                 
             /zed2/zed_node/left_raw/image_raw_color                                              45 msgs    : sensor_msgs/Image                      
             /zed2/zed_node/left_raw/image_raw_color/compressed                                   45 msgs    : sensor_msgs/CompressedImage            
             /zed2/zed_node/left_raw/image_raw_color/compressed/parameter_descriptions             1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/left_raw/image_raw_color/compressed/parameter_updates                  1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/left_raw/image_raw_color/compressedDepth/parameter_descriptions        1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/left_raw/image_raw_color/compressedDepth/parameter_updates             1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/left_raw/image_raw_color/theora                                       48 msgs    : theora_image_transport/Packet          
             /zed2/zed_node/left_raw/image_raw_color/theora/parameter_descriptions                 1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/left_raw/image_raw_color/theora/parameter_updates                      1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/left_raw/image_raw_gray                                               53 msgs    : sensor_msgs/Image                      
             /zed2/zed_node/left_raw/image_raw_gray/compressed                                    53 msgs    : sensor_msgs/CompressedImage            
             /zed2/zed_node/left_raw/image_raw_gray/compressed/parameter_descriptions              1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/left_raw/image_raw_gray/compressed/parameter_updates                   1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/left_raw/image_raw_gray/compressedDepth/parameter_descriptions         1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/left_raw/image_raw_gray/compressedDepth/parameter_updates              1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/left_raw/image_raw_gray/theora                                        56 msgs    : theora_image_transport/Packet          
             /zed2/zed_node/left_raw/image_raw_gray/theora/parameter_descriptions                  1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/left_raw/image_raw_gray/theora/parameter_updates                       1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/odom                                                                  52 msgs    : nav_msgs/Odometry                      
             /zed2/zed_node/parameter_descriptions                                                 1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/parameter_updates                                                     12 msgs    : dynamic_reconfigure/Config             
             /zed2/zed_node/path_map                                                              14 msgs    : nav_msgs/Path                          
             /zed2/zed_node/path_odom                                                             14 msgs    : nav_msgs/Path                          
             /zed2/zed_node/point_cloud/cloud_registered                                          51 msgs    : sensor_msgs/PointCloud2                
             /zed2/zed_node/pose                                                                  52 msgs    : geometry_msgs/PoseStamped              
             /zed2/zed_node/pose/status                                                          104 msgs    : zed_interfaces/PosTrackStatus          
             /zed2/zed_node/pose_with_covariance                                                  52 msgs    : geometry_msgs/PoseWithCovarianceStamped
             /zed2/zed_node/rgb/camera_info                                                      107 msgs    : sensor_msgs/CameraInfo                 
             /zed2/zed_node/rgb/image_rect_color                                                  54 msgs    : sensor_msgs/Image                      
             /zed2/zed_node/rgb/image_rect_color/compressed                                       54 msgs    : sensor_msgs/CompressedImage            
             /zed2/zed_node/rgb/image_rect_color/compressed/parameter_descriptions                 1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/rgb/image_rect_color/compressed/parameter_updates                      1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/rgb/image_rect_color/compressedDepth/parameter_descriptions            1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/rgb/image_rect_color/compressedDepth/parameter_updates                 1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/rgb/image_rect_color/theora                                           57 msgs    : theora_image_transport/Packet          
             /zed2/zed_node/rgb/image_rect_color/theora/parameter_descriptions                     1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/rgb/image_rect_color/theora/parameter_updates                          1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/rgb/image_rect_gray                                                   54 msgs    : sensor_msgs/Image                      
             /zed2/zed_node/rgb/image_rect_gray/compressed                                        54 msgs    : sensor_msgs/CompressedImage            
             /zed2/zed_node/rgb/image_rect_gray/compressed/parameter_descriptions                  1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/rgb/image_rect_gray/compressed/parameter_updates                       1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/rgb/image_rect_gray/compressedDepth/parameter_descriptions             1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/rgb/image_rect_gray/compressedDepth/parameter_updates                  1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/rgb/image_rect_gray/theora                                            56 msgs    : theora_image_transport/Packet          
             /zed2/zed_node/rgb/image_rect_gray/theora/parameter_descriptions                      1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/rgb/image_rect_gray/theora/parameter_updates                           1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/rgb_raw/camera_info                                                   84 msgs    : sensor_msgs/CameraInfo                 
             /zed2/zed_node/rgb_raw/image_raw_color                                               42 msgs    : sensor_msgs/Image                      
             /zed2/zed_node/rgb_raw/image_raw_color/compressed                                    53 msgs    : sensor_msgs/CompressedImage            
             /zed2/zed_node/rgb_raw/image_raw_color/compressed/parameter_descriptions              1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/rgb_raw/image_raw_color/compressed/parameter_updates                   1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/rgb_raw/image_raw_color/compressedDepth/parameter_descriptions         1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/rgb_raw/image_raw_color/compressedDepth/parameter_updates              1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/rgb_raw/image_raw_color/theora                                        47 msgs    : theora_image_transport/Packet          
             /zed2/zed_node/rgb_raw/image_raw_color/theora/parameter_descriptions                  1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/rgb_raw/image_raw_color/theora/parameter_updates                       1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/rgb_raw/image_raw_gray                                                53 msgs    : sensor_msgs/Image                      
             /zed2/zed_node/rgb_raw/image_raw_gray/compressed                                     53 msgs    : sensor_msgs/CompressedImage            
             /zed2/zed_node/rgb_raw/image_raw_gray/compressed/parameter_descriptions               1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/rgb_raw/image_raw_gray/compressed/parameter_updates                    1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/rgb_raw/image_raw_gray/compressedDepth/parameter_descriptions          1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/rgb_raw/image_raw_gray/compressedDepth/parameter_updates               1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/rgb_raw/image_raw_gray/theora                                         56 msgs    : theora_image_transport/Packet          
             /zed2/zed_node/rgb_raw/image_raw_gray/theora/parameter_descriptions                   1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/rgb_raw/image_raw_gray/theora/parameter_updates                        1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/right/camera_info                                                    106 msgs    : sensor_msgs/CameraInfo                 
             /zed2/zed_node/right/image_rect_color                                                53 msgs    : sensor_msgs/Image                      
             /zed2/zed_node/right/image_rect_color/compressed                                     53 msgs    : sensor_msgs/CompressedImage            
             /zed2/zed_node/right/image_rect_color/compressed/parameter_descriptions               1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/right/image_rect_color/compressed/parameter_updates                    1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/right/image_rect_color/compressedDepth/parameter_descriptions          1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/right/image_rect_color/compressedDepth/parameter_updates               1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/right/image_rect_color/theora                                         56 msgs    : theora_image_transport/Packet          
             /zed2/zed_node/right/image_rect_color/theora/parameter_descriptions                   1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/right/image_rect_color/theora/parameter_updates                        1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/right/image_rect_gray                                                 52 msgs    : sensor_msgs/Image                      
             /zed2/zed_node/right/image_rect_gray/compressed                                      52 msgs    : sensor_msgs/CompressedImage            
             /zed2/zed_node/right/image_rect_gray/compressed/parameter_descriptions                1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/right/image_rect_gray/compressed/parameter_updates                     1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/right/image_rect_gray/compressedDepth/parameter_descriptions           1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/right/image_rect_gray/compressedDepth/parameter_updates                1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/right/image_rect_gray/theora                                          55 msgs    : theora_image_transport/Packet          
             /zed2/zed_node/right/image_rect_gray/theora/parameter_descriptions                    1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/right/image_rect_gray/theora/parameter_updates                         1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/right_raw/camera_info                                                106 msgs    : sensor_msgs/CameraInfo                 
             /zed2/zed_node/right_raw/image_raw_color                                             53 msgs    : sensor_msgs/Image                      
             /zed2/zed_node/right_raw/image_raw_color/compressed                                  53 msgs    : sensor_msgs/CompressedImage            
             /zed2/zed_node/right_raw/image_raw_color/compressed/parameter_descriptions            1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/right_raw/image_raw_color/compressed/parameter_updates                 1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/right_raw/image_raw_color/compressedDepth/parameter_descriptions       1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/right_raw/image_raw_color/compressedDepth/parameter_updates            1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/right_raw/image_raw_color/theora                                      56 msgs    : theora_image_transport/Packet          
             /zed2/zed_node/right_raw/image_raw_color/theora/parameter_descriptions                1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/right_raw/image_raw_color/theora/parameter_updates                     1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/right_raw/image_raw_gray                                              52 msgs    : sensor_msgs/Image                      
             /zed2/zed_node/right_raw/image_raw_gray/compressed                                   52 msgs    : sensor_msgs/CompressedImage            
             /zed2/zed_node/right_raw/image_raw_gray/compressed/parameter_descriptions             1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/right_raw/image_raw_gray/compressed/parameter_updates                  1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/right_raw/image_raw_gray/compressedDepth/parameter_descriptions        1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/right_raw/image_raw_gray/compressedDepth/parameter_updates             1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/right_raw/image_raw_gray/theora                                       55 msgs    : theora_image_transport/Packet          
             /zed2/zed_node/right_raw/image_raw_gray/theora/parameter_descriptions                 1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/right_raw/image_raw_gray/theora/parameter_updates                      1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/stereo/image_rect_color                                               51 msgs    : sensor_msgs/Image                      
             /zed2/zed_node/stereo/image_rect_color/compressed                                    51 msgs    : sensor_msgs/CompressedImage            
             /zed2/zed_node/stereo/image_rect_color/compressed/parameter_descriptions              1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/stereo/image_rect_color/compressed/parameter_updates                   1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/stereo/image_rect_color/compressedDepth/parameter_descriptions         1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/stereo/image_rect_color/compressedDepth/parameter_updates              1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/stereo/image_rect_color/theora                                        54 msgs    : theora_image_transport/Packet          
             /zed2/zed_node/stereo/image_rect_color/theora/parameter_descriptions                  1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/stereo/image_rect_color/theora/parameter_updates                       1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/stereo_raw/image_raw_color                                            51 msgs    : sensor_msgs/Image                      
             /zed2/zed_node/stereo_raw/image_raw_color/compressed                                 51 msgs    : sensor_msgs/CompressedImage            
             /zed2/zed_node/stereo_raw/image_raw_color/compressed/parameter_descriptions           1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/stereo_raw/image_raw_color/compressed/parameter_updates                1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/stereo_raw/image_raw_color/compressedDepth/parameter_descriptions      1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/stereo_raw/image_raw_color/compressedDepth/parameter_updates           1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/stereo_raw/image_raw_color/theora                                     54 msgs    : theora_image_transport/Packet          
             /zed2/zed_node/stereo_raw/image_raw_color/theora/parameter_descriptions               1 msg     : dynamic_reconfigure/ConfigDescription  
             /zed2/zed_node/stereo_raw/image_raw_color/theora/parameter_updates                    1 msg     : dynamic_reconfigure/Config             
             /zed2/zed_node/temperature/imu                                                     1421 msgs    : sensor_msgs/Temperature                
             /zed2/zed_node/temperature/left                                                     178 msgs    : sensor_msgs/Temperature                
             /zed2/zed_node/temperature/right                                                    178 msgs    : sensor_msgs/Temperature
```
### Echo ZED ROS1 topics

Depth topic echo (32 bit): 
```
$ rostopic echo /zed2/zed_node/depth/depth_registered
header: 
  seq: 0
  stamp: 
    secs: 1735281348
    nsecs: 908521465
  frame_id: "zed2_left_camera_optical_frame"
height: 360
width: 640
encoding: "32FC1"
is_bigendian: 0
step: 2560
data: [255, 255, 255, 127, 255, 255, 255, 127, 255, 255, 255, 127, 255, 255,..]
```

Depth topic echo (16 bit): 
```
$ rostopic echo /zedm/zed_node/depth/depth_registered
header: 
  seq: 0
  stamp: 
    secs: 1737399426
    nsecs: 106011058
  frame_id: "zedm_left_camera_optical_frame"
height: 1080
width: 1920
encoding: "16UC1"
is_bigendian: 0
step: 3840
data: [238, 7, 236, 7, 232, 7, ...]
```

In ROS1 for zed cameras, 32 bit float in meters and 16 bit unsigned int in millimeters are used and can be launched. You have to change the parameters inside `common.yaml` file of `params` folder. Specially, `openni_depth_mode` should be `false` for `32bit float` meter units (32FC1) and should be set to `true` for 16bit uchar (mono16) millimeter units. 

Depth camera info topic echo:
```
$ rostopic echo /zed2/zed_node/depth/camera_info
header: 
  seq: 4
  stamp: 
    secs: 1735281502
    nsecs: 250405465
  frame_id: "zed2_left_camera_optical_frame"
height: 360
width: 640
distortion_model: "plumb_bob"
D: [0.0, 0.0, 0.0, 0.0, 0.0]
K: [261.0508728027344, 0.0, 316.148193359375, 0.0, 261.0508728027344, 181.02622985839844, 0.0, 0.0, 1.0]
R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P: [261.0508728027344, 0.0, 316.148193359375, 0.0, 0.0, 261.0508728027344, 181.02622985839844, 0.0, 0.0, 0.0, 1.0, 0.0]
binning_x: 0
binning_y: 0
roi: 
  x_offset: 0
  y_offset: 0
  height: 0
  width: 0
  do_rectify: False
```

IMU data info topic echo:
```
$ rostopic echo /zed2/zed_node/imu/data
header: 
  seq: 0
  stamp: 
    secs: 1735279660
    nsecs: 542892147
  frame_id: "zed2_imu_link"
orientation: 
  x: 0.0019067992689087987
  y: 0.01964346133172512
  z: -0.0008004685514606535
  w: 0.9998049139976501
orientation_covariance: [3.508294485066751e-10, 2.5143494870309872e-11, 5.7391818264666e-12, 2.514349703474694e-11, 6.3874578311802315e-09, 4.385051083941749e-13, 5.739183990903668e-12, 4.385046011042373e-13, 3.420512536438746e-10]
angular_velocity: 
  x: -0.0001717870612978656
  y: 6.668106762208743e-05
  z: -0.0005567486654310972
angular_velocity_covariance: [6.623836107184411e-10, 0.0, 0.0, 0.0, 5.743454273726701e-10, 0.0, 0.0, -0.0, 5.877751533323972e-10]
linear_acceleration: 
  x: -0.37852194905281067
  y: 0.041714418679475784
  z: 9.770483016967773
linear_acceleration_covariance: [0.007800576277077198, 0.0, 0.0, 0.0, 0.00585215026512742, 0.0, 0.0, -0.0, 0.008417878299951553]
```

Left camera topic echo:
```
$ rostopic echo /zed2/zed_node/left/image_rect_color
header: 
  seq: 4
  stamp: 
    secs: 1735278948
    nsecs: 606446465
  frame_id: "zed2_left_camera_optical_frame"
height: 360
width: 640
encoding: "bgra8"
is_bigendian: 0
step: 2560
data: [61, 34, 41, 255, 61, 36, 43, 255, 63, 37, 43, 255, 64, 40, 43, 255...]
```

The alpha channel in an image `bgra8` represents transparency or opacity for each pixel. While typical images in formats like JPEG / JPG use three color channels—red, green, and blue (RGB), PNG images with an alpha channel add a fourth channel to indicate how opaque or transparent each pixel is.

R (Red): Intensity of red color in a pixel, G (Green): Intensity of green color in a pixel, B (Blue): Intensity of blue color in a pixel, and A (Alpha): Opacity or transparency level of a pixel. An alpha value of 0 makes a pixel fully transparent, 255 makes it fully opaque, and values in between create partial transparency for blending effects.

When the ZED camera provides `bgra8` encoded images, the alpha channel is typically always fully opaque (255), meaning the images are completely visible without any transparency.

Left camera info topic echo:
```
$ rostopic echo -n1 /zed2/zed_node/left/camera_info
header: 
  seq: 8
  stamp: 
    secs: 1735279367
    nsecs: 629950465
  frame_id: "zed2_left_camera_optical_frame"
height: 360
width: 640
distortion_model: "plumb_bob"
D: [0.0, 0.0, 0.0, 0.0, 0.0]
K: [261.0508728027344, 0.0, 316.148193359375, 0.0, 261.0508728027344, 181.02622985839844, 0.0, 0.0, 1.0]
R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P: [261.0508728027344, 0.0, 316.148193359375, 0.0, 0.0, 261.0508728027344, 181.02622985839844, 0.0, 0.0, 0.0, 1.0, 0.0]
binning_x: 0
binning_y: 0
roi: 
  x_offset: 0
  y_offset: 0
  height: 0
  width: 0
  do_rectify: False
```

RGB camera topic echo:
```
$ rostopic echo /zed2/zed_node/rgb/image_rect_color
header: 
  seq: 0
  stamp: 
    secs: 1735278805
    nsecs: 864969465
  frame_id: "zed2_left_camera_optical_frame"
height: 360
width: 640
encoding: "bgra8"
is_bigendian: 0
step: 2560
data: [53, 32, 37, 255, 62, 37, 44, 255, 62, 36, 43, 255, 63, 33, 39, 255, 63, 34, 40, 255,..]
```

RGB camera info topic echo:
```
$ rostopic echo /zed2/zed_node/rgb/camera_info
header: 
  seq: 3
  stamp: 
    secs: 1735279113
    nsecs: 582182465
  frame_id: "zed2_left_camera_optical_frame"
height: 360
width: 640
distortion_model: "plumb_bob"
D: [0.0, 0.0, 0.0, 0.0, 0.0]
K: [261.0508728027344, 0.0, 316.148193359375, 0.0, 261.0508728027344, 181.02622985839844, 0.0, 0.0, 1.0]
R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P: [261.0508728027344, 0.0, 316.148193359375, 0.0, 0.0, 261.0508728027344, 181.02622985839844, 0.0, 0.0, 0.0, 1.0, 0.0]
binning_x: 0
binning_y: 0
roi: 
  x_offset: 0
  y_offset: 0
  height: 0
  width: 0
  do_rectify: False
```

Left camera monocular topic echo:
```
$ rostopic echo /zedm/zed_node/left/image_rect_gray
header: 
  seq: 5250
  stamp: 
    secs: 1737392916
    nsecs: 404275541
  frame_id: "zedm_left_camera_optical_frame"
height: 1080
width: 1920
encoding: "mono8"
is_bigendian: 0
step: 1920
data: [49, 49, 49, 50, ...]
```

Right camera topic echo:
```
$ rostopic echo /zed2/zed_node/right/image_rect_color
header: 
  seq: 0
  stamp: 
    secs: 1735279015
    nsecs: 576962465
  frame_id: "zed2_right_camera_optical_frame"
height: 360
width: 640
encoding: "bgra8"
is_bigendian: 0
step: 2560
data: [9, 22, 24, 255, 11, 22, 26, 255, 13, 24, 28, 255, 11, 23, 25, 255,..]
```

Right camera info topic echo:
```
$ rostopic echo /zed2/zed_node/right/camera_info
header: 
  seq: 5
  stamp: 
    secs: 1735279269
    nsecs: 590939465
  frame_id: "zed2_right_camera_optical_frame"
height: 360
width: 640
distortion_model: "plumb_bob"
D: [0.0, 0.0, 0.0, 0.0, 0.0]
K: [261.0508728027344, 0.0, 316.148193359375, 0.0, 261.0508728027344, 181.02622985839844, 0.0, 0.0, 1.0]
R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P: [261.0508728027344, 0.0, 316.148193359375, -31.317344665527344, 0.0, 261.0508728027344, 181.02622985839844, 0.0, 0.0, 0.0, 1.0, 0.0]
binning_x: 0
binning_y: 0
roi: 
  x_offset: 0
  y_offset: 0
  height: 0
  width: 0
  do_rectify: False
```
PointCloud topic echo:
```
$ rostopic echo /zed2/zed_node/point_cloud/cloud_registered
header: 
  seq: 5
  stamp: 
    secs: 1735285911
    nsecs: 666465292
  frame_id: "zed2_left_camera_frame"
height: 360
width: 640
fields: 
  - 
    name: "x"
    offset: 0
    datatype: 7
    count: 1
  - 
    name: "y"
    offset: 4
    datatype: 7
    count: 1
  - 
    name: "z"
    offset: 8
    datatype: 7
    count: 1
  - 
    name: "rgb"
    offset: 12
    datatype: 7
    count: 1
is_bigendian: False
point_step: 16
row_step: 10240
data: [255, 255, 255, 127, 255, 255, 255, 127, 255,..]
is_dense: False
```

TF topic echo:
```
$ rostopic echo /tf
transforms: 
  - 
    header: 
      seq: 0
      stamp: 
        secs: 1735283536
        nsecs: 198755292
      frame_id: "odom"
    child_frame_id: "base_link"
    transform: 
      translation: 
        x: 0.0
        y: 0.0
        z: 0.0
      rotation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
---
transforms: 
  - 
    header: 
      seq: 0
      stamp: 
        secs: 1735283536
        nsecs: 198755292
      frame_id: "map"
    child_frame_id: "odom"
    transform: 
      translation: 
        x: -0.00999281249476458
        y: 0.059999058377506
        z: 0.015500221462002136
      rotation: 
        x: 0.0022743462719155513
        y: 0.019449826117637875
        z: -0.00011381217288419188
        w: 0.9998082409441414
```

TF static topic echo:
```
arghya@arghya-Pulse-GL66-12UEK:~/zed_ws$ rostopic echo /tf_static
transforms: 
  - 
    header: 
      seq: 0
      stamp: 
        secs: 1735283503
        nsecs: 254296778
      frame_id: "zed2_camera_center"
    child_frame_id: "zed2_baro_link"
    transform: 
      translation: 
        x: 0.0
        y: 0.0
        z: 0.0
      rotation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
  - 
    header: 
      seq: 0
      stamp: 
        secs: 1735283503
        nsecs: 254300822
      frame_id: "base_link"
    child_frame_id: "zed2_base_link"
    transform: 
      translation: 
        x: 0.0
        y: 0.0
        z: 0.0
      rotation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
  - 
    header: 
      seq: 0
      stamp: 
        secs: 1735283503
        nsecs: 254301767
      frame_id: "zed2_base_link"
    child_frame_id: "zed2_camera_center"
    transform: 
      translation: 
        x: 0.0
        y: 0.0
        z: 0.015
      rotation: 
        x: 0.0
        y: 0.024997395914712332
        z: 0.0
        w: 0.9996875162757025
  - 
    header: 
      seq: 0
      stamp: 
        secs: 1735283503
        nsecs: 254302138
      frame_id: "zed2_camera_center"
    child_frame_id: "zed2_left_camera_frame"
    transform: 
      translation: 
        x: -0.01
        y: 0.06
        z: 0.0
      rotation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
  - 
    header: 
      seq: 0
      stamp: 
        secs: 1735283503
        nsecs: 254302550
      frame_id: "zed2_left_camera_frame"
    child_frame_id: "zed2_left_camera_optical_frame"
    transform: 
      translation: 
        x: 0.0
        y: 0.0
        z: 0.0
      rotation: 
        x: 0.5
        y: -0.4999999999999999
        z: 0.5
        w: -0.5000000000000001
  - 
    header: 
      seq: 0
      stamp: 
        secs: 1735283503
        nsecs: 254306274
      frame_id: "zed2_camera_center"
    child_frame_id: "zed2_mag_link"
    transform: 
      translation: 
        x: 0.0
        y: 0.0
        z: 0.0
      rotation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
  - 
    header: 
      seq: 0
      stamp: 
        secs: 1735283503
        nsecs: 254306512
      frame_id: "zed2_camera_center"
    child_frame_id: "zed2_right_camera_frame"
    transform: 
      translation: 
        x: -0.01
        y: -0.06
        z: 0.0
      rotation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
  - 
    header: 
      seq: 0
      stamp: 
        secs: 1735283503
        nsecs: 254307016
      frame_id: "zed2_right_camera_frame"
    child_frame_id: "zed2_right_camera_optical_frame"
    transform: 
      translation: 
        x: 0.0
        y: 0.0
        z: 0.0
      rotation: 
        x: 0.5
        y: -0.4999999999999999
        z: 0.5
        w: -0.5000000000000001
  - 
    header: 
      seq: 0
      stamp: 
        secs: 1735283503
        nsecs: 254307411
      frame_id: "zed2_left_camera_frame"
    child_frame_id: "zed2_temp_left_link"
    transform: 
      translation: 
        x: 0.0
        y: 0.0
        z: 0.0
      rotation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
  - 
    header: 
      seq: 0
      stamp: 
        secs: 1735283503
        nsecs: 254308268
      frame_id: "zed2_right_camera_frame"
    child_frame_id: "zed2_temp_right_link"
    transform: 
      translation: 
        x: 0.0
        y: 0.0
        z: 0.0
      rotation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
---
transforms: 
  - 
    header: 
      seq: 0
      stamp: 
        secs: 1735283507
        nsecs: 257780353
      frame_id: "zed2_left_camera_frame"
    child_frame_id: "zed2_imu_link"
    transform: 
      translation: 
        x: -0.0020000000949949026
        y: -0.023061001673340797
        z: 0.00021700002253055573
      rotation: 
        x: -0.0005459150415845215
        y: 0.0014597486006096005
        z: -0.0019339574500918388
        w: 0.9999969005584717
```
Odometry topic echo:
```
$ rostopic echo /zed2/zed_node/odom
header: 
  seq: 0
  stamp: 
    secs: 1735279500
    nsecs: 570706465
  frame_id: "odom"
child_frame_id: "base_link"
pose: 
  pose: 
    position: 
      x: 0.0
      y: 0.0
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
  covariance: [1.000000013351432e-10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.000000013351432e-10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.000000013351432e-10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.000000013351432e-10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.000000013351432e-10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.000000013351432e-10]
twist: 
  twist: 
    linear: 
      x: 0.0
      y: 0.0
      z: 0.0
    angular: 
      x: 0.0
      y: 0.0
      z: 0.0
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

Pose topic echo:
```
$ rostopic echo /zed2/zed_node/pose
header: 
  seq: 0
  stamp: 
    secs: 1735279462
    nsecs:  68683465
  frame_id: "map"
pose: 
  position: 
    x: -0.009993596057798843
    y: 0.059998770635863764
    z: 0.015500830393179694
  orientation: 
    x: 0.0026877267207852337
    y: 0.019631320670887617
    z: -0.00013449843675467787
    w: 0.999803665368337
```

Pose with covariance topic echo:
```
$ rostopic echo /zed2/zed_node/pose_with_covariance
header: 
  seq: 0
  stamp: 
    secs: 1735279595
    nsecs: 476123465
  frame_id: "map"
pose: 
  pose: 
    position: 
      x: -0.009993596057798843
      y: 0.059998770635863764
      z: 0.015500830393179694
    orientation: 
      x: 0.0026877267207852337
      y: 0.019631320670887617
      z: -0.00013449843675467787
      w: 0.999803665368337
  covariance: [1.000000013351432e-10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.000000013351432e-10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.000000013351432e-10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.000000013351432e-10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.000000013351432e-10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.000000013351432e-10]
```

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

## ROS1 Interfaces

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
/zed/zed_node/depth/depth_info
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
If you want to replay the ros2 bag (at default rate, r=1):
```bash
ros2 bag play zed_ros2_topics.bag -r 1
```
If you want to query the info of the ros2 bag:
```
$ ros2 bag info zed_rosbag_0.db3 

Files:             zed_rosbag_0.db3
Bag size:          400.3 MiB
Storage id:        sqlite3
Duration:          3.917s
Start:             Dec 26 2024 21:26:04.386 (1735269964.386)
End:               Dec 26 2024 21:26:08.304 (1735269968.304)
Messages:          3226
Topic information: Topic: /diagnostics | Type: diagnostic_msgs/msg/DiagnosticArray | Count: 4 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 612 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 79 | Serialization Format: cdr
                   Topic: /zed/robot_description | Type: std_msgs/msg/String | Count: 1 | Serialization Format: cdr
                   Topic: /zed/zed_node/atm_press | Type: sensor_msgs/msg/FluidPressure | Count: 96 | Serialization Format: cdr
                   Topic: /zed/zed_node/depth/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 6 | Serialization Format: cdr
                   Topic: /zed/zed_node/depth/depth_info | Type: zed_interfaces/msg/DepthInfoStamped | Count: 5 | Serialization Format: cdr
                   Topic: /zed/zed_node/depth/depth_registered | Type: sensor_msgs/msg/Image | Count: 6 | Serialization Format: cdr
                   Topic: /zed/zed_node/depth/depth_registered/compressedDepth | Type: sensor_msgs/msg/CompressedImage | Count: 6 | Serialization Format: cdr
                   Topic: /zed/zed_node/depth/depth_registered/theora | Type: theora_image_transport/msg/Packet | Count: 3 | Serialization Format: cdr
                   Topic: /zed/zed_node/disparity/disparity_image | Type: stereo_msgs/msg/DisparityImage | Count: 6 | Serialization Format: cdr
                   Topic: /zed/zed_node/imu/data | Type: sensor_msgs/msg/Imu | Count: 593 | Serialization Format: cdr
                   Topic: /zed/zed_node/imu/data_raw | Type: sensor_msgs/msg/Imu | Count: 595 | Serialization Format: cdr
                   Topic: /zed/zed_node/imu/mag | Type: sensor_msgs/msg/MagneticField | Count: 190 | Serialization Format: cdr
                   Topic: /zed/zed_node/left/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 7 | Serialization Format: cdr
                   Topic: /zed/zed_node/left/image_rect_color | Type: sensor_msgs/msg/Image | Count: 7 | Serialization Format: cdr
                   Topic: /zed/zed_node/left/image_rect_color/compressed | Type: sensor_msgs/msg/CompressedImage | Count: 7 | Serialization Format: cdr
                   Topic: /zed/zed_node/left/image_rect_color/theora | Type: theora_image_transport/msg/Packet | Count: 10 | Serialization Format: cdr
                   Topic: /zed/zed_node/left_cam_imu_transform | Type: geometry_msgs/msg/TransformStamped | Count: 599 | Serialization Format: cdr
                   Topic: /zed/zed_node/left_gray/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 6 | Serialization Format: cdr
                   Topic: /zed/zed_node/left_gray/image_rect_gray | Type: sensor_msgs/msg/Image | Count: 6 | Serialization Format: cdr
                   Topic: /zed/zed_node/left_gray/image_rect_gray/compressed | Type: sensor_msgs/msg/CompressedImage | Count: 6 | Serialization Format: cdr
                   Topic: /zed/zed_node/left_gray/image_rect_gray/theora | Type: theora_image_transport/msg/Packet | Count: 9 | Serialization Format: cdr
                   Topic: /zed/zed_node/left_raw/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 6 | Serialization Format: cdr
                   Topic: /zed/zed_node/left_raw/image_raw_color | Type: sensor_msgs/msg/Image | Count: 7 | Serialization Format: cdr
                   Topic: /zed/zed_node/left_raw/image_raw_color/compressed | Type: sensor_msgs/msg/CompressedImage | Count: 7 | Serialization Format: cdr
                   Topic: /zed/zed_node/left_raw/image_raw_color/theora | Type: theora_image_transport/msg/Packet | Count: 9 | Serialization Format: cdr
                   Topic: /zed/zed_node/left_raw_gray/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 6 | Serialization Format: cdr
                   Topic: /zed/zed_node/left_raw_gray/image_raw_gray | Type: sensor_msgs/msg/Image | Count: 6 | Serialization Format: cdr
                   Topic: /zed/zed_node/left_raw_gray/image_raw_gray/compressed | Type: sensor_msgs/msg/CompressedImage | Count: 6 | Serialization Format: cdr
                   Topic: /zed/zed_node/left_raw_gray/image_raw_gray/theora | Type: theora_image_transport/msg/Packet | Count: 9 | Serialization Format: cdr
                   Topic: /zed/zed_node/odom | Type: nav_msgs/msg/Odometry | Count: 6 | Serialization Format: cdr
                   Topic: /zed/zed_node/path_map | Type: nav_msgs/msg/Path | Count: 7 | Serialization Format: cdr
                   Topic: /zed/zed_node/path_odom | Type: nav_msgs/msg/Path | Count: 7 | Serialization Format: cdr
                   Topic: /zed/zed_node/point_cloud/cloud_registered | Type: sensor_msgs/msg/PointCloud2 | Count: 6 | Serialization Format: cdr
                   Topic: /zed/zed_node/pose | Type: geometry_msgs/msg/PoseStamped | Count: 6 | Serialization Format: cdr
                   Topic: /zed/zed_node/pose/status | Type: zed_interfaces/msg/PosTrackStatus | Count: 6 | Serialization Format: cdr
                   Topic: /zed/zed_node/pose_with_covariance | Type: geometry_msgs/msg/PoseWithCovarianceStamped | Count: 6 | Serialization Format: cdr
                   Topic: /zed/zed_node/rgb/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 7 | Serialization Format: cdr
                   Topic: /zed/zed_node/rgb/image_rect_color | Type: sensor_msgs/msg/Image | Count: 7 | Serialization Format: cdr
                   Topic: /zed/zed_node/rgb/image_rect_color/compressed | Type: sensor_msgs/msg/CompressedImage | Count: 7 | Serialization Format: cdr
                   Topic: /zed/zed_node/rgb/image_rect_color/theora | Type: theora_image_transport/msg/Packet | Count: 10 | Serialization Format: cdr
                   Topic: /zed/zed_node/rgb_gray/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 6 | Serialization Format: cdr
                   Topic: /zed/zed_node/rgb_gray/image_rect_gray | Type: sensor_msgs/msg/Image | Count: 6 | Serialization Format: cdr
                   Topic: /zed/zed_node/rgb_gray/image_rect_gray/compressed | Type: sensor_msgs/msg/CompressedImage | Count: 6 | Serialization Format: cdr
                   Topic: /zed/zed_node/rgb_gray/image_rect_gray/theora | Type: theora_image_transport/msg/Packet | Count: 9 | Serialization Format: cdr
                   Topic: /zed/zed_node/rgb_raw/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 6 | Serialization Format: cdr
                   Topic: /zed/zed_node/rgb_raw/image_raw_color | Type: sensor_msgs/msg/Image | Count: 6 | Serialization Format: cdr
                   Topic: /zed/zed_node/rgb_raw/image_raw_color/compressed | Type: sensor_msgs/msg/CompressedImage | Count: 6 | Serialization Format: cdr
                   Topic: /zed/zed_node/rgb_raw/image_raw_color/theora | Type: theora_image_transport/msg/Packet | Count: 9 | Serialization Format: cdr
                   Topic: /zed/zed_node/rgb_raw_gray/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 6 | Serialization Format: cdr
                   Topic: /zed/zed_node/rgb_raw_gray/image_raw_gray | Type: sensor_msgs/msg/Image | Count: 6 | Serialization Format: cdr
                   Topic: /zed/zed_node/rgb_raw_gray/image_raw_gray/compressed | Type: sensor_msgs/msg/CompressedImage | Count: 6 | Serialization Format: cdr
                   Topic: /zed/zed_node/rgb_raw_gray/image_raw_gray/theora | Type: theora_image_transport/msg/Packet | Count: 9 | Serialization Format: cdr
                   Topic: /zed/zed_node/right/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 6 | Serialization Format: cdr
                   Topic: /zed/zed_node/right/image_rect_color | Type: sensor_msgs/msg/Image | Count: 6 | Serialization Format: cdr
                   Topic: /zed/zed_node/right/image_rect_color/compressed | Type: sensor_msgs/msg/CompressedImage | Count: 6 | Serialization Format: cdr
                   Topic: /zed/zed_node/right/image_rect_color/theora | Type: theora_image_transport/msg/Packet | Count: 9 | Serialization Format: cdr
                   Topic: /zed/zed_node/right_gray/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 5 | Serialization Format: cdr
                   Topic: /zed/zed_node/right_gray/image_rect_gray | Type: sensor_msgs/msg/Image | Count: 5 | Serialization Format: cdr
                   Topic: /zed/zed_node/right_gray/image_rect_gray/compressed | Type: sensor_msgs/msg/CompressedImage | Count: 5 | Serialization Format: cdr
                   Topic: /zed/zed_node/right_gray/image_rect_gray/theora | Type: theora_image_transport/msg/Packet | Count: 8 | Serialization Format: cdr
                   Topic: /zed/zed_node/right_raw/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 6 | Serialization Format: cdr
                   Topic: /zed/zed_node/right_raw/image_raw_color | Type: sensor_msgs/msg/Image | Count: 6 | Serialization Format: cdr
                   Topic: /zed/zed_node/right_raw/image_raw_color/compressed | Type: sensor_msgs/msg/CompressedImage | Count: 6 | Serialization Format: cdr
                   Topic: /zed/zed_node/right_raw/image_raw_color/theora | Type: theora_image_transport/msg/Packet | Count: 9 | Serialization Format: cdr
                   Topic: /zed/zed_node/right_raw_gray/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 6 | Serialization Format: cdr
                   Topic: /zed/zed_node/right_raw_gray/image_raw_gray | Type: sensor_msgs/msg/Image | Count: 6 | Serialization Format: cdr
                   Topic: /zed/zed_node/right_raw_gray/image_raw_gray/compressed | Type: sensor_msgs/msg/CompressedImage | Count: 6 | Serialization Format: cdr
                   Topic: /zed/zed_node/right_raw_gray/image_raw_gray/theora | Type: theora_image_transport/msg/Packet | Count: 9 | Serialization Format: cdr
                   Topic: /zed/zed_node/stereo/image_rect_color | Type: sensor_msgs/msg/Image | Count: 6 | Serialization Format: cdr
                   Topic: /zed/zed_node/stereo/image_rect_color/compressed | Type: sensor_msgs/msg/CompressedImage | Count: 6 | Serialization Format: cdr
                   Topic: /zed/zed_node/stereo/image_rect_color/theora | Type: theora_image_transport/msg/Packet | Count: 9 | Serialization Format: cdr
                   Topic: /zed/zed_node/stereo_raw/image_raw_color | Type: sensor_msgs/msg/Image | Count: 5 | Serialization Format: cdr
                   Topic: /zed/zed_node/stereo_raw/image_raw_color/compressed | Type: sensor_msgs/msg/CompressedImage | Count: 5 | Serialization Format: cdr
                   Topic: /zed/zed_node/stereo_raw/image_raw_color/theora | Type: theora_image_transport/msg/Packet | Count: 8 | Serialization Format: cdr
                   Topic: /zed/zed_node/temperature/imu | Type: sensor_msgs/msg/Temperature | Count: 4 | Serialization Format: cdr
                   Topic: /zed/zed_node/temperature/left | Type: sensor_msgs/msg/Temperature | Count: 4 | Serialization Format: cdr
                   Topic: /zed/zed_node/temperature/right | Type: sensor_msgs/msg/Temperature | Count: 4 | Serialization Format: cdr
```

# Echo ZED ROS2 topics
Depth topic echo (32 bit): 
```
$ ros2 topic echo /zed/zed_node/depth/depth_registered
header:
  stamp:
    sec: 1735262712
    nanosec: 658582223
  frame_id: zed_left_camera_optical_frame
height: 720
width: 1280
encoding: 32FC1
is_bigendian: 0
step: 5120
data:
- 255
- 255
- 255
- 127
```

Depth topic echo (16 bit):
```
$ ros2 topic echo /zed/zed_node/depth/depth_registered
header:
  stamp:
    sec: 1737397460
    nanosec: 165392093
  frame_id: zed_left_camera_optical_frame
height: 1080
width: 1920
encoding: mono16
is_bigendian: 0
step: 3840
data:
- 0
- 0
- 0
- 0
- 134
- 1
- 134
- 1
- 134
- 1
- 134
- 1
- 134
```

In ROS2 for zed cameras, 32 bit float in meters and 16 bit unsigned int in millimeters are used and can be launched. You have to change the parameters inside `common.yaml` file of `params` folder. Specially, `openni_depth_mode` should be `false` for `32bit float` meter units (32FC1) and should be set to `true` for 16bit uchar (mono16) millimeter units. 

Depth camera info topic echo:
```
$ ros2 topic echo /zed/zed_node/depth/camera_info
header:
  stamp:
    sec: 1735262988
    nanosec: 641024223
  frame_id: zed_left_camera_optical_frame
height: 720
width: 1280
distortion_model: rational_polynomial
d:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
k:
- 522.1017456054688
- 0.0
- 632.29638671875
- 0.0
- 522.1017456054688
- 362.0524597167969
- 0.0
- 0.0
- 1.0
r:
- 1.0
- 0.0
- 0.0
- 0.0
- 1.0
- 0.0
- 0.0
- 0.0
- 1.0
p:
- 522.1017456054688
- 0.0
- 632.29638671875
- 0.0
- 0.0
- 522.1017456054688
- 362.0524597167969
- 0.0
- 0.0
- 0.0
- 1.0
- 0.0
binning_x: 0
binning_y: 0
roi:
  x_offset: 0
  y_offset: 0
  height: 0
  width: 0
  do_rectify: false
```

Depth info topic echo:
```
$ ros2 topic echo /zed/zed_node/depth/depth_info
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: zed_left_camera_optical_frame
min_depth: 0.6266739368438721
max_depth: 7.9091668128967285
```

IMU data info topic echo:
```
$ ros2 topic echo /zed/zed_node/imu/data
header:
  stamp:
    sec: 1735263162
    nanosec: 41508051
  frame_id: zed_imu_link
orientation:
  x: 0.002038976177573204
  y: 0.01958724670112133
  z: 0.0013864522334188223
  w: 0.9998050928115845
orientation_covariance:
- 3.507596324246348e-10
- 1.4403283259251969e-11
- 5.7892582424538244e-12
- 1.4403283259251969e-11
- 4.27705758031462e-09
- 8.226906248468142e-12
- 5.789257160235291e-12
- 8.226905166249608e-12
- 3.421399782481341e-10
angular_velocity:
  x: -0.0012592397357855962
  y: 0.001456692460137099
  z: 0.0003174208851875086
angular_velocity_covariance:
- 6.623836107184411e-10
- 0.0
- 0.0
- 0.0
- 5.743454273726701e-10
- 0.0
- 0.0
- -0.0
- 5.877751533323972e-10
linear_acceleration:
  x: -0.35825222730636597
  y: 0.02272910438477993
  z: 9.77094554901123
linear_acceleration_covariance:
- 0.007800576277077198
- 0.0
- 0.0
- 0.0
- 0.00585215026512742
- 0.0
- 0.0
- -0.0
- 0.008417878299951553
```

Left camera topic echo:
```
$ ros2 topic echo /zed/zed_node/left/image_rect_color
header:
  stamp:
    sec: 1735263384
    nanosec: 29969223
  frame_id: zed_left_camera_optical_frame
height: 720
width: 1280
encoding: bgra8
is_bigendian: 0
step: 5120
data:
- 14
- 60
- 77
- 255
- 16
- 60
- 78
- 255
```

The alpha channel in an image `bgra8` represents transparency or opacity for each pixel. While typical images in formats like JPEG / JPG use three color channels—red, green, and blue (RGB), PNG images with an alpha channel add a fourth channel to indicate how opaque or transparent each pixel is.

R (Red): Intensity of red color in a pixel, G (Green): Intensity of green color in a pixel, B (Blue): Intensity of blue color in a pixel, and A (Alpha): Opacity or transparency level of a pixel. An alpha value of 0 makes a pixel fully transparent, 255 makes it fully opaque, and values in between create partial transparency for blending effects.

When the ZED camera provides `bgra8` encoded images, the alpha channel is typically always fully opaque (255), meaning the images are completely visible without any transparency.

Left camera info topic:
```
$ ros2 topic echo /zed/zed_node/left/camera_info
header:
  stamp:
    sec: 1735264140
    nanosec: 272821223
  frame_id: zed_left_camera_optical_frame
height: 720
width: 1280
distortion_model: rational_polynomial
d:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
k:
- 522.1017456054688
- 0.0
- 632.29638671875
- 0.0
- 522.1017456054688
- 362.0524597167969
- 0.0
- 0.0
- 1.0
r:
- 1.0
- 0.0
- 0.0
- 0.0
- 1.0
- 0.0
- 0.0
- 0.0
- 1.0
p:
- 522.1017456054688
- 0.0
- 632.29638671875
- 0.0
- 0.0
- 522.1017456054688
- 362.0524597167969
- 0.0
- 0.0
- 0.0
- 1.0
- 0.0
binning_x: 0
binning_y: 0
roi:
  x_offset: 0
  y_offset: 0
  height: 0
  width: 0
  do_rectify: false
```

RGB camera topic echo:
```
$ ros2 topic echo /zed/zed_node/rgb/image_rect_color
header:
  stamp:
    sec: 1735264721
    nanosec: 772274223
  frame_id: zed_left_camera_optical_frame
height: 720
width: 1280
encoding: bgra8
is_bigendian: 0
step: 5120
data:
- 10
- 58
- 79
- 255
- 12
- 61
- 82
- 255
```

RGB camera info topic echo:
```
$ ros2 topic echo /zed/zed_node/rgb/camera_info
header:
  stamp:
    sec: 1735264839
    nanosec: 279019223
  frame_id: zed_left_camera_optical_frame
height: 720
width: 1280
distortion_model: rational_polynomial
d:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
k:
- 522.1017456054688
- 0.0
- 632.29638671875
- 0.0
- 522.1017456054688
- 362.0524597167969
- 0.0
- 0.0
- 1.0
r:
- 1.0
- 0.0
- 0.0
- 0.0
- 1.0
- 0.0
- 0.0
- 0.0
- 1.0
p:
- 522.1017456054688
- 0.0
- 632.29638671875
- 0.0
- 0.0
- 522.1017456054688
- 362.0524597167969
- 0.0
- 0.0
- 0.0
- 1.0
- 0.0
binning_x: 0
binning_y: 0
roi:
  x_offset: 0
  y_offset: 0
  height: 0
  width: 0
  do_rectify: false
```

Left camera monocular topic echo:
```
$ ros2 topic echo /zed/zed_node/left_gray/image_rect_gray
header:
  stamp:
    sec: 1737393862
    nanosec: 787813064
  frame_id: zed_left_camera_optical_frame
height: 1080
width: 1920
encoding: mono8
is_bigendian: 0
step: 1920
data:
- 18
- 22
- 20
- 21
```

Right camera topic echo:
```
$ ros2 topic echo /zed/zed_node/right/image_rect_color
header:
  stamp:
    sec: 1735263484
    nanosec: 135618223
  frame_id: zed_right_camera_optical_frame
height: 720
width: 1280
encoding: bgra8
is_bigendian: 0
step: 5120
data:
- 142
- 173
- 174
- 255
- 140
- 171
- 172
- 255
```

Right camera info topic echo:
```
$ ros2 topic echo /zed/zed_node/right/camera_info
header:
  stamp:
    sec: 1735264284
    nanosec: 80861223
  frame_id: zed_right_camera_optical_frame
height: 720
width: 1280
distortion_model: rational_polynomial
d:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
k:
- 522.1017456054688
- 0.0
- 632.29638671875
- 0.0
- 522.1017456054688
- 362.0524597167969
- 0.0
- 0.0
- 1.0
r:
- 1.0
- 0.0
- 0.0
- 0.0
- 1.0
- 0.0
- 0.0
- 0.0
- 1.0
p:
- 522.1017456054688
- 0.0
- 632.29638671875
- -62.63468933105469
- 0.0
- 522.1017456054688
- 362.0524597167969
- 0.0
- 0.0
- 0.0
- 1.0
- 0.0
binning_x: 0
binning_y: 0
roi:
  x_offset: 0
  y_offset: 0
  height: 0
  width: 0
  do_rectify: false
```

PointCloud topic echo:
```
$ ros2 topic echo /zed/zed_node/point_cloud/cloud_registered
header:
  stamp:
    sec: 1735286175
    nanosec: 614932393
  frame_id: zed_left_camera_frame
height: 720
width: 1280
fields:
- name: x
  offset: 0
  datatype: 7
  count: 1
- name: y
  offset: 4
  datatype: 7
  count: 1
- name: z
  offset: 8
  datatype: 7
  count: 1
- name: rgb
  offset: 12
  datatype: 7
  count: 1
is_bigendian: false
point_step: 16
row_step: 20480
data:
- 255
- 255
- 255
- 127
- 255
- 255
- 255
- 127
- 255
- 127
- '...'
is_dense: false
```

Odometry topic echo:
```
$ ros2 topic echo /zed/zed_node/odom
header:
  stamp:
    sec: 1735263581
    nanosec: 407536223
  frame_id: odom
child_frame_id: zed_camera_link
pose:
  pose:
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
  covariance:
  - 1.000000013351432e-10
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 1.000000013351432e-10
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 1.000000013351432e-10
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 1.000000013351432e-10
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 1.000000013351432e-10
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 1.000000013351432e-10
twist:
  twist:
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  covariance:
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
```

TF topic echo:
```
$ ros2 topic echo /tf
transforms:
- header:
    stamp:
      sec: 1735282353
      nanosec: 65259225
    frame_id: odom
  child_frame_id: zed_camera_link
  transform:
    translation:
      x: 0.0
      y: 0.0
      z: 0.0
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
---
transforms:
- header:
    stamp:
      sec: 1735282353
      nanosec: 65259225
    frame_id: map
  child_frame_id: odom
  transform:
    translation:
      x: -6.447698225248155e-06
      y: -9.489409711072039e-07
      z: -5.630725508007361e-07
    rotation:
      x: 0.002790777155081556
      y: -0.005017017650249147
      z: -6.978396780946552e-05
      w: 0.9999835179776543
---
transforms:
- header:
    stamp:
      sec: 1735282353
      nanosec: 103295809
    frame_id: zed_left_camera_frame
  child_frame_id: zed_imu_link
  transform:
    translation:
      x: -0.0020000000949949026
      y: -0.023061001673340797
      z: 0.00021700002253055573
    rotation:
      x: -0.0005459150415845215
      y: 0.0014597486006096005
      z: -0.0019339574500918388
      w: 0.9999969005584717
```
TF static topic echo:
```
$ ros2 topic echo /tf_static
transforms:
- header:
    stamp:
      sec: 1735282772
      nanosec: 835417536
    frame_id: zed_camera_center
  child_frame_id: zed_baro_link
  transform:
    translation:
      x: 0.0
      y: 0.0
      z: 0.0
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
- header:
    stamp:
      sec: 1735282772
      nanosec: 835423212
    frame_id: zed_camera_link
  child_frame_id: zed_camera_center
  transform:
    translation:
      x: 0.0
      y: 0.0
      z: 0.015
    rotation:
      x: 0.0
      y: 0.024997395914712332
      z: 0.0
      w: 0.9996875162757025
- header:
    stamp:
      sec: 1735282772
      nanosec: 835423865
    frame_id: zed_camera_center
  child_frame_id: zed_left_camera_frame
  transform:
    translation:
      x: -0.01
      y: 0.06
      z: 0.0
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
- header:
    stamp:
      sec: 1735282772
      nanosec: 835424467
    frame_id: zed_left_camera_frame
  child_frame_id: zed_left_camera_optical_frame
  transform:
    translation:
      x: 0.0
      y: 0.0
      z: 0.0
    rotation:
      x: 0.5
      y: -0.4999999999999999
      z: 0.5
      w: -0.5000000000000001
- header:
    stamp:
      sec: 1735282772
      nanosec: 835425039
    frame_id: zed_camera_center
  child_frame_id: zed_mag_link
  transform:
    translation:
      x: 0.0
      y: 0.0
      z: 0.0
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
- header:
    stamp:
      sec: 1735282772
      nanosec: 835426086
    frame_id: zed_camera_center
  child_frame_id: zed_right_camera_frame
  transform:
    translation:
      x: -0.01
      y: -0.06
      z: 0.0
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
- header:
    stamp:
      sec: 1735282772
      nanosec: 835426817
    frame_id: zed_right_camera_frame
  child_frame_id: zed_right_camera_optical_frame
  transform:
    translation:
      x: 0.0
      y: 0.0
      z: 0.0
    rotation:
      x: 0.5
      y: -0.4999999999999999
      z: 0.5
      w: -0.5000000000000001
- header:
    stamp:
      sec: 1735282772
      nanosec: 835427290
    frame_id: zed_left_camera_frame
  child_frame_id: zed_temp_left_link
  transform:
    translation:
      x: 0.0
      y: 0.0
      z: 0.0
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
- header:
    stamp:
      sec: 1735282772
      nanosec: 835427703
    frame_id: zed_right_camera_frame
  child_frame_id: zed_temp_right_link
  transform:
    translation:
      x: 0.0
      y: 0.0
      z: 0.0
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
```
Pose topic echo:
```
$ ros2 topic echo /zed/zed_node/pose
header:
  stamp:
    sec: 1735263835
    nanosec: 22263223
  frame_id: map
pose:
  position:
    x: -5.217121462012392e-06
    y: -1.0152986544623155e-06
    z: 5.227932532159141e-07
  orientation:
    x: 0.0022268780334843727
    y: -0.005629480340657605
    z: -5.568355206051486e-05
    w: 0.9999816732643955
```

Pose with covariance topic echo:
```
$ ros2 topic echo /zed/zed_node/pose_with_covariance
header:
  stamp:
    sec: 1735264018
    nanosec: 899214223
  frame_id: map
pose:
  pose:
    position:
      x: -5.217121462012392e-06
      y: -1.0152986544623155e-06
      z: 5.227932532159141e-07
    orientation:
      x: 0.0022268780334843727
      y: -0.005629480340657605
      z: -5.568355206051486e-05
      w: 0.9999816732643955
  covariance:
  - 1.000000013351432e-10
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 1.000000013351432e-10
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 1.000000013351432e-10
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 1.000000013351432e-10
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 1.000000013351432e-10
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 1.000000013351432e-10
```

