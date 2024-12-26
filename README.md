# All-About-ZED
This repository is all about zed and it's implementation in my PhD.

# ROS1 topic from ZED2
```
arghya@arghya-Pulse-GL66-12UEK:~/zed_ws$ rostopic list
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

# ROS2 topic from ZED2
```
arghya@arghya-Pulse-GL66-12UEK:~/zed_ws$ rostopic list
/clock
/tf
/tf_static
/zed2/zed_node/atm_press
/zed2/zed_node/confidence/confidence_map
/zed2/zed_node/depth/camera_info
/zed2/zed_node/depth/depth_registered
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
/zed2/zed_node/left/image_rect_color/compressedDepth/parameter_descriptions
/zed2/zed_node/left/image_rect_color/compressedDepth/parameter_updates
/zed2/zed_node/left/image_rect_color/theora
/zed2/zed_node/left/image_rect_color/theora/parameter_descriptions
/zed2/zed_node/left/image_rect_color/theora/parameter_updates
/zed2/zed_node/left/image_rect_gray
/zed2/zed_node/left/image_rect_gray/compressed
/zed2/zed_node/left/image_rect_gray/compressed/parameter_descriptions
/zed2/zed_node/left/image_rect_gray/compressed/parameter_updates
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
/zed2/zed_node/left_raw/image_raw_color/compressedDepth/parameter_descriptions
/zed2/zed_node/left_raw/image_raw_color/compressedDepth/parameter_updates
/zed2/zed_node/left_raw/image_raw_color/theora
/zed2/zed_node/left_raw/image_raw_color/theora/parameter_descriptions
/zed2/zed_node/left_raw/image_raw_color/theora/parameter_updates
/zed2/zed_node/left_raw/image_raw_gray
/zed2/zed_node/left_raw/image_raw_gray/compressed
/zed2/zed_node/left_raw/image_raw_gray/compressed/parameter_descriptions
/zed2/zed_node/left_raw/image_raw_gray/compressed/parameter_updates
/zed2/zed_node/left_raw/image_raw_gray/compressedDepth/parameter_descriptions
/zed2/zed_node/left_raw/image_raw_gray/compressedDepth/parameter_updates
/zed2/zed_node/left_raw/image_raw_gray/theora
/zed2/zed_node/left_raw/image_raw_gray/theora/parameter_descriptions
/zed2/zed_node/left_raw/image_raw_gray/theora/parameter_updates
/zed2/zed_node/odom
/zed2/zed_node/parameter_descriptions
/zed2/zed_node/parameter_updates
/zed2/zed_node/path_map
/zed2/zed_node/path_odom
/zed2/zed_node/point_cloud/cloud_registered
/zed2/zed_node/pose
/zed2/zed_node/pose_with_covariance
/zed2/zed_node/rgb/camera_info
/zed2/zed_node/rgb/image_rect_color
/zed2/zed_node/rgb/image_rect_color/compressed
/zed2/zed_node/rgb/image_rect_color/compressed/parameter_descriptions
/zed2/zed_node/rgb/image_rect_color/compressed/parameter_updates
/zed2/zed_node/rgb/image_rect_color/compressedDepth/parameter_descriptions
/zed2/zed_node/rgb/image_rect_color/compressedDepth/parameter_updates
/zed2/zed_node/rgb/image_rect_color/theora
/zed2/zed_node/rgb/image_rect_color/theora/parameter_descriptions
/zed2/zed_node/rgb/image_rect_color/theora/parameter_updates
/zed2/zed_node/rgb/image_rect_gray
/zed2/zed_node/rgb/image_rect_gray/compressed
/zed2/zed_node/rgb/image_rect_gray/compressed/parameter_descriptions
/zed2/zed_node/rgb/image_rect_gray/compressed/parameter_updates
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
/zed2/zed_node/rgb_raw/image_raw_color/compressedDepth/parameter_descriptions
/zed2/zed_node/rgb_raw/image_raw_color/compressedDepth/parameter_updates
/zed2/zed_node/rgb_raw/image_raw_color/theora
/zed2/zed_node/rgb_raw/image_raw_color/theora/parameter_descriptions
/zed2/zed_node/rgb_raw/image_raw_color/theora/parameter_updates
/zed2/zed_node/rgb_raw/image_raw_gray
/zed2/zed_node/rgb_raw/image_raw_gray/compressed
/zed2/zed_node/rgb_raw/image_raw_gray/compressed/parameter_descriptions
/zed2/zed_node/rgb_raw/image_raw_gray/compressed/parameter_updates
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
/zed2/zed_node/right/image_rect_color/compressedDepth/parameter_descriptions
/zed2/zed_node/right/image_rect_color/compressedDepth/parameter_updates
/zed2/zed_node/right/image_rect_color/theora
/zed2/zed_node/right/image_rect_color/theora/parameter_descriptions
/zed2/zed_node/right/image_rect_color/theora/parameter_updates
/zed2/zed_node/right/image_rect_gray
/zed2/zed_node/right/image_rect_gray/compressed
/zed2/zed_node/right/image_rect_gray/compressed/parameter_descriptions
/zed2/zed_node/right/image_rect_gray/compressed/parameter_updates
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
/zed2/zed_node/right_raw/image_raw_color/compressedDepth/parameter_descriptions
/zed2/zed_node/right_raw/image_raw_color/compressedDepth/parameter_updates
/zed2/zed_node/right_raw/image_raw_color/theora
/zed2/zed_node/right_raw/image_raw_color/theora/parameter_descriptions
/zed2/zed_node/right_raw/image_raw_color/theora/parameter_updates
/zed2/zed_node/right_raw/image_raw_gray
/zed2/zed_node/right_raw/image_raw_gray/compressed
/zed2/zed_node/right_raw/image_raw_gray/compressed/parameter_descriptions
/zed2/zed_node/right_raw/image_raw_gray/compressed/parameter_updates
/zed2/zed_node/right_raw/image_raw_gray/compressedDepth/parameter_descriptions
/zed2/zed_node/right_raw/image_raw_gray/compressedDepth/parameter_updates
/zed2/zed_node/right_raw/image_raw_gray/theora
/zed2/zed_node/right_raw/image_raw_gray/theora/parameter_descriptions
/zed2/zed_node/right_raw/image_raw_gray/theora/parameter_updates
/zed2/zed_node/stereo/image_rect_color
/zed2/zed_node/stereo/image_rect_color/compressed
/zed2/zed_node/stereo/image_rect_color/compressed/parameter_descriptions
/zed2/zed_node/stereo/image_rect_color/compressed/parameter_updates
/zed2/zed_node/stereo/image_rect_color/compressedDepth/parameter_descriptions
/zed2/zed_node/stereo/image_rect_color/compressedDepth/parameter_updates
/zed2/zed_node/stereo/image_rect_color/theora
/zed2/zed_node/stereo/image_rect_color/theora/parameter_descriptions
/zed2/zed_node/stereo/image_rect_color/theora/parameter_updates
/zed2/zed_node/stereo_raw/image_raw_color
/zed2/zed_node/stereo_raw/image_raw_color/compressed
/zed2/zed_node/stereo_raw/image_raw_color/compressed/parameter_descriptions
/zed2/zed_node/stereo_raw/image_raw_color/compressed/parameter_updates
/zed2/zed_node/stereo_raw/image_raw_color/compressedDepth/parameter_descriptions
/zed2/zed_node/stereo_raw/image_raw_color/compressedDepth/parameter_updates
/zed2/zed_node/stereo_raw/image_raw_color/theora
/zed2/zed_node/stereo_raw/image_raw_color/theora/parameter_descriptions
/zed2/zed_node/stereo_raw/image_raw_color/theora/parameter_updates
/zed2/zed_node/temperature/imu
/zed2/zed_node/temperature/left
/zed2/zed_node/temperature/right
```
# Setup ROS1 workspace

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
