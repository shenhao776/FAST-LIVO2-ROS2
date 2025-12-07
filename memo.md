## run docker
```bash
docker run -it -e ROS_DOMAIN_ID=2 \
 -v /var/run/docker.sock:/var/run/docker.sock \
 -v /tmp/.x11-unix:/tmp/.x11-unix \
 -v ~/shared_files:/root/shared_files \
 -v /dev:/dev --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all \
 -e DISPLAY=unix$DISPLAY -e GDK_SCALE -e GDK_DPI_SCALE \
 --privileged --name "ros2_ws" --hostname "ros2_ws" \
 --net=host my-ros2-image:latest zsh
```
## mid360 fastlivo2 in ROS2
### launch
```bash
ros2 launch map_updater mapping_mid360.launch.py
```
### plag bag
```bash
ros2 bag play demo_full_rescan_nav_bag_2025-09-28_02-42-03/
```
### save map
```zsh
ros2 service call /save_map std_srvs/srv/Trigger
```

## lidar camera calibration docker
``` bash
# origin:https://koide3.github.io/direct_visual_lidar_calibration/programs/
bag_path="/home/hao/shared_files/rosbag/ros2bag/livox_data_20250703_cali"
preprocessed_path="/home/hao/shared_files/rosbag/ros2bag/livox_preprocessed_20250703_cali"
docker run \
  --rm -it \
  --net host \
  --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e DISPLAY=$DISPLAY \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -v $bag_path:/tmp/input_bags \
  -v $preprocessed_path:/tmp/preprocessed \
  koide3/direct_visual_lidar_calibration:humble bash

# pre process
ros2 run direct_visual_lidar_calibration preprocess -av /tmp/input_bags /tmp/preprocessed
# initial pose 
ros2 run direct_visual_lidar_calibration find_matches_superglue.py /tmp/preprocessed/
ros2 run direct_visual_lidar_calibration initial_guess_auto /tmp/preprocessed/
# optimization
ros2 run direct_visual_lidar_calibration calibrate /tmp/preprocessed/
# visualization
ros2 run direct_visual_lidar_calibration viewer /tmp/preprocessed
```

## camera info
```json
{
  "camera": {
    "camera_model": "plumb_bob",
    "distortion_coeffs": [
      -0.055450838059186935,
      0.06841126084327698,
      -3.5516146454028785e-05,
      -0.00010084670066135004,
      -0.02119487151503563
    ],
    "intrinsics": [
      631.1903076171875,
      630.41162109375,
      628.8812255859375,
      371.5096435546875
    ]
  },
  "meta": {
    "bag_names": [
      "livox_data_20250703_cali_0.db3"
    ],
    "camera_info_topic": "/camera/camera/color/camera_info",
    "data_path": "/tmp/input_bags",
    "image_topic": "/camera/camera/color/image_raw",
    "intensity_channel": "intensity",
    "points_topic": "/livox/lidar"
  },
  "results": {
    "T_lidar_camera": [
      0.03842141220053184,
      -0.01910240283287441,
      -0.07586569777611907,
      -0.5004375069928715,
      0.49727401587174835,
      -0.5011884722776524,
      0.5010897823639363
    ],
    "init_T_lidar_camera": [
      -0.01,
      -0.03,
      -0.09,
      -0.4999999999999999,
      0.5,
      -0.5,
      0.5000000000000001
    ]
  }
}
```

## note
pcl_w_wait_pub  \
pcl_wait_save_intensity \
原始的激光雷达数据：feats_undistort \
降采样后的激光雷达数据：feats_down_body 




