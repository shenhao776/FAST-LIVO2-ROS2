## Forked from this repo:
[Origin: https://github.com/hku-mars/FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2)

# Deployment Memo

## Basic config for visualization
```bash
# run rviz2 in the remote device via ssh
export DISPLAY=:0
# if you want to run rviz2 in docker
xhost +  
# run vnc server (only if you run the code in the remote device)
/usr/lib/vino/vino-server
```

## Run ros2 docker images
For details, refer to the separate guide. 
* **Link**: [Dockerfile Usage Reference](./docker/dockerfile_usage.md)

## Build ros2 workspace
```zsh
# enter the docker container
docker exec -it contain_id zsh
# ros2 build
cd /root/shared_files/ros2_ws/src
git clone https://github.com/shenhao776/rpg_vikit.git
git clone https://github.com/shenhao776/FAST-LIVO2-ROS2.git
cd ..
colcon build --symlink-install
source install/setup.zsh
```

## Integrated launch
```zsh
source install/setup.zsh
ros2 launch map_updater mapping_mid360.launch.py
ros2 bag play demo_full_rescan_nav_bag_2025-09-28_02-42-03/
# save map
ros2 service call /save_map std_srvs/srv/Trigger
```




