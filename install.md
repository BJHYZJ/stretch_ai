setuptools==59.8.0


cd stretch_ws/src/stretch_mujoco/third_party/robocasa/
pip install -e .
cd ../robosuit
pip install -e .

python robocasa/scripts/download_kitchen_assets.py   # Caution: Assets to be downloaded are around 5GB.
python robocasa/scripts/setup_macros.py              # Set up system variables.

cd ../../
pip install -e .  # install stretch_mujoco



https://github.com/hello-robot/stretch_tutorials/blob/2ccee6617dc8bb7bfa5b2c8438185dec5f99e053/ros2/remote_compute.md



```bash
# mkdir -p ~/ament_ws/src
# cd ~/ament_ws/src/
git clone https://github.com/hello-robot/stretch_ros2
git clone https://github.com/hello-binit/ros2_numpy -b humble
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-master  # default ros2-development
# git clone https://github.com/Slamtec/sllidar_ros2.git -b main
# git clone https://github.com/hello-binit/respeaker_ros2.git -b humble
# git clone https://github.com/hello-binit/audio_common.git -b humble
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
git clone https://github.com/Ericsii/FAST_LIO_ROS2.git --recursive
git clone https://github.com/westonrobot/ugv_sdk.git
git clone https://github.com/westonrobot/ranger_ros2.git
git clone https://github.com/xArm-Developer/xarm_ros2.git --recursive -b humbleccc

git clone https://github.com/introlab/rtabmap.git
git clone --branch ros2 https://github.com/introlab/rtabmap_ros.git

```


```bash
cd livox_ros_driver2
cp package_ROS2.xml package.xml
cp -rf launch_ROS2/ launch/

cd ../..
source /opt/ros/humble/setup.bash
rosdep install --rosdistro=humble -iyr --skip-keys="librealsense2" --from-paths src

colcon build --packages-select livox_ros_driver2 fast_lio --symlink-install --cmake-args -DHUMBLE_ROS=humble -DCMAKE_BUILD_TYPE=Release
colcon build --symlink-install --cmake-args -DHUMBLE_ROS=humble -DCMAKE_BUILD_TYPE=Release
colcon build --packages-select yanzj_ros2_bridge --symlink-install --cmake-args -DHUMBLE_ROS=humble -DCMAKE_BUILD_TYPE=Release

# MAKEFLAGS=-j1 colcon build \
#   --executor sequential \
#   --parallel-workers 1 \
#   --symlink-install \
#   --packages-up-to rtabmap_ros \
#   --cmake-args -G "Unix Makefiles" \
#                -DCMAKE_BUILD_PARALLEL_LEVEL=4 \
#                -DBUILD_TESTING=OFF \
#                -DCMAKE_BUILD_TYPE=Release \
#                -DCMAKE_CXX_FLAGS_RELEASE="-g0"

colcon build --packages-select stretch_ros2 --symlink-install --cmake-args -DHUMBLE_ROS=humble -DCMAKE_BUILD_TYPE=Release
colcon build --symlink-install --cmake-args -DHUMBLE_ROS=humble -DCMAKE_BUILD_TYPE=Release

# cd src/livox_ros_driver2
# rm package.xml
# rm -rf launch/
cd ../..
```


```bash
# Terminal 1: Start the Stretch Driver Node
ros2 launch stretch_core stretch_driver.launch.py
# Terminal 2: Start the realsense D435i stream.
ros2 launch stretch_core d435i_high_resolution.launch.py
# Terminal 3: Start lidar.
ros2 launch stretch_core rplidar.launch.py
```



```bash
ros2 daemon stop
ros2 daemon start
# kill node by node name
ps -ef | grep camera | grep -v grep
ps -ef | grep d435i_accel_correction_node
ps -ef | grep spawner_xarm6_traj_controller
ps -ef | grep ros2
pgrep -a -f 'realsense|d435|d435i|accel|component_container|camera|xarm|livox|robot_description|ros2 launch|joint_states|parametr_events'

pkill -9 -f ros2
pkill -9 -f rviz
pkill -9 -f move_group

pkill -f "ros2 launch"

ros2 lifecycle nodes
ros2 lifecycle set /d435i_accel_correction_node shutdown



ps aux | grep ros2
ros2 daemon stop && sleep 2 && ros2 daemon start


ps aux | grep -E "(ufactory|xarm|ranger)" | grep -v grep


pkill -9 static_transform_publisher
pkill -9 -f static

pkill -9 -f ranger_base_node 强制清除node
```




```bash
ros2 service call /save_map nav2_msgs/srv/SaveMap "{map_url: 'orbslam3_map'}"
ros2 service call /load_map nav2_msgs/srv/LoadMap "{map_url: 'orbslam3_map'}"
```


view tf frames
```bash
ros2 run tf2_tools view_frames
# vie params from rtqbmap
rtabmap --params | grep Grid

# 查看frame之间的转换
ros2 run tf2_ros tf2_echo livox_frame base_link
```


start ranger

```bash
sudo apt install -y can-utils
sudo modprobe gs_usb
sudo ip link set can0 up type can bitrate 500000
````


record bags
```bash
ros2 bag record \
/actuator_state \
/battery_state \
/camera/accel/imu_info \
/camera/accel/sample \
/camera/accel/sample_corrected \
/camera/aligned_depth_to_color/camera_info \
/camera/aligned_depth_to_color/image_raw \
/camera/color/camera_info \
/camera/color/image_raw \
/camera/depth/camera_info \
/camera/depth/color/points \
/camera/depth/image_rect_raw \
/camera/extrinsics/depth_to_accel \
/camera/extrinsics/depth_to_color \
/camera/extrinsics/depth_to_depth \
/camera/extrinsics/depth_to_gyro \
/camera/gyro/imu_info \
/camera/gyro/sample \
/clicked_point \
/cmd_vel \
/goal_pose \
/initialpose \
/joint_states \
/livox/imu \
/livox/lidar \
/motion_state \
/odom \
/parameter_events \
/ranger/joint_states \
/rc_state \
/robot_description \
/rosout \
/system_state \
/tf \
/tf_static \
/xarm/joint_states \
/xarm/robot_states \
/xarm/uf_ftsensor_ext_states \
/xarm/uf_ftsensor_raw_states \
/xarm/vc_set_cartesian_velocity \
/xarm/vc_set_joint_velocity \
/xarm/xarm_cgpio_statesc

```

play bag
```bash
ros2 bag play /home/yanzj/stretch_ws/stretch_ws/rosbag2_2025_09_03-19_28_42 \
    --clock \
    --topics \
        /camera/aligned_depth_to_color/image_raw \
        /camera/color/camera_info \
        /camera/color/image_raw \
        /joint_states \
        /livox/imu \
        /livox/lidar \
        /motion_state \
        /ranger/joint_states \
        /xarm/joint_states \
        /xarm/robot_states \
        /system_state \
        /tf \
        /tf_static


```