 export export GAZEBO_MODEL_PATH=~/go2_ws/install/realsense2_description/share/realsense2_description/meshes/$GAZEBO_MODEL_PATH


references:

https://github.com/anujjain-dev/unitree-go2-ros2.git
https://github.com/IntelRealSense/realsense-ros.git
https://github.com/pal-robotics/realsense_gazebo_plugin.git

sudo apt install ros-${ROS_DISTRO}-rtabmap-ros


ros2 launch rtabmap_launch rtabmap.launch.py \
    rtabmap_args:="--delete_db_on_start" \
    rgb_topic:=/camera/color/image_raw \
    depth_topic:=/camera/aligned_depth_to_color/image_raw \
    camera_info_topic:=/camera/color/camera_info \
    depth_camera_info_topic:=/camera/aligned_depth_to_color/camera_info \
    frame_id:=base_link \
    approx_sync:=true \
    wait_imu_to_init:=false \
    imu_topic:=/imu/data \
    odom_topic:=/odom \
    visual_odometry:=false \
    odom_frame_id:=odom \
    publish_tf:=true \
    use_sim_time:=true


ros2 launch rtabmap_launch rtabmap.launch.py     localization:=true     rtabmap_args:="--Mem/IncrementalMemory false --Mem/InitWMWithAllNodes true"     rgb_topic:=/camera/color/image_raw     depth_topic:=/camera/aligned_depth_to_color/image_raw     camera_info_topic:=/camera/color/camera_info     depth_camera_info_topic:=/camera/aligned_depth_to_color/camera_info     frame_id:=base_link     approx_sync:=true     wait_imu_to_init:=false     imu_topic:=/imu/data     odom_topic:=/odom     visual_odometry:=false     odom_frame_id:=odom     publish_tf:=true     use_sim_time:=true

