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
    depth_camera_info_topic:=/camera/depth/camera_info \
    frame_id:=base_link \
    approx_sync:=true \
    wait_imu_to_init:=false \
    imu_topic:=/imu/data \
    odom_topic:=/odom \
    visual_odometry:=true \
    odom_frame_id:=odom \
    publish_tf:=true \
    use_sim_time:=true \
    Rtabmap/DetectionRate:=1.0 \
    Mem/IncrementalMemory:=true \
    Mem/InitWMWithAllNodes:=false


ros2 launch rtabmap_launch rtabmap.launch.py     localization:=true     rtabmap_args:="--Mem/IncrementalMemory false --Mem/InitWMWithAllNodes true"     rgb_topic:=/camera/color/image_raw     depth_topic:=/camera/aligned_depth_to_color/image_raw     camera_info_topic:=/camera/color/camera_info     depth_camera_info_topic:=/camera/aligned_depth_to_color/camera_info     frame_id:=base_link     approx_sync:=true     wait_imu_to_init:=false     imu_topic:=/imu/data     odom_topic:=/odom     visual_odometry:=false     odom_frame_id:=odom     publish_tf:=true     use_sim_time:=true

rtabmap-databaseViewer ~/.ros/rtabmap.db


ros2 launch rtabmap_ros rtabmap.launch.py \
 args:="--delete_db_on_start" \
 depth_topic:=/camera/aligned_depth_to_color/image_raw \
 rgb_topic:=/camera/color/image_raw \
 camera_info_topic:=/camera/color/camera_info \
 approx_sync:=false \
 frame_id:=camera_link \
 use_sim_time:=true







ros2 launch go2_config rtabmap.launch.py \
  rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/color/camera_info \
  odom_topic:=/odom \
  frame_id:=base_link \
  rviz:=false \
  visual_odometry:=false \
  approx_sync:=true \
  use_sim_time:=true \
  imu_topic:=/imu/data \
  args:='--delete_db_on_start' \
  approx_sync_max_interval:=0.1 \
  topic_queue_size:=30 \
  sync_queue_size:=30



  ros2 launch go2_config rtabmap.launch.py   rgb_topic:=/camera/color/image_raw   depth_topic:=/camera/aligned_depth_to_color/image_raw   camera_info_topic:=/camera/color/camera_info   frame_id:=base_link   rviz:=false   visual_odometry:=true   icp_odometry:=false   approx_sync:=true   use_sim_time:=true   imu_topic:=/imu/data   args:='--delete_db_on_start'   approx_sync_max_interval:=0.1   topic_queue_size:=30   sync_queue_size:=30


ros2 launch go2_config rtabmap.launch.py   rgb_topic:=/camera/color/image_raw   depth_topic:=/camera/aligned_depth_to_color/image_raw   camera_info_topic:=/camera/color/camera_info   frame_id:=base_link   odom_frame_id:=odom   publish_tf:=true   rviz:=false   visual_odometry:=true   icp_odometry:=false   approx_sync:=true   use_sim_time:=true   wait_for_transform:=0.2   rtabmap_args:='--delete_db_on_start'   approx_sync_max_interval:=0.1   queue_size:=30



 ros2 launch go2_config rtabmap.launch.py   rgb_topic:=/camera/color/image_raw   depth_topic:=/camera/aligned_depth_to_color/image_raw   camera_info_topic:=/camera/color/camera_info   frame_id:=base_link   odom_frame_id:=odom   publish_tf_odom:=false   publish_tf_map:=true   rviz:=false   visual_odometry:=true   icp_odometry:=false   odom_topic:=/odom   vo_frame_id:=odom_visual   approx_sync:=true   use_sim_time:=true   wait_for_transform:=0.2   rtabmap_args:='--delete_db_on_start'   approx_sync_max_interval:=0.1   queue_size:=30   imu_topic:=/imu/data


 ros2 launch go2_config rtabmap.launch.py   rgb_topic:=/camera/color/image_raw   depth_topic:=/camera/aligned_depth_to_color/image_raw   camera_info_topic:=/camera/color/camera_info   frame_id:=base_link   odom_frame_id:=odom   publish_tf_odom:=false   publish_tf_map:=true   rviz:=false   visual_odometry:=true   icp_odometry:=false   odom_topic:=/odom   vo_frame_id:=odom_visual   approx_sync:=true   use_sim_time:=true   wait_for_transform:=0.2   localization:=true   approx_sync_max_interval:=0.1   queue_size:=30   imu_topic:=/imu/data
