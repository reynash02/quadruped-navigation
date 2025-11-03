from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('database_path', default_value='~/.ros/rtabmap.db'),
        
        # RTABMap mapping node
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'publish_tf': True,
                'approx_sync': True,
                'wait_for_transform': 0.2,
                
                # Visual odometry parameters
                'Vis/MinInliers': 15,
                'Vis/InlierDistance': 0.1,
                'Odom/ResetCountdown': 0,
                'Odom/Strategy': 0,  # 0=Frame-to-Map, 1=Frame-to-Frame
            }],
            remappings=[
                ('rgb/image', '/camera/color/image_raw'),
                ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
            ],
            arguments=['--ros-args', '--log-level', 'info']
        ),
        
        # RTABMap SLAM node
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'database_path': LaunchConfiguration('database_path'),
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'map_frame_id': 'map',
                'publish_tf': True,
                'approx_sync': True,
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'subscribe_odom_info': True,
                
                # Delete database on start for fresh mapping
                'delete_db_on_start': True,
                
                # Core RTAB-Map parameters for better loop closure detection
                'RGBD/ProximityBySpace': True,
                'RGBD/ProximityMaxGraphDepth': 0,
                'RGBD/ProximityPathMaxNeighbors': 10,
                'RGBD/AngularUpdate': 0.05,  # rad - add new node if rotation > this
                'RGBD/LinearUpdate': 0.05,   # m - add new node if translation > this
                'RGBD/OptimizeFromGraphEnd': False,
                
                # Memory management
                'Mem/IncrementalMemory': True,  # True for mapping
                'Mem/InitWMWithAllNodes': False,
                'Mem/STMSize': 30,
                'Mem/NotLinkedNodesKept': True,  # Keep for potential future loop closures
                'Mem/IntermediateNodeDataKept': True,
                
                # Loop closure detection parameters
                'Rtabmap/DetectionRate': 1.0,  # Hz - check for loop closures
                'Rtabmap/TimeThr': 0,  # No time threshold
                'Kp/DetectorStrategy': 6,  # 0=SURF, 6=ORB, 8=FAST
                'Kp/MaxFeatures': 600,
                'Kp/MinInliers': 15,  # Minimum features to accept loop closure
                'Kp/RoiRatios': '0.0 0.0 0.0 0.0',  # Use full image
                
                # Loop closure constraints
                'Reg/Strategy': 1,  # 0=Visual, 1=ICP, 2=Visual+ICP
                'Reg/Force3DoF': True,  # True for ground robots (2D plane)
                'RGBD/LoopClosureReextractFeatures': True,
                
                # Visual registration
                'Vis/MaxFeatures': 1000,
                'Vis/MinInliers': 30,
                'Vis/InlierDistance': 0.1,
                'Vis/CorType': 0,  # 0=Features Matching, 1=Optical Flow
                
                # ICP parameters (for refinement)
                'Icp/MaxTranslation': 0.2,
                'Icp/VoxelSize': 0.05,
                'Icp/MaxCorrespondenceDistance': 0.1,
                'Icp/PointToPlane': True,
                'Icp/Iterations': 30,
                
                # Graph optimization
                'RGBD/OptimizeMaxError': 1.0,
                'Optimizer/Strategy': 1,  # 0=TORO, 1=g2o, 2=GTSAM
                'Optimizer/Iterations': 100,
                'Optimizer/Slam2D': True,  # True for ground robots
                
                # Logging and statistics - IMPORTANT for monitoring loop closures
                'Rtabmap/PublishStats': True,
                'Rtabmap/PublishLastSignature': True,
                'Rtabmap/PublishLikelihood': True,
                'Rtabmap/PublishRAMUsage': True,
            }],
            remappings=[
                ('rgb/image', '/camera/color/image_raw'),
                ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('odom', '/odom'),
            ],
            arguments=['--ros-args', '--log-level', 'info']
        ),
        
        # RTABMap visualization
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'subscribe_odom_info': True,
                'approx_sync': True,
            }],
            remappings=[
                ('rgb/image', '/camera/color/image_raw'),
                ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('odom', '/odom'),
            ]
        ),
        
        # Node to monitor loop closures
        Node(
            package='rtabmap_util',
            executable='point_cloud_assembler',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            remappings=[
                ('cloud', '/rtabmap/cloud_map'),
            ]
        ),
        
        LogInfo(msg='RTABMap Mapping Started'),
        LogInfo(msg='Monitor loop closures with: ros2 topic echo /rtabmap/info'),
        LogInfo(msg='View database after mapping: rtabmap-databaseViewer ~/.ros/rtabmap.db'),
    ])