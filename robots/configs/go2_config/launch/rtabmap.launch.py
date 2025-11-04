from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    parameters = [{
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_scan_cloud': True,
        'use_sim_time': True,
        'approx_sync': True,
        'qos': 2,
        'wait_for_transform': 0.5,
        
        # RTAB-Map parameters
        'Reg/Strategy': '1',  # 0=Visual, 1=ICP, 2=Visual+ICP
        'Reg/Force3DoF': 'false',
        'RGBD/ProximityMaxGraphDepth': '0',
        'RGBD/ProximityPathMaxNeighbors': '1',
        'RGBD/AngularUpdate': '0.05',
        'RGBD/LinearUpdate': '0.05',
        'Mem/NotLinkedNodesKept': 'false',
        'Mem/STMSize': '30',
        'Mem/LaserScanNormalK': '20',
        
        # ICP parameters for loop closure
        'Icp/VoxelSize': '0.05',
        'Icp/MaxCorrespondenceDistance': '0.1',
        'Icp/PointToPlane': 'true',
        'Icp/PointToPlaneK': '20',
        'Icp/PointToPlaneRadius': '0.0',
        'Icp/MaxTranslation': '3',
        'Icp/Epsilon': '0.001',
        'Icp/CorrespondenceRatio': '0.2',
        
        # Grid parameters
        'Grid/FromDepth': 'false',
        'Grid/RangeMax': '20',
        'Grid/RayTracing': 'true',
        'Grid/CellSize': '0.05',
        'Grid/3D': 'true',
        'Grid/MaxObstacleHeight': '2.0',
        
        # Optimization
        'RGBD/OptimizeFromGraphEnd': 'false',
        'RGBD/OptimizeMaxError': '3',
    }]
    
    remappings = [
        ('rgb/image', '/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/color/camera_info'),
        ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
        ('scan_cloud', '/mid360_PointCloud2'),
        ('odom', '/odom'),
    ]

    return LaunchDescription([
        
        # Set RTAB-Map database path
        DeclareLaunchArgument(
            'database_path', 
            default_value='~/.ros/rtabmap.db',
            description='Path to RTAB-Map database'
        ),
        
        DeclareLaunchArgument(
            'localization', 
            default_value='false',
            description='Set to true for localization mode'
        ),
        
        DeclareLaunchArgument(
            'rtabmap_args',
            default_value='--delete_db_on_start',
            description='Arguments for rtabmap node'
        ),
        
        # RTAB-Map SLAM Node (using existing odometry from /odom)
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[parameters[0], {
                'database_path': LaunchConfiguration('database_path'),
                'localization': LaunchConfiguration('localization'),
            }],
            remappings=remappings,
            arguments=[LaunchConfiguration('rtabmap_args')],
        ),
        
        # RTAB-Map Visualization Node
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[parameters[0]],
            remappings=remappings,
        ),
    ])