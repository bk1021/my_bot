from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('my_bot')

    xacro_file = os.path.join(share_dir, 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    rviz_config_file = os.path.join(share_dir, 'config', 'gazebo.rviz')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_urdf,
                'use_sim_time': True
            }
        ]
    )

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', os.path.join(share_dir, 'worlds', 'shapes.sdf')],
        output='screen'
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-string', robot_urdf, '-name', 'my_robot', '-z', '0.05'],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            # '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            # '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/depth_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/depth_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/depth_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        output='screen',
    )

    # odom_to_tf = Node(
    #     package='my_bot',
    #     executable='odom_to_tf',
    #     name='odom_to_tf',
    #     output='screen',
    #     parameters=[{'use_sim_time': True}]
    # )

    cmd_vel_republisher = Node(
        package='my_bot',
        executable='cmd_vel_republisher',
        name='cmd_vel_republisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    joint_broad = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad', '--switch-timeout', '10'],
        output='screen'
    )
    
    diff_cont = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_cont', '--switch-timeout', '10', 
            '--controller-ros-args',
            "--ros-args --remap /diff_cont/cmd_vel:=/cmd_vel_stamped --remap /diff_cont/odom:=/odom"
        ],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gazebo,
        spawn_entity,
        bridge,
        # odom_to_tf,
        joint_broad,
        diff_cont,
        cmd_vel_republisher,
        rviz_node,
    ])