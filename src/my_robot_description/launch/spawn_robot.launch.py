import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    urdf_path = os.path.join(
        get_package_share_directory('my_robot_description'),
        'urdf', 'my_robot.urdf'
    )

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    world_path = os.path.expanduser(
        '~/ros_ws/src/my_robot/worlds/my_world.sdf'
    )

    return LaunchDescription([

        # 1. Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # 2. Launch Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_path],
            output='screen'
        ),

        # 3. Spawn robot — delayed 3s to let Gazebo start
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=[
                        '-name', 'my_robot',
                        '-topic', 'robot_description',
                        '-x', '0', '-y', '0', '-z', '0.05'
                    ],
                    output='screen'
                ),
            ]
        ),

        # 4. Bridge — delayed 4s to let robot spawn first
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='ros_gz_bridge',
                    arguments=[
                        # clock
                        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                        # cmd_vel: ROS2 → Gazebo
                        '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                        # scan: Gazebo → ROS2
                        '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                        # odom: Gazebo → ROS2
                        '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                        # TF from Gazebo model
                        '/model/my_robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                        #camera: Gazebo → ROS2
                        '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                    ],
                    parameters=[{'use_sim_time': True}],
                    output='screen'
                ),
                Node(
                    package='topic_tools',
                    executable='relay',
                    name='tf_relay',
                    arguments=['/model/my_robot/tf', '/tf'],
                    parameters=[{'use_sim_time': True}]
                )
            ]
        ),

    ])