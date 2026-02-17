from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, RegisterEventHandler
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import os

def generate_launch_description():

    pkg_path = get_package_share_directory('dexhandv2_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'dexhandv2_cobot_right.xacro')

    # 1. Process Xacro
    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = {'robot_description': robot_description_content}

    # 2. Environment Variables
    # Point to the meshes
    install_dir = os.path.abspath(os.path.join(pkg_path, '..'))
    
    # CRITICAL FIX: Find the ign_ros2_control plugin
    # This assumes you have installed ros-humble-ign-ros2-control
    ign_ros2_control_path = os.path.join(get_package_prefix('ign_ros2_control'), 'lib')

    env_vars = [
        SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=[install_dir]),
        SetEnvironmentVariable(name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH', value=[ign_ros2_control_path])
    ]

    # 3. Nodes
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': True}],
        output='screen'
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'dexhand_right', '-topic', 'robot_description', '-z', '0.5'],
        output='screen'
    )

    # Load Controllers (Delayed to ensure startup)
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    hand_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hand_controller"],
        output="screen",
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Bridge the joint states so Rviz works
            '/model/dexhand_right/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
            # Bridge the clock so ROS time syncs with Gazebo
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
        ],
        output='screen'
    )

    return LaunchDescription(env_vars + [
        # Start Gazebo
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', 'empty.sdf'],
            output='screen'
        ),
        robot_state_pub,
        spawn_robot,
        bridge,
        joint_state_broadcaster,
        hand_controller
    ])
