from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_robot = FindPackageShare('robot').find('robot')
    urdf_path = PathJoinSubstitution([pkg_robot, 'urdf', 'robot.urdf'])
    world_path = PathJoinSubstitution([pkg_robot, 'worlds', 'empty.world'])

    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    # Lanzar Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={'world': world_path}.items()
    )

    # Publicar el robot_description y TFs
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Spawnear el robot en Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'robot',
                   '-x', '0.0', '-y', '0.0', '-z', '0.0'],
        output='screen'
    )

    # Spawnear controladores (ya con el controller_manager de Gazebo)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    robot_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['robot_arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    delay_robot_arm_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_arm_controller_spawner],
        )
    )

    return LaunchDescription([
        gazebo,
        robot_state_pub,
        spawn_entity,
        TimerAction(period=5.0, actions=[joint_state_broadcaster_spawner]),
        delay_robot_arm_controller_spawner
    ])

