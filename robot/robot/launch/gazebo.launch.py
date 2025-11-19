from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    pkg_robot = FindPackageShare('robot')
    
    # 1. PROCESAR XACRO
    # Usamos xacro para convertir el archivo a URDF puro
    xacro_file = PathJoinSubstitution([pkg_robot, 'urdf', 'robot.urdf.xacro'])
    robot_description_content = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    
    # Diccionario para pasar al robot_state_publisher
    params = {
        'robot_description': robot_description_content, 
        'use_sim_time': True  # <--- ¡CRUCIAL PARA GAZEBO!
    }

    # 2. ROBOT STATE PUBLISHER
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # 3. LANZAR GAZEBO CLASSIC
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ]),
        # Opcional: Si quieres pausar al inicio para ver que todo carga bien, agrega: '-u'
        # launch_arguments={'pause': 'true'}.items() 
    )

    # 4. SPAWN DEL ROBOT
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description', 
            '-entity', 'mi_robot_tesis',
            '-x', '0', '-y', '0', '-z', '0.1' # Levantar un poco para que no choque con el suelo
        ],
        output='screen'
    )

    # 5. CARGAR CONTROLADORES (SPAWNERS)
    # Nota: NO lanzamos 'ros2_control_node', Gazebo lo hace por nosotros vía plugin.
    
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

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])
