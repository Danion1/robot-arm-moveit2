import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('braco2_description')
    # Os dois podem ser trocados por argumento (o pick and place usa o mundo com o
    # objeto e a descrição com a pega); os padrões são os da etapa do Gazebo.
    urdf_path = PathJoinSubstitution([pkg_share, 'urdf', LaunchConfiguration('descricao')])
    world_path = LaunchConfiguration('mundo')
    gz_sim_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')

    gui = LaunchConfiguration('gui')

    # O xacro resolve o $(find braco2_description) que está dentro da
    # tag <gazebo> do URDF (caminho do controladores.yaml).
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    # O Gazebo procura "package://braco2_description/meshes/..." dentro
    # das pastas listadas nesta variável — precisa da pasta que CONTÉM o
    # pacote, por isso o '..'.
    set_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(pkg_share, '..'))

    # -r = já começa rodando (sem isso a simulação abre pausada).
    # -s = só servidor, sem janela.
    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={'gz_args': ['-r ', world_path]}.items(),
        condition=IfCondition(gui),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={'gz_args': ['-r -s ', world_path]}.items(),
        condition=UnlessCondition(gui),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'braco2', '-allow_renaming', 'true'],
    )

    # Formato de cada ponte: tópico@tipo_ROS[tipo_Gazebo. O "[" quer dizer
    # "só do Gazebo pro ROS".
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster'],
    )

    braco_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['braco_controller'],
    )

    garra_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['garra_controller'],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mundo',
            default_value='empty.sdf',
            description='Mundo do Gazebo (nome conhecido do Gazebo ou caminho completo)'
        ),
        DeclareLaunchArgument(
            'descricao',
            default_value='braco_gazebo.urdf.xacro',
            description='Arquivo da pasta urdf/ com a descrição do braço'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Abre a janela 3D do Gazebo se true'
        ),
        set_resource_path,
        gazebo_gui,
        gazebo_headless,
        clock_bridge,
        robot_state_publisher,
        spawn_robot,
        # Os controladores só podem subir depois que o robô existe no
        # Gazebo (é o plugin dele que cria o controller_manager), e um de
        # cada vez.
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[braco_controller_spawner],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=braco_controller_spawner,
            on_exit=[garra_controller_spawner],
        )),
    ])
