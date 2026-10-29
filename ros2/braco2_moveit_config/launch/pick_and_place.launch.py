"""
Pick and place no Gazebo: o MoveIt2 planeja e o Gazebo executa (hardware
simulado no lugar do falso da demo).

Sobe:
  - o gazebo.launch.py do braco2_description com o mundo do objeto
    (worlds/pick_and_place.sdf) e a descrição com a pega (urdf/braco_pega.urdf.xacro);
    ele já sobe robot_state_publisher, ponte do /clock e os 3 controladores;
  - as pontes da pega (ROS 2 <-> Gazebo): /braco2/prender e /braco2/soltar
    (std_msgs/msg/Empty, ROS -> Gazebo), /braco2/estado_pega (String) e
    /model/objeto/pose (Pose, só para a conferência), Gazebo -> ROS;
  - o move_group com use_sim_time (o relógio é o do Gazebo);
  - o RViz2 com o painel MotionPlanning (use_rviz:=false para não abrir).

ATENÇÃO: o DetachableJoint começa PRESO (o objeto acompanha a mão desde o início).
A primeira coisa a fazer é publicar em /braco2/soltar:
    ros2 topic pub --once /braco2/soltar std_msgs/msg/Empty

Os controladores têm os mesmos nomes da demo (braco_controller,
garra_controller): o moveit_controllers.yaml não muda.

Vídeo (video:=true): monta na hora uma cópia do mundo com uma CÂMERA simulada e o
sistema de sensores, e faz a ponte da imagem para /camera_video. O Gazebo renderiza
cada quadro no tempo da simulação e espera o quadro ficar pronto, então o vídeo sai
liso mesmo com o WSL desenhando sem placa de vídeo (a simulação fica mais lenta que o
relógio, não o vídeo). Quem grava o arquivo é o ros2/gravar_video.py.

Uso:
    ros2 launch braco2_moveit_config pick_and_place.launch.py
    ros2 launch braco2_moveit_config pick_and_place.launch.py gui:=false use_rviz:=false
    ros2 launch braco2_moveit_config pick_and_place.launch.py gui:=false use_rviz:=false video:=true
"""

import math
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


# Câmera do vídeo: de onde olha e para onde (m). Mira entre a base e o objeto.
CAMERA_EM = (0.55, 0.42, 0.33)
CAMERA_ALVO = (0.13, 0.0, 0.05)


def camera_sdf():
    dx, dy, dz = (a - b for a, b in zip(CAMERA_ALVO, CAMERA_EM))
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(-dz, math.hypot(dx, dy))     # positivo = olhar para baixo
    x, y, z = CAMERA_EM
    return f"""
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <model name="camera_video">
      <static>true</static>
      <pose>{x} {y} {z} 0 {pitch:.5f} {yaw:.5f}</pose>
      <link name="link">
        <sensor name="camera" type="camera">
          <always_on>1</always_on>
          <update_rate>30</update_rate>
          <topic>/camera_video</topic>
          <camera>
            <horizontal_fov>0.85</horizontal_fov>
            <image><width>1280</width><height>720</height></image>
            <clip><near>0.02</near><far>20</far></clip>
          </camera>
        </sensor>
      </link>
    </model>
"""


def gazebo_com_mundo(context, desc_share):
    mundo = os.path.join(desc_share, 'worlds', 'pick_and_place.sdf')
    if LaunchConfiguration('video').perform(context) == 'true':
        with open(mundo, encoding='utf-8') as f:
            texto = f.read().replace('</world>', camera_sdf() + '  </world>')
        mundo = os.path.join(tempfile.mkdtemp(prefix='mundo_video_'), 'pick_and_place_video.sdf')
        with open(mundo, 'w', encoding='utf-8') as f:
            f.write(texto)
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(desc_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'mundo': mundo,
            'descricao': 'braco_pega.urdf.xacro',
            'gui': LaunchConfiguration('gui'),
        }.items(),
    )]


def generate_launch_description():
    desc_share = get_package_share_directory('braco2_description')
    moveit_share = get_package_share_directory('braco2_moveit_config')
    moveit_config = MoveItConfigsBuilder("braco2", package_name="braco2_moveit_config").to_moveit_configs()
    sim = {'use_sim_time': True}

    gazebo = OpaqueFunction(function=gazebo_com_mundo, args=[desc_share])

    # Formato: tópico@tipo_ROS]tipo_Gazebo ("]" = do ROS para o Gazebo; "[" = do Gazebo para o ROS)
    pontes_pega = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/braco2/prender@std_msgs/msg/Empty]gz.msgs.Empty',
            '/braco2/soltar@std_msgs/msg/Empty]gz.msgs.Empty',
            '/braco2/estado_pega@std_msgs/msg/String[gz.msgs.StringMsg',
            '/model/objeto/pose@geometry_msgs/msg/Pose[gz.msgs.Pose',
        ],
    )

    ponte_video = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=['/camera_video@sensor_msgs/msg/Image[gz.msgs.Image'],
        condition=IfCondition(LaunchConfiguration('video')),
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict(), sim],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', os.path.join(moveit_share, 'config', 'moveit.rviz')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            sim,
        ],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Abre a janela do Gazebo'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Abre o RViz2 com o MotionPlanning'),
        DeclareLaunchArgument('video', default_value='false', description='Câmera simulada para gravar o vídeo'),
        gazebo,
        pontes_pega,
        ponte_video,
        move_group,
        rviz,
    ])
