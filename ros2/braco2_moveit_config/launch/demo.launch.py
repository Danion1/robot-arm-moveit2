"""
Sobe tudo de uma vez com hardware falso: robot_state_publisher, move_group
(o planejador), os controladores do ros2_control e o RViz2 com o painel
MotionPlanning.

Segue o formato que o Setup Assistant gera no ROS2 Jazzy (funções prontas
do pacote moveit_configs_utils). Formato testado na demo com o MoveIt2 2.12.4.

Uso (depois de compilar braco2_description e braco2_moveit_config):
    ros2 launch braco2_moveit_config demo.launch.py
    ros2 launch braco2_moveit_config demo.launch.py use_rviz:=false    (sem janela)

Dois erros aparecem no log e são normais numa demo sem câmera:
"No 3D sensor plugin(s) defined for octomap updates" e
"Action server: /recognize_objects not available".
"""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("braco2", package_name="braco2_moveit_config").to_moveit_configs()
    return generate_demo_launch(moveit_config)
