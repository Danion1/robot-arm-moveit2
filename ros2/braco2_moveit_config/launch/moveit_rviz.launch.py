"""Abre o RViz2 com o painel MotionPlanning (config/moveit.rviz)."""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("braco2", package_name="braco2_moveit_config").to_moveit_configs()
    return generate_moveit_rviz_launch(moveit_config)
