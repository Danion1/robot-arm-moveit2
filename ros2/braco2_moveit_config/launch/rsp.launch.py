"""Sobe o robot_state_publisher com o URDF do braço (publica os TFs)."""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("braco2", package_name="braco2_moveit_config").to_moveit_configs()
    return generate_rsp_launch(moveit_config)
