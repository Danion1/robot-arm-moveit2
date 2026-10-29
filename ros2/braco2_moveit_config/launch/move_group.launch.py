"""Sobe só o move_group (o nó que planeja). É incluído pelo demo.launch.py."""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("braco2", package_name="braco2_moveit_config").to_moveit_configs()
    return generate_move_group_launch(moveit_config)
