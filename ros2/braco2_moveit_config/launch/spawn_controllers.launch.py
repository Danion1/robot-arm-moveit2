"""Ativa os controladores do ros2_control listados em moveit_controllers.yaml e o joint_state_broadcaster."""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("braco2", package_name="braco2_moveit_config").to_moveit_configs()
    return generate_spawn_controllers_launch(moveit_config)
