"""Reabre este pacote no Setup Assistant, pra revisar ou regerar a
configuração (na demo, abriu e carregou o URDF e o SRDF sem erro)."""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_setup_assistant_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("braco2", package_name="braco2_moveit_config").to_moveit_configs()
    return generate_setup_assistant_launch(moveit_config)
