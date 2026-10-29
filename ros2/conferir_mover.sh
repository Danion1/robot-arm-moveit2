#!/bin/bash
# Conferência do nó mover_para_ponto no Gazebo sem janela.
# Workspace temporário e rede ROS/Gazebo própria:
# não toca no seu workspace nem numa sessão ROS aberta.
# Uso: bash conferir_mover.sh     (de qualquer pasta; leva ~2 min)
AQUI="$(cd "$(dirname "$0")" && pwd)"
W=$(mktemp -d); mkdir $W/src
ln -s "$AQUI/braco2_description" "$AQUI/braco2_nos" $W/src/
export ROS_DOMAIN_ID=77 GZ_PARTITION=conferencia_$$
cd $W && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install > build.log 2>&1 || { tail build.log; exit 1; }
source install/setup.bash
setsid ros2 launch braco2_description gazebo.launch.py gui:=false > launch.log 2>&1 &
PID=$!
sleep 40
python3 "$AQUI/conferir_mover_para_ponto.py" "$W/install/braco2_nos/lib/braco2_nos/mover_para_ponto"
R=$?
kill -TERM -- -$PID; sleep 8; kill -KILL -- -$PID 2>/dev/null
rm -rf "$W"
exit $R
