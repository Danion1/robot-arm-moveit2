#!/bin/bash
# Conferência do mundo do pick and place (Gazebo + MoveIt2), sem janela.
# Workspace temporário e rede ROS/Gazebo própria.
# Uso: bash conferir_pega.sh     (de qualquer pasta; ~3 min)
AQUI="$(cd "$(dirname "$0")" && pwd)"
W=$(mktemp -d); mkdir $W/src
ln -s "$AQUI/braco2_description" "$AQUI/braco2_moveit_config" $W/src/
export ROS_DOMAIN_ID=77 GZ_PARTITION=conferencia_$$
cd $W && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install > build.log 2>&1 || { tail -30 build.log; rm -rf "$W"; exit 1; }
source install/setup.bash
setsid ros2 launch braco2_moveit_config pick_and_place.launch.py gui:=false use_rviz:=false > launch.log 2>&1 &
PID=$!
for _ in $(seq 1 90); do grep -q "Configured and activated garra_controller" launch.log && break; sleep 1; done
python3 "$AQUI/conferir_pega.py"
R=$?
grep -E "\[Err\]|\[ERROR\]|process has died" launch.log | grep -v "No 3D sensor plugin" | head -5
kill -TERM -- -$PID; for _ in $(seq 1 10); do kill -0 -- -$PID 2>/dev/null || break; sleep 1; done; kill -KILL -- -$PID 2>/dev/null
rm -rf "$W"
exit $R
