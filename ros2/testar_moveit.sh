#!/bin/bash
# Sobe o MoveIt2 com hardware FALSO, sem janela, num workspace temporário, e
# confere que ele ficou pronto: move_group planejando, 3 controladores ativos,
# log sem erro (fora os dois avisos normais sem câmera).
# Não confere planejamento contra o CAD: isso é a conferência MoveIt x CAD.
# Workspace temporário e rede ROS/Gazebo própria: não mexe em nenhum workspace aberto.

PROJ="$(cd "$(dirname "$0")" && pwd)"   # pasta ros2/ deste projeto
W=$(mktemp -d); mkdir $W/src
ln -s "$PROJ/braco2_description" "$PROJ/braco2_moveit_config" $W/src/
export ROS_DOMAIN_ID=77
cd $W && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install > build.log 2>&1 || { tail -30 build.log; rm -rf "$W"; exit 1; }
source install/setup.bash
setsid ros2 launch braco2_moveit_config demo.launch.py use_rviz:=false > demo.log 2>&1 &
PID=$!
FALHOU=0
for _ in $(seq 1 60); do grep -q "You can start planning now" demo.log && break; sleep 1; done
grep -q "You can start planning now" demo.log && echo "[OK] move_group pronto" || { echo "[FALHOU] move_group não subiu"; FALHOU=1; }
for c in joint_state_broadcaster braco_controller garra_controller; do
  for _ in $(seq 1 30); do grep -q "Configured and activated $c" demo.log && break; sleep 1; done
  grep -q "Configured and activated $c" demo.log && echo "[OK] $c ativo" || { echo "[FALHOU] $c não ativou"; FALHOU=1; }
done
echo "== /joint_states (nomes e posições)"
timeout 10 ros2 topic echo --once /joint_states | grep -A8 -E "^(name|position):"
kill -TERM -- -$PID; for _ in $(seq 1 10); do kill -0 -- -$PID 2>/dev/null || break; sleep 1; done; kill -KILL -- -$PID 2>/dev/null
ERROS=$(grep -E "\[ERROR\]|\[FATAL\]|process has died" demo.log | grep -v -E "No 3D sensor plugin|recognize_objects")
[ -z "$ERROS" ] && echo "[OK] log sem erros" || { echo "[FALHOU] erros no log:"; echo "$ERROS" | head -20; FALHOU=1; }
echo "== avisos (informativo)"; grep "\[WARN\]" demo.log | sed 's/.*\[WARN\]//' | sort | uniq -c | head -15
pgrep -a -x "move_group|robot_state_pub|ros2_control_no|rviz2"
rm -rf "$W"; exit $FALHOU
