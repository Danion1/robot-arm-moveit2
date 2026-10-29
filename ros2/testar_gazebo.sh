#!/bin/bash
# Abre o gazebo.launch.py sem janela num workspace temporário e lê o estado.
# Workspace temporário: não toca no seu workspace.
# Ordem dos esforços = ordem dos nomes impressos (o /joint_states não segue o URDF).
# Esperado na pose zero (massa.py): ombro 0,667 N.m, cotovelo 0,278, punho 0,070.

PROJ="$(cd "$(dirname "$0")" && pwd)"   # pasta ros2/ deste projeto
W=$(mktemp -d); mkdir $W/src
ln -s "$PROJ/braco2_description" $W/src/
export ROS_DOMAIN_ID=77 GZ_PARTITION=teste_claude_$$   # rede própria: não cruza com outra sessão ROS aberta
cd $W && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select braco2_description > build.log 2>&1 || { tail build.log; exit 1; }
source install/setup.bash
setsid ros2 launch braco2_description gazebo.launch.py gui:=false > launch.log 2>&1 &
PID=$!
sleep 45
echo "== controladores"; timeout 15 ros2 control list_controllers
for t in 1 2 3; do
  echo "== /joint_states (leitura $t)"
  timeout 10 ros2 topic echo --once --field position /joint_states; timeout 10 ros2 topic echo --once --field effort /joint_states
  timeout 10 ros2 topic echo --once --field name /joint_states | tr '\n' ' '; echo
  sleep 10
done
echo "== erros/avisos no log"; grep -i -E "error|warn|fail" launch.log | grep -v -i "octomap" | head -20
kill -TERM -- -$PID; sleep 10; kill -KILL -- -$PID 2>/dev/null
pgrep -a -x "gz|ruby|robot_state_pub|parameter_bri"; rm -rf "$W"; echo fim
