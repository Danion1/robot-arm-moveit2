#!/bin/bash
# Conferência MoveIt2 x CAD, de ponta a ponta:
#   1. sobe o MoveIt2 com hardware falso, sem janela, num workspace temporário
#      e numa rede ROS própria;
#   2. gravar_moveit.py pergunta ao MoveIt2 e grava resultados/moveit_teste.json;
#   3. encerra o MoveIt2 e o cad/conferir_moveit.py confere tudo com os sólidos do CAD.
# Uso: bash conferir_moveit.sh     (de qualquer pasta; leva ~10 min)
AQUI="$(cd "$(dirname "$0")" && pwd)"
W=$(mktemp -d); mkdir $W/src
ln -s "$AQUI/braco2_description" "$AQUI/braco2_moveit_config" $W/src/
export ROS_DOMAIN_ID=77
cd $W && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install > build.log 2>&1 || { tail -30 build.log; rm -rf "$W"; exit 1; }
source install/setup.bash
setsid ros2 launch braco2_moveit_config demo.launch.py use_rviz:=false > demo.log 2>&1 &
PID=$!
for _ in $(seq 1 60); do grep -q "You can start planning now" demo.log && break; sleep 1; done
sleep 5    # controladores
python3 "$AQUI/gravar_moveit.py"
R=$?
kill -TERM -- -$PID; for _ in $(seq 1 10); do kill -0 -- -$PID 2>/dev/null || break; sleep 1; done; kill -KILL -- -$PID 2>/dev/null
rm -rf "$W"
[ $R -ne 0 ] && { echo "gravar_moveit.py falhou"; exit $R; }
cd "$AQUI/../cad" && "${CADQUERY_PYTHON:-python3}" conferir_moveit.py   # Python com CadQuery
