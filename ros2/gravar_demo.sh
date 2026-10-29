#!/bin/bash
# Grava o vídeo de demonstração do pick and place (Propostas e estudos.md, item 1, opção A
# com câmera simulada): Gazebo sem janela com a câmera do vídeo, o nó pick_and_place indo
# e voltando (ciclos:=2), e o gravar_video.py escrevendo midia/pick_and_place.mp4.
# A simulação roda mais devagar que o relógio (espera cada quadro); o vídeo sai em tempo real.
# Uso: bash gravar_demo.sh     (de qualquer pasta; ~4 min)
AQUI="$(cd "$(dirname "$0")" && pwd)"
SAIDA="$AQUI/../midia/pick_and_place.mp4"; mkdir -p "$(dirname "$SAIDA")"
W=$(mktemp -d); mkdir $W/src
ln -s "$AQUI/braco2_description" "$AQUI/braco2_moveit_config" "$AQUI/braco2_nos" $W/src/
export ROS_DOMAIN_ID=77 GZ_PARTITION=demo_$$
cd $W && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install > build.log 2>&1 || { tail -30 build.log; rm -rf "$W"; exit 1; }
source install/setup.bash
setsid ros2 launch braco2_moveit_config pick_and_place.launch.py gui:=false use_rviz:=false video:=true > launch.log 2>&1 &
PID=$!
for _ in $(seq 1 120); do grep -q "Configured and activated garra_controller" launch.log && break; sleep 1; done
python3 "$AQUI/gravar_video.py" "$SAIDA" &
GV=$!
"$W/install/braco2_nos/lib/braco2_nos/pick_and_place" --ros-args -p ciclos:=2 2>&1 | grep -E "ciclo|pega|concluído|ERROR"
R=${PIPESTATUS[0]}
sleep 3                                  # um respiro parado no fim do vídeo
kill -INT $GV; wait $GV; L=$?
kill -TERM -- -$PID; for _ in $(seq 1 10); do kill -0 -- -$PID 2>/dev/null || break; sleep 1; done; kill -KILL -- -$PID 2>/dev/null
rm -rf "$W"
[ $R -eq 0 ] && [ $L -eq 0 ] && echo "VÍDEO OK: $SAIDA" || echo "FALHOU (nó: $R, vídeo liso: $L)"
