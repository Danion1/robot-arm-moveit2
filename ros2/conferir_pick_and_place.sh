#!/bin/bash
# Conferência do nó pick_and_place no Gazebo + MoveIt2, sem janela.
#   1. caso normal, ida e volta (ciclos:=2): o nó sai com 0 e o objeto volta ao lugar
#      inicial, depois de ter passado pelo destino;
#   2. pega no vazio (ponto alcançável sem objeto): o nó sai com 1, diz "pega reprovada", e o
#      objeto não sai do lugar.
# Posição do objeto pelo /model/objeto/pose (só a conferência usa; o nó não).
# Workspace temporário e rede própria. Uso: bash conferir_pick_and_place.sh (~6 min)
AQUI="$(cd "$(dirname "$0")" && pwd)"
W=$(mktemp -d); mkdir $W/src
ln -s "$AQUI/braco2_description" "$AQUI/braco2_moveit_config" "$AQUI/braco2_nos" $W/src/
export ROS_DOMAIN_ID=77
cd $W && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install > build.log 2>&1 || { tail -30 build.log; rm -rf "$W"; exit 1; }
source install/setup.bash
NO="$W/install/braco2_nos/lib/braco2_nos/pick_and_place"     # executável direto
FALHOU=0

pose() {   # x y z do objeto em mm
  timeout 10 ros2 topic echo --once /model/objeto/pose 2>/dev/null | awk '/position:/{f=1;next} f&&/[xyz]:/{printf "%.1f ", $2*1000} /orientation/{exit}'
}
subir() {
  export GZ_PARTITION=conferencia_$$_$1
  setsid ros2 launch braco2_moveit_config pick_and_place.launch.py gui:=false use_rviz:=false > launch_$1.log 2>&1 &
  PID=$!
  for _ in $(seq 1 90); do grep -q "Configured and activated garra_controller" launch_$1.log && break; sleep 1; done
  sleep 3
}
descer() {
  kill -TERM -- -$PID; for _ in $(seq 1 10); do kill -0 -- -$PID 2>/dev/null || break; sleep 1; done; kill -KILL -- -$PID 2>/dev/null
}
perto() {   # perto "x y z" X Y TOL_mm
  python3 -c "import sys,math; x,y,z=map(float,'$1'.split()); sys.exit(0 if math.hypot(x-$2,y-$3)<$4 and abs(z-10)<2 else 1)"
}

echo "== 1. ida e volta (ciclos:=2)"
subir 1
INI=$(pose); echo "   objeto no início: $INI mm"
timeout 300 "$NO" --ros-args -p ciclos:=2 > no_1.log 2>&1; R=$?
grep -E "pega conferida|garra parou|concluído|ERROR" no_1.log | sed 's/^/   /'
FIM=$(pose); echo "   objeto no fim: $FIM mm"
[ $R -eq 0 ] && echo "   [OK] nó saiu com 0" || { echo "   [FALHOU] nó saiu com $R"; tail -8 no_1.log; FALHOU=1; }
[ $(grep -c "pega conferida" no_1.log) -eq 2 ] && echo "   [OK] duas pegas conferidas" || { echo "   [FALHOU] pegas conferidas: $(grep -c 'pega conferida' no_1.log)"; FALHOU=1; }
perto "$FIM" 160 60 5 && echo "   [OK] objeto de volta ao lugar inicial (< 5 mm), sobre a mesa" || { echo "   [FALHOU] objeto fora do lugar"; FALHOU=1; }
descer

echo "== 2. pega no vazio (objeto em (0, 160) mm, onde não há nada)"
subir 2
timeout 300 "$NO" --ros-args -p objeto_x:=0.0 -p objeto_y:=0.160 > no_2.log 2>&1; R=$?
grep -E "garra parou|reprovada" no_2.log | sed 's/^/   /'
FIM=$(pose); echo "   objeto: $FIM mm"
[ $R -eq 1 ] && grep -q "pega reprovada: a garra fechou no vazio" no_2.log && echo "   [OK] nó reprovou a pega e saiu com 1" || { echo "   [FALHOU] saiu com $R"; tail -8 no_2.log; FALHOU=1; }
perto "$FIM" 160 60 1 && echo "   [OK] objeto não saiu do lugar" || { echo "   [FALHOU] objeto mexeu"; FALHOU=1; }
descer

grep -h -E "\[Err\]|process has died" launch_*.log | head -5
rm -rf "$W"
[ $FALHOU -eq 0 ] && echo "RESULTADO: OK" || echo "RESULTADO: FALHOU"
exit $FALHOU
