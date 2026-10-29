#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
conferir_cinematica.py
======================
Confere a cinemática do nó (`ros2/braco2_nos/braco2_nos/cinematica.py`)
contra a CADEIA DE SÓLIDOS do `montagem.py`: o caminho independente, que
não usa nenhuma fórmula, só gira e move as peças do CAD.

Para cada pose (as de trabalho do montagem.py + aleatórias dentro dos limites):
  1. TCP pelos sólidos (mm) == fk_analitica (m);
  2. ik(TCP dos sólidos) devolve os mesmos ângulos da pose.
O arquivo da cinemática NÃO é copiado: é importado do pacote (um dono só).

Uso (de dentro de cad/):
    python conferir_cinematica.py
"""

import math
import os
import random
import sys

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(AQUI, "..", "ros2", "braco2_nos"))

from braco2_nos import cinematica as cin   # noqa: E402
import montagem as m                       # noqa: E402

TOL_MM = 1e-6
TOL_GRAUS = 1e-6


def poses():
    lista = [p[:4] for p in m.POSES_TRABALHO.values()]
    rng = random.Random(5)
    while len(lista) < 30:
        q = tuple(math.degrees(rng.uniform(lo, hi)) for lo, hi in cin.LIMITES.values())
        if abs(q[2]) > 1.0 and cin.fk_analitica(0.0, *map(math.radians, q[1:]))[0] > 1e-3:
            lista.append(q)
    return lista


def main():
    pior_mm = pior_graus = 0.0
    for q in poses():
        pelo_cad = m.tcp_cadeia(q)                                  # mm, sólidos
        q_rad = tuple(math.radians(a) for a in q)
        x, y, z, phi = cin.fk_analitica(*q_rad)
        pior_mm = max(pior_mm, *(abs(a - b * 1000.0) for a, b in zip(pelo_cad, (x, y, z))))

        alvo = [v / 1000.0 for v in pelo_cad]
        ramo = "baixo" if q[2] > 0 else "cima"
        volta = cin.ik(*alvo, phi, cotovelo=ramo)
        pior_graus = max(pior_graus, *(abs(math.degrees(cin.normaliza(a - b)))
                                       for a, b in zip(volta, q_rad)))
    print(f"{len(poses())} poses | FK fórmula x sólidos: pior erro {pior_mm:.2e} mm"
          f" | IK(sólidos) x pose: pior erro {pior_graus:.2e}°")
    ok = pior_mm < TOL_MM and pior_graus < TOL_GRAUS
    print("OK" if ok else "FALHOU")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
