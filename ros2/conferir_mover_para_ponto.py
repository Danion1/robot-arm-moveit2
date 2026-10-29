#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
conferir_mover_para_ponto.py
============================
Conferência automática do nó do Daniel (`braco2_nos/mover_para_ponto.py`) no
Gazebo, por caminhos INDEPENDENTES dele:

  1. posição da garra: TF world -> tcp (robot_state_publisher, a partir das
     juntas reais do Gazebo) contra o alvo pedido;
  2. ângulos: /joint_states contra a IK (cinematica.solucoes);
  3. torque estático em ombro, cotovelo e punho: esforço do Gazebo contra a
     gravidade calculada com as massas e centros de massa do CAD
     (resultados/massas.json, saída do cad/massa.py) na pose alcançada;
  4. alvo inalcançável: o nó recusa e o braço não se mexe.

Não roda sozinho: precisa do Gazebo aberto NA MESMA rede ROS. Quem abre tudo,
isolado de outra sessão ROS aberta, é o conferir_mover.sh ao lado.
"""

import json
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(AQUI, "braco2_nos"))
from braco2_nos import cinematica as cin   # noqa: E402

G = 9.81
MASSAS = os.path.join(AQUI, "..", "resultados", "massas.json")
TOL_TCP_MM = 1.0
TOL_ANG_GRAUS = 0.2
TOL_TORQUE = 0.02          # 2 % do valor, ou 0,005 N.m, o que for maior
ALVOS = [                  # (x, y, z, phi em graus); o último é inalcançável
    (0.150, 0.0, 0.050, -90.0),
    (0.120, 0.10, 0.030, -90.0),
    (0.0, 0.180, 0.050, -90.0),
    (0.200, 0.0, 0.200, 0.0),
    (0.2785, 0.0, 0.08825, 0.0),
    (0.200, 0.0, 0.100, -90.0),
]
# Massa de cada link, no frame de qual link do cin.fk_matrizes (mordentes: frame
# do link4 com a garra fechada, origem 0 no URDF).
FRAME_DO = {"link2": "link2", "link3": "link3", "link4": "link4",
            "mordente_cima": "link4", "mordente_baixo": "link4"}
DISTAIS = {"junta_ombro": ("link2", ["link2", "link3", "link4", "mordente_cima", "mordente_baixo"]),
           "junta_cotovelo": ("link3", ["link3", "link4", "mordente_cima", "mordente_baixo"]),
           "junta_punho": ("link4", ["link4", "mordente_cima", "mordente_baixo"])}


def aplica(m, p):
    return [sum(m[i][k] * p[k] for k in range(3)) + m[i][3] for i in range(3)]


def torque_esperado(q, massas):
    """Torque que o motor faz para segurar a pose q parada (N.m, sinal do URDF).

    Para cada massa à frente da junta: torque da gravidade = (c - p) x F, com
    F = (0, 0, -m g); o motor faz o oposto, projetado no eixo da junta (-Y do
    frame do link, levado ao mundo pela rotação da cadeia).
    """
    mats = cin.fk_matrizes(*q)
    out = {}
    for junta, (link_junta, elos) in DISTAIS.items():
        mj = mats[link_junta]
        p = [mj[i][3] for i in range(3)]
        eixo = [-mj[i][1] for i in range(3)]
        tau = 0.0
        for e in elos:
            c = aplica(mats[FRAME_DO[e]], massas[e]["com_m"])
            r = [c[i] - p[i] for i in range(3)]
            f = (0.0, 0.0, -massas[e]["massa_kg"] * G)
            cruz = (r[1] * f[2] - r[2] * f[1], r[2] * f[0] - r[0] * f[2], r[0] * f[1] - r[1] * f[0])
            tau -= sum(cruz[i] * eixo[i] for i in range(3))
        out[junta] = tau
    return out


class Conferente(Node):
    def __init__(self):
        super().__init__("conferente")
        self.estado = None
        self.create_subscription(JointState, "/joint_states", self.ao_estado, 10)
        self.tf = Buffer()
        self.ouvinte = TransformListener(self.tf, self)

    def ao_estado(self, msg):
        self.estado = (dict(zip(msg.name, msg.position)), dict(zip(msg.name, msg.velocity)),
                       dict(zip(msg.name, msg.effort)))

    def esperar(self, cond, limite):
        fim = time.time() + limite
        while time.time() < fim:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.estado and cond():
                return True
        return False

    def parado_em(self, alvo_q):
        pos, vel, _ = self.estado
        return all(abs(math.degrees(pos[n] - a)) < TOL_ANG_GRAUS and abs(vel[n]) < 1e-3
                   for n, a in zip(cin.NOMES, alvo_q))


def rodar_no(executavel, alvo):
    args = [executavel, "--ros-args"] + sum(
        (["-p", f"{nome}:={float(v)}"] for nome, v in zip(("x", "y", "z", "phi_graus"), alvo)), [])
    return subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                            start_new_session=True)


def parar_no(proc):
    os.killpg(proc.pid, signal.SIGINT)
    try:
        saida, _ = proc.communicate(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        saida, _ = proc.communicate()
    return saida


def main(executavel):
    with open(MASSAS) as f:
        massas = json.load(f)["elos"]
    rclpy.init()
    no = Conferente()
    ok = True
    if not no.esperar(lambda: True, 30):
        print("[FALHOU] nada no /joint_states: o Gazebo está aberto nesta rede ROS?")
        sys.exit(1)

    for alvo in ALVOS:
        x, y, z, phi = alvo
        sols = cin.solucoes(x, y, z, math.radians(phi))
        print(f"\nAlvo {alvo}")
        antes = dict(no.estado[0])
        proc = rodar_no(executavel, alvo)

        if not sols:
            time.sleep(4)
            no.esperar(lambda: True, 1)
            saida = parar_no(proc)
            parado = all(abs(no.estado[0][n] - antes[n]) < 1e-4 for n in cin.NOMES)
            recusou = "inalcan" in saida.lower()
            ok &= recusou and parado
            print(f"  [{'OK' if recusou and parado else 'FALHOU'}] inalcançável: o nó recusou "
                  f"({recusou}) e o braço ficou parado ({parado})")
            continue

        q = sols[0]
        chegou = no.esperar(lambda: no.parado_em(q), 15)
        no.esperar(lambda: False, 1.0)            # 1 s parado: sobra só a gravidade
        parar_no(proc)
        if not chegou:
            ok = False
            print("  [FALHOU] o braço não parou no alvo em 15 s")
            continue
        pos, _, esf = no.estado
        erro_ang = max(abs(math.degrees(pos[n] - a)) for n, a in zip(cin.NOMES, q))
        t = no.tf.lookup_transform("world", "tcp", Time()).transform.translation
        erro_tcp = math.dist((t.x, t.y, t.z), (x, y, z)) * 1000
        print(f"  [{'OK' if erro_ang < TOL_ANG_GRAUS else 'FALHOU'}] juntas = IK: pior erro {erro_ang:.4f}°")
        print(f"  [{'OK' if erro_tcp < TOL_TCP_MM else 'FALHOU'}] TF world->tcp = alvo: erro {erro_tcp:.3f} mm")
        ok &= erro_ang < TOL_ANG_GRAUS and erro_tcp < TOL_TCP_MM
        esperado = torque_esperado([pos[n] for n in cin.NOMES], massas)
        for n, tau in esperado.items():
            tol = max(TOL_TORQUE * abs(tau), 0.005)
            bom = abs(esf[n] - tau) < tol
            ok &= bom
            print(f"  [{'OK' if bom else 'FALHOU'}] {n:15s} Gazebo {esf[n]:+.4f}  CAD {tau:+.4f} N.m")

    no.destroy_node()
    rclpy.try_shutdown()
    print("\nRESULTADO:", "OK" if ok else "FALHOU")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main(sys.argv[1])
