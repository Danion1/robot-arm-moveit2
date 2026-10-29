#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
conferir_pega.py
================
Conferência do mundo do pick and place no Gazebo (decisão C, 2026-10-14), ANTES
do nó do Daniel: prova que a física e a pega se comportam como o nó vai supor.
O braço vai pelo MoveIt2 (alvos em juntas da IK analítica); a garra, direto
pelo /garra_controller/joint_trajectory (como o nó fará).

  0. a junta espelho acompanha a junta_garra (mimic no Bullet-Featherstone);
  1. o objeto começa SOLTO: o braço se move e ele não sai do lugar;
  2. pega no vazio: a garra fecha até ~0 (é assim que o nó vai reprovar);
  3. descer em volta do objeto não o empurra;
  4. fechar no objeto (alvo 1 mm além do contato): a garra PARA na largura dele
     (20 mm -> 10 mm por dedo + 1 mm de margem da caixa), os dois dedos iguais,
     o objeto não é empurrado; esforço da garra x stall;
  5. parar de apertar (garra fica onde parou), prender (DetachableJoint) e subir:
     o objeto sobe junto;
  6. levar ao outro lado, soltar, abrir e RECUAR NA VERTICAL: o objeto fica na
     mesa no lugar novo.

Uso: chamado pelo ros2/conferir_pega.sh (sobe o Gazebo + MoveIt2 sem janela antes).
"""

import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)
from gravar_moveit import BRACO, Gravador, cin   # noqa: E402

OBJ_INICIAL = (0.160, 0.060, 0.010)    # worlds/pick_and_place.sdf
LUGAR = (0.160, -0.060)                # onde o objeto é deixado
LADO = 0.020                           # aresta do cubo (m)
STALL_GARRA = 52.8                     # N, <limit effort> da junta_garra (stall / 2R)
MARGEM = 0.001       # a caixa de colisão do dedo é 1 mm maior (malhas_urdf.MARGEM): o contato
                     # acontece com 11 mm por dedo num cubo de 20 mm, não 10
CONTATO = LADO / 2 + MARGEM
# Fechar até 0 com o objeto no meio aperta com ~28 N e o objeto é espremido para fora
# (o motor deixa o dedo entrar 3 mm no cubo). Como no robô real, manda-se o alvo só
# um pouco além do contato: 1 mm por dedo, ~3 N (medido em 2026-10-14).
APERTO = 0.001
BAIXO = math.radians(-90)
FALHAS = []


def conferir(nome, ok, info=""):
    print(f"  [{'OK' if ok else 'FALHOU'}] {nome}" + (f": {info}" if info else ""))
    if not ok:
        FALHAS.append(nome)
    return ok


class Conferente(Gravador):
    def __init__(self):
        super().__init__()
        self.obj = None
        self.estados_pega = []
        self.esforco = {}
        self.create_subscription(Pose, "/model/objeto/pose", self._obj, 10)
        self.create_subscription(String, "/braco2/estado_pega", lambda m: self.estados_pega.append(m.data), 10)
        self.garra = self.create_publisher(JointTrajectory, "/garra_controller/joint_trajectory", 10)
        self.prender = self.create_publisher(Empty, "/braco2/prender", 10)
        self.soltar = self.create_publisher(Empty, "/braco2/soltar", 10)

    def _estado(self, msg):
        super()._estado(msg)
        if msg.effort:
            self.esforco = dict(zip(msg.name, msg.effort))

    def _obj(self, msg):
        self.obj = (msg.position.x, msg.position.y, msg.position.z)

    def esperar(self, s):
        fim = time.time() + s
        while time.time() < fim:
            rclpy.spin_once(self, timeout_sec=0.02)

    def publicar(self, pub, msg):
        while pub.get_subscription_count() == 0:
            self.esperar(0.1)
        pub.publish(msg)

    def mover_garra(self, valor, espera=2.0, duracao=1):
        t = JointTrajectory(joint_names=["junta_garra"])
        t.points = [JointTrajectoryPoint(positions=[valor])]
        t.points[0].time_from_start.sec = duracao
        self.publicar(self.garra, t)
        self.esperar(espera)
        return self.estado["junta_garra"], self.estado["junta_garra_espelho"]

    def braco(self, nome, q):
        r = self.planejar_e_executar("braco", dict(zip(BRACO, q)))
        self.esperar(1.0)
        if not r["ok"]:
            conferir(nome, False, f"MoveIt2 devolveu {r['codigo']}")
        return r["ok"]


def mm(a, b):
    return math.dist(a, b) * 1000


def main():
    rclpy.init()
    no = Conferente()
    try:
        no.esperar(3.0)
        if no.obj is None or no.estado is None:
            conferir("pose do objeto e /joint_states chegando", False)
            return
        pronto = (0.0, 1.0472, -1.48353, -1.13446)    # poses_nomeadas.xml

        print("\n0. Mimic: os dois dedos andam juntos")
        g, e = no.mover_garra(0.020)
        conferir("garra aberta: espelho = junta_garra", abs(g - e) < 0.0005 and abs(g - 0.020) < 0.0005,
                 f"{g * 1000:.2f} x {e * 1000:.2f} mm")

        print("\n1. Objeto começa solto")
        no.publicar(no.soltar, Empty())      # se o plugin começou preso, solta; se não, ele só avisa
        no.esperar(1.0)
        inicio = no.obj
        conferir("objeto nasce no lugar do mundo", mm(inicio, OBJ_INICIAL) < 1.0, f"a {mm(inicio, OBJ_INICIAL):.2f} mm")
        if no.braco("ir para pronto", pronto):
            conferir("braço se moveu e o objeto ficou", mm(no.obj, inicio) < 0.5, f"andou {mm(no.obj, inicio):.2f} mm")

        print("\n2. Pega no vazio (o nó reprova assim)")
        g, e = no.mover_garra(0.0)
        conferir("garra fecha até ~0 sem objeto", g < 0.001, f"{g * 1000:.2f} mm por dedo")
        no.mover_garra(0.020)

        print("\n3. Descer em volta do objeto")
        acima = cin.solucoes(OBJ_INICIAL[0], OBJ_INICIAL[1], 0.060, BAIXO)[0]
        pegar = cin.solucoes(OBJ_INICIAL[0], OBJ_INICIAL[1], 0.020, BAIXO)[0]
        no.braco("acima do objeto", acima)
        no.braco("descer", pegar)
        conferir("descer não empurrou o objeto", mm(no.obj, inicio) < 2.0, f"andou {mm(no.obj, inicio):.2f} mm")

        print("\n4. Fechar no objeto")
        antes = no.obj
        g, e = no.mover_garra(CONTATO - APERTO, espera=3.0)
        conferir("garra parou no objeto, antes do alvo", g > CONTATO - APERTO + 0.0002 and abs(g - CONTATO) < 0.0008,
                 f"{g * 1000:.2f} mm por dedo (contato em {CONTATO * 1000:.1f}, alvo {(CONTATO - APERTO) * 1000:.1f})")
        conferir("os dois dedos iguais", abs(g - e) < 0.0005, f"{g * 1000:.2f} x {e * 1000:.2f} mm")
        # ao apertar, o objeto se acomoda entre os dedos (0,5 a 1,3 mm medidos): é o centramento
        conferir("objeto não foi empurrado", mm(no.obj, antes) < 2.0, f"andou {mm(no.obj, antes):.2f} mm")
        f = abs(no.esforco.get("junta_garra", float("nan")))
        print(f"     esforço da garra apertando: {f:.1f} N (stall {STALL_GARRA} N; informativo)")

        print("\n5. Parar de apertar, prender e subir")
        # Aperto + junta fixa brigam (o dedo está 0,2 mm dentro do objeto) e, ao soltar,
        # a energia guardada joga o objeto 3-5 mm (2026-10-14). Então: o aperto serve só
        # para CONFERIR a pega; depois a garra fica onde parou e quem segura é a junta.
        no.mover_garra(g, espera=1.0)
        no.publicar(no.prender, Empty())
        no.esperar(1.0)
        print(f"     /braco2/estado_pega recebeu: {no.estados_pega or 'nada'}")
        z0 = no.obj[2]
        no.braco("subir com o objeto", acima)
        conferir("objeto subiu junto (~40 mm)", abs(no.obj[2] - z0 - 0.040) < 0.005,
                 f"subiu {(no.obj[2] - z0) * 1000:.1f} mm")

        print("\n6. Levar, soltar e abrir")
        acima2 = cin.solucoes(LUGAR[0], LUGAR[1], 0.060, BAIXO)[0]
        # Soltar 2 mm acima da mesa: apoiado e ainda preso, a junta fixa empurra o objeto
        # contra a mesa e ele pula ao soltar (2-3 mm, medido em 2026-10-14)
        soltar = cin.solucoes(LUGAR[0], LUGAR[1], 0.022, BAIXO)[0]
        def onde(rotulo):
            print(f"     {rotulo}: objeto em {tuple(round(v * 1000, 1) for v in no.obj)} mm")
        no.braco("levar ao outro lado", acima2)
        onde("acima do lugar novo")
        no.braco("descer para soltar", soltar)
        onde("desceu")
        no.publicar(no.soltar, Empty())
        no.esperar(0.5)
        onde("soltou")
        no.mover_garra(0.020, espera=3.0, duracao=2)    # abrir devagar: aberto rápido, o dedo "chuta"
        onde("abriu")
        # Recuar NA VERTICAL antes de ir embora: o MoveIt2 não sabe que o objeto existe
        # (não está na cena), e sair de lado com a mão baixa arrasta o objeto (~25 mm, 2026-10-14).
        no.braco("recuar para cima", acima2)
        onde("recuou")
        no.braco("voltar para pronto", pronto)
        no.esperar(1.0)
        onde("braço em pronto")
        fim = no.obj
        conferir("objeto ficou no lugar novo, sobre a mesa", mm(fim[:2], LUGAR) < 5.0 and abs(fim[2] - LADO / 2) < 0.002,
                 f"a {mm(fim[:2], LUGAR):.1f} mm do alvo em XY, z = {fim[2] * 1000:.1f} mm")
    finally:
        no.destroy_node()
        rclpy.try_shutdown()
    if FALHAS:
        sys.exit(f"FALHOU: {FALHAS}")
    print("\nTudo conferido.")


if __name__ == "__main__":
    main()
