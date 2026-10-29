#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gravar_moveit.py
================
Primeira metade da conferência MoveIt2 x CAD.
Com o MoveIt2 rodando (hardware falso), faz perguntas e GRAVA as respostas,
sem julgar nada:

  1. validade (colisão) de poses sorteadas, sem mesa e depois com a mesa na cena;
  2. planeja E executa uma sequência de pick and place (alvos em JUNTAS, da IK
     analítica do braco2_nos) e um CASO DIFÍCIL: duas poses livres cuja linha
     reta no espaço das juntas atravessaria a mesa ou a base;
  3. cinemática inversa do próprio MoveIt2 (KDL, só posição) em 3 pontos.

Grava em resultados/moveit_teste.json. Quem confere é cad/conferir_moveit.py,
com os sólidos do CAD: um caminho independente do MoveIt2.

Uso: chamado pelo ros2/conferir_moveit.sh (sobe a demo sem janela antes).
"""

import json
import math
import os
import random
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (CollisionObject, Constraints, JointConstraint, MoveItErrorCodes,
                             PlanningScene, RobotState)
from moveit_msgs.srv import ApplyPlanningScene, GetPositionIK, GetStateValidity
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

AQUI = os.path.dirname(os.path.abspath(__file__))
PROJETO = os.path.dirname(AQUI)
SAIDA = os.path.join(PROJETO, "resultados", "moveit_teste.json")
sys.path.insert(0, os.path.join(AQUI, "braco2_nos"))
sys.path.insert(0, os.path.join(PROJETO, "cad"))
from braco2_nos import cinematica as cin   # noqa: E402  (só calcula os alvos)
import gerar_moveit_config as gm            # noqa: E402  (caixas, só para ACHAR o caso difícil)

BRACO = list(cin.NOMES)
MESA_TOPO_M = -0.002     # 2 mm abaixo da base, senão a base "colide" por encostar
ESCALA = 0.5             # pede no máximo metade da velocidade e da aceleração
GARRA = (0.0, 0.01, 0.02)  # aberturas sorteadas (m por mordente): o CAD guarda uma montagem por valor
N_POSES = 300            # por bateria (sem mesa / com mesa)
# Tolerância do alvo NA UNIDADE DA JUNTA: 0,001 rad (0,06°) nas revolutas, mas na garra
# (prismática, metros) 0,001 seria 1 mm, e o planejador parou a 0,7 mm do alvo (2026-10-14).
TOL = {False: 0.001, True: 0.0001}
PENETRA_MESA = 0.010     # caso difícil: a reta afunda 10 mm na mesa (a margem da caixa é só 1 mm)


class Gravador(Node):
    def __init__(self):
        super().__init__("gravador_conferencia")
        self.validade = self.create_client(GetStateValidity, "check_state_validity")
        self.cena = self.create_client(ApplyPlanningScene, "apply_planning_scene")
        self.ik = self.create_client(GetPositionIK, "compute_ik")
        self.mover = ActionClient(self, MoveGroup, "move_action")
        self.estado = None
        self.create_subscription(JointState, "joint_states", self._estado, 10)
        for cli in (self.validade, self.cena, self.ik):
            if not cli.wait_for_service(timeout_sec=30.0):
                raise RuntimeError(f"serviço {cli.srv_name} não apareceu: o MoveIt2 está rodando?")
        if not self.mover.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("ação move_action não apareceu")

    def _estado(self, msg):
        self.estado = dict(zip(msg.name, msg.position))

    def chamar(self, cliente, req):
        fut = cliente.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
        return fut.result()

    def estado_valido(self, q):
        """q = {junta: valor}. Devolve (válido, pares em contato)."""
        req = GetStateValidity.Request(group_name="braco", robot_state=RobotState())
        req.robot_state.joint_state.name = list(q)
        req.robot_state.joint_state.position = [float(v) for v in q.values()]
        r = self.chamar(self.validade, req)
        return r.valid, sorted({tuple(sorted((c.contact_body_1, c.contact_body_2))) for c in r.contacts})

    def por_mesa(self):
        mesa = CollisionObject(id="mesa", operation=CollisionObject.ADD)
        mesa.header.frame_id = "world"
        pose = Pose()
        pose.position.z = MESA_TOPO_M - 0.01
        pose.orientation.w = 1.0
        mesa.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[1.2, 1.2, 0.02])]
        mesa.primitive_poses = [pose]
        cena = PlanningScene(is_diff=True)
        cena.world.collision_objects = [mesa]
        return self.chamar(self.cena, ApplyPlanningScene.Request(scene=cena)).success

    def planejar_e_executar(self, grupo, alvo):
        """alvo = {junta: valor}. Devolve o código, a trajetória planejada e onde o robô parou."""
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = grupo
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = ESCALA
        req.max_acceleration_scaling_factor = ESCALA
        req.start_state.is_diff = True
        req.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=j, position=float(p), tolerance_above=TOL[j == "junta_garra"],
                            tolerance_below=TOL[j == "junta_garra"], weight=1.0) for j, p in alvo.items()])]
        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        inicio = dict(self.estado or {})

        fut = self.mover.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            return {"codigo": None, "ok": False, "motivo": "pedido recusado"}
        fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, fut, timeout_sec=120.0)
        res = fut.result().result
        traj = res.planned_trajectory.joint_trajectory
        fim = time.time() + 1.0          # deixa o /joint_states alcançar o fim do movimento
        while time.time() < fim:
            rclpy.spin_once(self, timeout_sec=0.05)
        return {
            "codigo": res.error_code.val,
            "ok": res.error_code.val == MoveItErrorCodes.SUCCESS,
            "tempo_planejamento_s": res.planning_time,
            "estado_inicial": inicio,
            "juntas": list(traj.joint_names),
            "pontos": [{"t": p.time_from_start.sec + p.time_from_start.nanosec * 1e-9,
                        "q": list(p.positions), "v": list(p.velocities)} for p in traj.points],
            "estado_final": dict(self.estado or {}),
        }

    def ik_posicao(self, x, y, z):
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = "braco"
        r.avoid_collisions = True
        r.robot_state.is_diff = True
        r.pose_stamped.header.frame_id = "world"
        r.pose_stamped.pose.position.x, r.pose_stamped.pose.position.y = x, y
        r.pose_stamped.pose.position.z = z
        r.pose_stamped.pose.orientation.w = 1.0
        r.timeout.sec = 2
        res = self.chamar(self.ik, req)
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return {"codigo": res.error_code.val, "alvo": [x, y, z],
                "q": [sol[j] for j in BRACO] if sol and all(j in sol for j in BRACO) else None}


def poses_sorteadas(n, semente):
    rng = random.Random(semente)
    poses = []
    for _ in range(n):
        q = {j: rng.uniform(*cin.LIMITES[j]) for j in BRACO}
        q["junta_garra"] = rng.choice(GARRA)
        poses.append(q)
    return poses


def achar_caso_dificil(semente=7):
    """Duas poses livres (caixas + mesa) cuja RETA nas juntas afunda na mesa.

    Usa as caixas do gerar_moveit_config só para ESCOLHER o caso; quem confere
    a trajetória do MoveIt2 é o CAD. A reta tem de afundar PENETRA_MESA, não só
    encostar: a 1ª versão aceitava 0,9 mm, que era só a margem da caixa, e no
    CAD a reta saía livre (o caso não provava nada)."""
    _, juntas, _ = gm.ler_urdf()
    with open(gm.CAIXAS, encoding="utf-8") as f:
        caixas = json.load(f)["caixas"]
    vizinhos = {(jt["pai"], jt["filho"]) for jt in juntas} | {("mordente_cima", "mordente_baixo")}
    pares = [(a, b) for i, a in enumerate(caixas) for b in list(caixas)[i + 1:]
             if (a, b) not in vizinhos and (b, a) not in vizinhos]
    sinais = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])

    def z_minimo(cm):
        return min((c + (sinais * h) @ R.T)[:, 2].min()
                   for link, lista in cm.items() if link != "base_link" for c, R, h in lista)

    def caixas_de(q):
        return gm.caixas_no_mundo(caixas, gm.fk(juntas, dict(q, junta_garra=0.0)))

    def colide(q):
        cm = caixas_de(q)
        return z_minimo(cm) < MESA_TOPO_M or any(gm.links_colidem(cm, a, b) for a, b in pares)

    def perto_da_mesa(rng):
        """Pose sorteada com o TCP a 20-60 mm da mesa (IK analítica): é aí que a reta afunda."""
        while True:
            r, ang = rng.uniform(0.10, 0.25), rng.uniform(-1.0, 1.0)
            sol = cin.solucoes(r * math.cos(ang), r * math.sin(ang), rng.uniform(0.02, 0.06),
                               math.radians(rng.uniform(-90, 0)))
            if sol:
                return dict(zip(BRACO, rng.choice(sol)))

    rng = random.Random(semente)
    for _ in range(5000):
        a, b = perto_da_mesa(rng), perto_da_mesa(rng)
        if abs(b["junta_base"] - a["junta_base"]) > math.radians(60) or colide(a) or colide(b):
            continue    # base quase parada: o desvio tem de vir do ombro/cotovelo/punho
        reta = [{j: a[j] + k / 40 * (b[j] - a[j]) for j in BRACO} for k in range(1, 40)]
        if min(z_minimo(caixas_de(q)) for q in reta) < MESA_TOPO_M - PENETRA_MESA:
            return a, b
    raise RuntimeError("caso difícil não encontrado")


def main():
    rclpy.init()
    no = Gravador()
    saida = {"mesa_topo_m": MESA_TOPO_M, "escala_velocidade": ESCALA}
    try:
        print(f"1. validade de {N_POSES} poses sorteadas, sem mesa...")
        saida["validade_sem_mesa"] = [dict(zip(("valido", "contatos"), no.estado_valido(q)), q=q)
                                      for q in poses_sorteadas(N_POSES, 101)]
        ok = no.por_mesa()
        print("2. mesa na cena:", "ok" if ok else "FALHOU")
        saida["mesa_na_cena"] = ok
        saida["validade_com_mesa"] = [dict(zip(("valido", "contatos"), no.estado_valido(q)), q=q)
                                      for q in poses_sorteadas(N_POSES, 202)]

        print("3. planejar e executar...")
        baixo = math.radians(-90)
        acima = cin.solucoes(0.160, 0.060, 0.070, baixo)[0]
        pegar = cin.solucoes(0.160, 0.060, 0.030, baixo)[0]
        pronto = (0.0, 1.0472, -1.48353, -1.13446)      # poses_nomeadas.xml
        alto = (-0.5236, 1.22173, -0.69813, -0.34907)
        de, para = achar_caso_dificil()
        saida["caso_dificil"] = {"de": de, "para": para}
        braco = lambda q: dict(zip(BRACO, q))            # noqa: E731
        passos = [
            ("abrir a garra", "garra", {"junta_garra": 0.02}),
            ("ir para pronto", "braco", braco(pronto)),
            ("acima do objeto (IK analítica)", "braco", braco(acima)),
            ("descer até o objeto", "braco", braco(pegar)),
            ("fechar a garra", "garra", {"junta_garra": 0.0}),
            ("subir com o objeto", "braco", braco(acima)),
            ("ir para alto", "braco", braco(alto)),
            ("início do caso difícil", "braco", de),
            ("CASO DIFÍCIL (a reta colidiria)", "braco", para),
            ("voltar para pronto (fim)", "braco", braco(pronto)),
        ]
        saida["movimentos"] = []
        for nome, grupo, alvo in passos:
            r = no.planejar_e_executar(grupo, alvo)
            r.update({"nome": nome, "grupo": grupo, "alvo": alvo})
            saida["movimentos"].append(r)
            print(f"   {nome}: código {r['codigo']} ({'sucesso' if r['ok'] else 'FALHOU'}), "
                  f"{len(r.get('pontos', []))} pontos")

        print("4. cinemática inversa do MoveIt2 (KDL, só posição)...")
        saida["ik"] = [no.ik_posicao(0.150, 0.050, 0.100), no.ik_posicao(0.100, -0.120, 0.180),
                       no.ik_posicao(0.400, 0.0, 0.100)]
        for r in saida["ik"]:
            print(f"   alvo {r['alvo']}: código {r['codigo']}")
    finally:
        no.destroy_node()
        rclpy.shutdown()

    with open(SAIDA, "w", encoding="utf-8") as f:
        json.dump(saida, f, indent=1)
    print("Gravado:", SAIDA)


if __name__ == "__main__":
    main()
