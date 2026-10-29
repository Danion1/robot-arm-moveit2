#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gerar_moveit_config.py
======================
Escreve os arquivos DERIVADOS do pacote ros2/braco2_moveit_config (o que o
Setup Assistant do MoveIt2 escreveria), a partir do URDF gerado e das caixas
de colisão, sem número digitado à mão aqui:

  config/braco2.srdf ............ grupos, efetuador, poses nomeadas (do Daniel,
                                  em config/poses_nomeadas.xml) e pares de
                                  colisão desligados
  config/braco2.urdf.xacro ...... braco.urdf + <ros2_control> com hardware FALSO
  config/joint_limits.yaml ...... velocidade do URDF (datasheet) + aceleração ASSUMIDA
  config/moveit_controllers.yaml  controladores e juntas do controladores.yaml do Daniel
  config/ros2_controllers.yaml .. cópia do controladores.yaml (uma fonte só)
  config/initial_positions.yaml . pose em que o hardware falso começa
  config/pilz_cartesian_limits.yaml, config/kinematics.yaml

Os arquivos fixos (launch, rviz, package.xml, CMakeLists) são escritos à mão
no pacote.

Pares de colisão desligados ("matriz de colisão", decisão 2026-10-14):
  Adjacent = ligados por uma junta (sempre se encostam no eixo);
  Never    = nunca colidiram em N poses sorteadas dentro dos limites do URDF.
  O que colide em alguma pose CONTINUA sendo checado pelo MoveIt2.
  Pares acoplados por mimic (os dois mordentes) = "Default": fechados, se encostam.
  Colisão aqui = caixas (resultados/caixas_colisao.json, do malhas_urdf.py)
  sobrepostas pelo teste dos eixos separadores (SAT). As caixas são as
  mesmas que formam o STL de colisão; caixa sobreposta é colisão mesmo
  quando a malha do FCL não acusaria (uma caixa inteira dentro da outra),
  então o erro é para o lado seguro: menos pares desligados.

Verificações (param antes de gravar):
  1. a cinemática deste script (lida do URDF) leva o TCP ao mesmo ponto da
     IK do pacote de nós (cinematica.py), em várias poses;
  2. as poses de trabalho do montagem.py estão livres de colisão com as caixas
     (o mesmo resultado do malhas_urdf.py, por outro caminho: SAT x booleana);
  3. todo nome do SRDF existe no URDF; juntas dos controladores = juntas
     móveis não mimic do URDF; poses nomeadas dentro dos limites e livres;
  4. o xacro expande e o check_urdf aceita.

Uso (de dentro de cad/):  python3 gerar_moveit_config.py
(Python puro + numpy; não precisa do CadQuery nem do MoveIt2.)
"""

import itertools
import json
import math
import os
import random
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
PROJETO = os.path.dirname(AQUI)
DESC = os.path.join(PROJETO, "ros2", "braco2_description")
URDF = os.path.join(DESC, "urdf", "braco.urdf")
CONTROLADORES = os.path.join(DESC, "config", "controladores.yaml")
CAIXAS = os.path.join(PROJETO, "resultados", "caixas_colisao.json")
PKG = os.path.join(PROJETO, "ros2", "braco2_moveit_config")
CFG = os.path.join(PKG, "config")
POSES_DANIEL = os.path.join(CFG, "poses_nomeadas.xml")
ROBO = "braco2"

AMOSTRAS = 10000      # mesmo padrão do Setup Assistant
SEMENTE = 1
# ASSUMIDO: o datasheet não publica aceleração. 90° em 1 s, perfil triangular
#.
ACELERACAO = 6.28     # rad/s²
ACELERACAO_GARRA = 0.0838  # m/s²: 20 mm em 1 s, perfil triangular (4·0,02/1²)
FATOR_VEL = 0.5       # o planejador usa no máximo metade da velocidade sem carga

sys.path.insert(0, os.path.join(PROJETO, "ros2", "braco2_nos"))
from braco2_nos import cinematica as cin  # noqa: E402

FALHAS = []


def conferir(nome, ok, info=""):
    print(f"  [{'OK' if ok else 'FALHOU'}] {nome}" + (f": {info}" if info else ""))
    if not ok:
        FALHAS.append(nome)
    return ok


# ----------------------------------------------------------------------------
# URDF -> árvore de juntas e cinemática direta (independente do CAD)
# ----------------------------------------------------------------------------

def ler_urdf():
    raiz = ET.parse(URDF).getroot()
    juntas = []
    for jt in raiz.findall("joint"):
        o = jt.find("origin")
        rpy = [float(v) for v in o.get("rpy", "0 0 0").split()]
        assert rpy == [0, 0, 0], f"{jt.get('name')}: rpy != 0 não suportado aqui"
        lim = jt.find("limit")
        mim = jt.find("mimic")
        juntas.append({
            "nome": jt.get("name"), "tipo": jt.get("type"),
            "pai": jt.find("parent").get("link"), "filho": jt.find("child").get("link"),
            "xyz": np.array([float(v) for v in o.get("xyz").split()]),
            "eixo": np.array([float(v) for v in jt.find("axis").get("xyz").split()]) if jt.find("axis") is not None else None,
            "lim": (float(lim.get("lower")), float(lim.get("upper"))) if lim is not None else None,
            "vel": float(lim.get("velocity")) if lim is not None else None,
            "mimic": mim.get("joint") if mim is not None else None,
        })
    links = {l.get("name"): l.find("collision") is not None for l in raiz.findall("link")}
    return raiz.get("name"), juntas, links


def rot(eixo, a):
    x, y, z = eixo
    c, s, v = math.cos(a), math.sin(a), 1 - math.cos(a)
    return np.array([[c + x * x * v, x * y * v - z * s, x * z * v + y * s],
                     [y * x * v + z * s, c + y * y * v, y * z * v - x * s],
                     [z * x * v - y * s, z * y * v + x * s, c + z * z * v]])


def fk(juntas, q):
    """{link: (R, p)} no mundo, em m. q = {junta: valor} (mimic copia a mestra)."""
    mundo = {"world": (np.eye(3), np.zeros(3))}
    for jt in juntas:          # o URDF gerado lista as juntas da raiz para as pontas
        R, p = mundo[jt["pai"]]
        p2 = p + R @ jt["xyz"]
        v = q.get(jt["mimic"] or jt["nome"], 0.0)
        if jt["tipo"] == "revolute":
            R2 = R @ rot(jt["eixo"], v)
        elif jt["tipo"] == "prismatic":
            R2, p2 = R, p2 + R @ (jt["eixo"] * v)
        else:
            R2 = R
        mundo[jt["filho"]] = (R2, p2)
    return mundo


# ----------------------------------------------------------------------------
# Colisão entre caixas orientadas (teste dos eixos separadores, SAT)
# ----------------------------------------------------------------------------

def caixas_no_mundo(caixas, mundo):
    """{link: [(centro, R, meias_medidas)]} em m."""
    out = {}
    for link, lista in caixas.items():
        R, p = mundo[link]
        out[link] = []
        for x0, y0, z0, x1, y1, z1 in lista:
            c = np.array([x0 + x1, y0 + y1, z0 + z1]) / 2000.0
            h = np.array([x1 - x0, y1 - y0, z1 - z0]) / 2000.0
            out[link].append((p + R @ c, R, h))
    return out


def sobrepoe(a, b, folga=1e-7):
    """True se as duas caixas se interpenetram (mais que `folga` m em todo eixo)."""
    ca, Ra, ha = a
    cb, Rb, hb = b
    t = cb - ca
    if np.linalg.norm(t) > np.linalg.norm(ha) + np.linalg.norm(hb):    # esferas envolventes separadas
        return False
    eixos = [Ra[:, i] for i in range(3)] + [Rb[:, i] for i in range(3)]
    eixos += [np.cross(Ra[:, i], Rb[:, k]) for i in range(3) for k in range(3)]
    for e in eixos:
        n = np.linalg.norm(e)
        if n < 1e-9:
            continue
        e = e / n
        ra = sum(ha[i] * abs(Ra[:, i] @ e) for i in range(3))
        rb = sum(hb[i] * abs(Rb[:, i] @ e) for i in range(3))
        if abs(t @ e) > ra + rb - folga:
            return False
    return True


def links_colidem(cm, a, b):
    return any(sobrepoe(x, y) for x in cm[a] for y in cm[b])


def pares_em_colisao(juntas, caixas, q, pares):
    cm = caixas_no_mundo(caixas, fk(juntas, q))
    return [(a, b) for a, b in pares if links_colidem(cm, a, b)]


# ----------------------------------------------------------------------------
# Geração
# ----------------------------------------------------------------------------

def matriz_de_colisao(juntas, caixas, pares):
    moveis = [jt for jt in juntas if jt["tipo"] in ("revolute", "prismatic") and not jt["mimic"]]
    rng = random.Random(SEMENTE)
    contagem = {p: 0 for p in pares}
    for _ in range(AMOSTRAS):
        q = {jt["nome"]: rng.uniform(*jt["lim"]) for jt in moveis}
        for p in pares_em_colisao(juntas, caixas, q, pares):
            contagem[p] += 1
    return contagem


def ler_poses_daniel():
    """<group_state> escritos pelo Daniel. Devolve [] se o arquivo não existe ainda."""
    if not os.path.exists(POSES_DANIEL):
        return []
    return ET.parse(POSES_DANIEL).getroot().findall("group_state")


def srdf(adjacentes, acoplados, nunca, poses, grupos):
    o = ['<?xml version="1.0" encoding="UTF-8"?>',
         '<!-- ARQUIVO GERADO por cad/gerar_moveit_config.py: não editar à mão.',
         '     As poses nomeadas vêm de config/poses_nomeadas.xml (escritas pelo Daniel). -->',
         f'<robot name="{ROBO}">',
         '',
         '  <!-- Grupo de planejamento do braço: a cadeia da base até o TCP. -->',
         '  <group name="braco">',
         '    <chain base_link="base_link" tip_link="tcp"/>',
         '  </group>',
         '',
         '  <!-- A garra: só a junta comandada; a espelho (mimic) vem junto sozinha. -->',
         '  <group name="garra">',
         f'    <joint name="{grupos["garra"][0]}"/>',
         '  </group>',
         '',
         '  <end_effector name="garra_ee" parent_link="tcp" group="garra" parent_group="braco"/>',
         '']
    if poses:
        o.append('  <!-- Poses nomeadas (Daniel) -->')
        for gs in poses:
            o.append("  " + ET.tostring(gs, encoding="unicode").strip())
        o.append('')
    o.append('  <!-- Pares que o planejador não precisa checar. Os demais PODEM colidir')
    o.append(f'       e continuam sendo checados ({AMOSTRAS} poses sorteadas, caixas de colisão). -->')
    for a, b in adjacentes:
        o.append(f'  <disable_collisions link1="{a}" link2="{b}" reason="Adjacent"/>')
    for a, b in acoplados:
        o.append(f'  <!-- {a} x {b}: mesma junta (mimic); fechados se encostam de propósito -->')
        o.append(f'  <disable_collisions link1="{a}" link2="{b}" reason="Default"/>')
    for a, b in nunca:
        o.append(f'  <disable_collisions link1="{a}" link2="{b}" reason="Never"/>')
    o.append('</robot>')
    return "\n".join(o) + "\n"


def urdf_xacro():
    return f"""<?xml version="1.0"?>
<!-- ARQUIVO GERADO por cad/gerar_moveit_config.py: não editar à mão.
     O braço (braco.urdf, sem mudança) + o bloco ros2_control com hardware
     FALSO. É o robot_description que o MoveIt2 carrega no teste sem física. -->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="{ROBO}">
  <xacro:arg name="initial_positions_file" default="initial_positions.yaml"/>

  <xacro:include filename="$(find braco2_description)/urdf/braco.urdf"/>
  <xacro:include filename="{ROBO}.ros2_control.xacro"/>

  <xacro:{ROBO}_ros2_control name="HardwareFalso" initial_positions_file="$(arg initial_positions_file)"/>
</robot>
"""


def ros2_control_xacro(comandadas, mimicas):
    linhas = []
    for nome in comandadas:
        linhas += [f'      <joint name="{nome}">',
                   '        <command_interface name="position"/>',
                   '        <state_interface name="position">',
                   f'          <param name="initial_value">${{initial_positions[\'{nome}\']}}</param>',
                   '        </state_interface>',
                   '        <state_interface name="velocity"/>',
                   '      </joint>']
    for nome in mimicas:
        linhas += [f'      <!-- Mimic: copia a mestra (tag <mimic> do braco.urdf); só se lê. -->',
                   f'      <joint name="{nome}">',
                   '        <state_interface name="position"/>',
                   '        <state_interface name="velocity"/>',
                   '      </joint>']
    corpo = "\n".join(linhas)
    return f"""<?xml version="1.0"?>
<!-- ARQUIVO GERADO por cad/gerar_moveit_config.py: não editar à mão.
     Hardware FALSO (mock_components): devolve como estado exatamente o que foi
     comandado, sem física. Serve para configurar e conferir o MoveIt2 sozinho
     (Decisões, 2026-10-14: hardware falso primeiro, Gazebo depois). No Gazebo
     este bloco é o do braco_gazebo.urdf.xacro; no braço real, o driver dos
     DYNAMIXEL. Mesmas juntas e interfaces nos três. -->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="{ROBO}_ros2_control" params="name initial_positions_file">
    <xacro:property name="initial_positions"
                    value="${{xacro.load_yaml(initial_positions_file)['initial_positions']}}"/>
    <ros2_control name="${{name}}" type="system">
      <hardware>
        <plugin>mock_components/GenericSystem</plugin>
      </hardware>
{corpo}
    </ros2_control>
  </xacro:macro>
</robot>
"""


def ler_controladores():
    """{controlador: [juntas]} do controladores.yaml do Daniel (sem depender de PyYAML)."""
    out, atual, em_juntas = {}, None, False
    for linha in open(CONTROLADORES, encoding="utf-8"):
        s = linha.split("#")[0].rstrip()
        if not s.strip():
            continue
        recuo = len(s) - len(s.lstrip())
        if recuo == 0:
            atual, em_juntas = s.strip().rstrip(":"), False
        elif s.strip() == "joints:":
            em_juntas = True
            out[atual] = []
        elif em_juntas and s.strip().startswith("- "):
            out[atual].append(s.strip()[2:])
        else:
            em_juntas = False
    return out


def moveit_controllers(ctrl):
    o = ["# ARQUIVO GERADO por cad/gerar_moveit_config.py: não editar à mão.",
         "# Diz ao MoveIt2 para quais controladores do ros2_control mandar a trajetória",
         "# planejada. Nomes e juntas lidos do controladores.yaml (Daniel).",
         "moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager",
         "", "moveit_simple_controller_manager:", "  controller_names:"]
    o += [f"    - {c}" for c in ctrl]
    for c, js in ctrl.items():
        o += ["", f"  {c}:", "    type: FollowJointTrajectory", "    action_ns: follow_joint_trajectory",
              "    default: true", "    joints:"] + [f"      - {jn}" for jn in js]
    return "\n".join(o) + "\n"


def joint_limits(comandadas, juntas):
    por_nome = {jt["nome"]: jt for jt in juntas}
    o = ["# ARQUIVO GERADO por cad/gerar_moveit_config.py: não editar à mão.",
         "# Limites usados pelo MoveIt2 para dar TEMPO à trajetória. Os limites de",
         "# POSIÇÃO vêm do URDF e não se repetem aqui.",
         "# max_velocity = velocity do URDF (datasheet ROBOTIS, sem carga, 12 V).",
         f"# O fator {FATOR_VEL} deixa o planejador usar no máximo metade: sob carga o servo",
         "# não chega na velocidade sem carga.",
         f"# max_acceleration ASSUMIDO (o datasheet não publica): juntas {ACELERACAO} rad/s²",
         f"# (90° em 1 s, perfil triangular); garra {ACELERACAO_GARRA} m/s² (20 mm em 1 s).",
         f"default_velocity_scaling_factor: {FATOR_VEL}",
         f"default_acceleration_scaling_factor: {FATOR_VEL}", "", "joint_limits:"]
    for nome in comandadas:
        jt = por_nome[nome]
        acc = ACELERACAO_GARRA if jt["tipo"] == "prismatic" else ACELERACAO
        o += [f"  {nome}:", "    has_velocity_limits: true", f"    max_velocity: {jt['vel']}",
              "    has_acceleration_limits: true", f"    max_acceleration: {acc}"]
    return "\n".join(o) + "\n"


def pilz(juntas):
    por_nome = {jt["nome"]: jt for jt in juntas}
    alcance = cin.L2 + cin.L3 + cin.L4          # m, braço esticado: a ponta anda mais rápido
    w = FATOR_VEL * por_nome["junta_ombro"]["vel"]
    return f"""# ARQUIVO GERADO por cad/gerar_moveit_config.py: não editar à mão.
# Limites da PONTA para o planejador Pilz (linhas retas e arcos). O
# MoveItConfigsBuilder exige o arquivo mesmo sem usar o Pilz.
# Derivados dos limites das juntas com o braço esticado ({alcance * 1000:.1f} mm):
#   velocidade: {FATOR_VEL} x {por_nome['junta_ombro']['vel']} rad/s x {alcance:.4f} m
#   aceleração: {ACELERACAO} rad/s² (ASSUMIDO) x {alcance:.4f} m
cartesian_limits:
  max_trans_vel: {w * alcance:.3f}
  max_trans_acc: {ACELERACAO * alcance:.3f}
  max_trans_dec: {-ACELERACAO * alcance:.3f}
  max_rot_vel: {w:.3f}
"""


KINEMATICS = """# ARQUIVO GERADO por cad/gerar_moveit_config.py: não editar à mão.
# Solver de IK do MoveIt2 para o grupo "braco". Com 4 juntas, o KDL só
# resolve em modo posição (acerta x, y, z; a inclinação da garra sai qualquer
# uma). Por isso o pick and place calcula as juntas com a
# IK analítica do braco2_nos (cinematica.py) e manda ao MoveIt2 um alvo em
# JUNTAS. Este solver serve só para o arraste da garra no RViz2.
braco:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05
  position_only_ik: true
"""


# ----------------------------------------------------------------------------
# Verificações
# ----------------------------------------------------------------------------

def verificar_fk(juntas):
    print("\n1. Cinemática lida do URDF x cinematica.fk_analitica (pacote de nós), 200 poses sorteadas")
    rng = random.Random(SEMENTE)
    erro = 0.0
    for _ in range(200):
        qs = [rng.uniform(*cin.LIMITES[n]) for n in cin.NOMES]
        p = fk(juntas, dict(zip(cin.NOMES, qs)))["tcp"][1]
        erro = max(erro, np.linalg.norm(p - np.array(cin.fk_analitica(*qs)[:3])) * 1000)
    conferir("TCP pelo URDF = TCP pela fórmula", erro < 1e-6, f"erro máx. {erro:.2e} mm")


def verificar_poses_trabalho(juntas, caixas, pares):
    print("\n2. Poses de trabalho do montagem.py livres com as caixas (SAT)")
    # Cópia de montagem.POSES_TRABALHO (graus; abertura em mm): o montagem.py importa CadQuery
    poses = {"zero": (0, 0, 0, 0, 40), "pronto": (0, 60, -85, -65, 40),
             "pegar_na_mesa": (20, 40, -70, -60, 20), "alto": (-30, 70, -40, -20, 40)}
    for nome, (q1, q2, q3, q4, ab) in poses.items():
        q = dict(zip(["junta_base", "junta_ombro", "junta_cotovelo", "junta_punho"],
                     map(math.radians, (q1, q2, q3, q4))))
        q["junta_garra"] = ab / 2000.0
        bate = pares_em_colisao(juntas, caixas, q, pares)
        conferir(f"{nome}", not bate, str(bate) if bate else "")


def verificar_nomes(texto, juntas, links, ctrl, poses, caixas, pares):
    print("\n3. Nomes, controladores e poses nomeadas")
    jn = {jt["nome"]: jt for jt in juntas}
    s = ET.fromstring(texto)
    citados_l = {el.get(a) for el in s.iter() for a in ("link1", "link2", "base_link", "tip_link", "parent_link") if el.get(a)}
    citadas_j = {el.get("name") for el in s.iter() if el.tag == "joint"}
    conferir("links e juntas do SRDF existem no URDF", citados_l <= set(links) and citadas_j <= set(jn),
             str(sorted((citados_l - set(links)) | (citadas_j - set(jn)))))
    moveis = {n for n, jt in jn.items() if jt["tipo"] in ("revolute", "prismatic") and not jt["mimic"]}
    nos_ctrl = [x for js in ctrl.values() for x in js]
    conferir("juntas dos controladores = juntas móveis não mimic do URDF",
             sorted(nos_ctrl) == sorted(moveis), f"{sorted(nos_ctrl)}")
    if not poses:
        print("  [PENDENTE] config/poses_nomeadas.xml ainda não existe (Daniel): SRDF sem poses nomeadas")
        return
    grupos = {"braco": ["junta_base", "junta_ombro", "junta_cotovelo", "junta_punho"], "garra": ["junta_garra"]}
    for gs in poses:
        nome, grupo = gs.get("name"), gs.get("group")
        vals = {j.get("name"): float(j.get("value")) for j in gs.findall("joint")}
        if not conferir(f"pose '{nome}': grupo e juntas", grupo in grupos and sorted(vals) == sorted(grupos.get(grupo, [])),
                        f"grupo {grupo}, juntas {sorted(vals)}"):
            continue
        # o URDF arredonda os limites para dentro (5 casas)
        fora = [f"{k} = {v} fora de {jn[k]['lim']}" for k, v in vals.items()
                if not jn[k]["lim"][0] <= v <= jn[k]["lim"][1]]
        conferir(f"pose '{nome}': dentro dos limites do URDF", not fora, "; ".join(fora))
        q = dict(vals)
        bate = pares_em_colisao(juntas, caixas, q, pares)
        conferir(f"pose '{nome}': sem colisão entre elos", not bate, str(bate) if bate else "")


def verificar_xacro(pasta):
    print("\n4. xacro expande e check_urdf aceita")
    with tempfile.TemporaryDirectory() as tmp:
        # O $(find braco2_description) precisa do pacote instalado; aqui, um prefixo
        # temporário que aponta para a pasta do pacote (sem compilar nada).
        idx = os.path.join(tmp, "share", "ament_index", "resource_index", "packages")
        os.makedirs(idx)
        open(os.path.join(idx, "braco2_description"), "w").close()
        os.symlink(DESC, os.path.join(tmp, "share", "braco2_description"))
        saida = os.path.join(tmp, "braco2.urdf")
        cmd = (f"source /opt/ros/jazzy/setup.bash && export AMENT_PREFIX_PATH='{tmp}':$AMENT_PREFIX_PATH && "
               f"cd '{pasta}' && xacro {ROBO}.urdf.xacro initial_positions_file:=initial_positions.yaml > '{saida}' "
               f"&& check_urdf '{saida}'")
        r = subprocess.run(["bash", "-c", cmd], capture_output=True, text=True)
        conferir("xacro + check_urdf", r.returncode == 0 and "Successfully Parsed" in r.stdout,
                 (r.stderr or r.stdout).strip().splitlines()[-1] if r.returncode else "")
        if r.returncode == 0:
            texto = open(saida, encoding="utf-8").read()
            conferir("<ros2_control> com mock_components no robot_description",
                     "mock_components/GenericSystem" in texto)


# ----------------------------------------------------------------------------

def main():
    robo, juntas, links = ler_urdf()
    assert robo == ROBO, robo
    with open(CAIXAS, encoding="utf-8") as f:
        caixas = json.load(f)["caixas"]
    assert set(caixas) == {n for n, tem in links.items() if tem}, "caixas_colisao.json desatualizado: rodar malhas_urdf.py"
    adjacentes = [(jt["pai"], jt["filho"]) for jt in juntas if jt["pai"] in caixas and jt["filho"] in caixas]
    # Os dois mordentes: a mimic os prende à MESMA junta, então a distância entre eles
    # só depende da abertura, e fechada eles se encostam DE PROPÓSITO (com a margem de
    # 1 mm de cada caixa, a sobreposição começa abaixo de ~2 mm de abertura). O Setup
    # Assistant chama isso de "Default" (colide na pose padrão) e também desliga.
    acoplados = [(a["filho"], b["filho"]) for a, b in itertools.combinations(juntas, 2)
                 if a["mimic"] == b["nome"] or b["mimic"] == a["nome"]]
    pares = [p for p in itertools.combinations(caixas, 2)
             if not {p, p[::-1]} & set(adjacentes + acoplados)]
    ctrl = {c: js for c, js in ler_controladores().items() if c != "controller_manager"}
    comandadas = [x for js in ctrl.values() for x in js]
    mimicas = [jt["nome"] for jt in juntas if jt["mimic"]]

    verificar_fk(juntas)
    verificar_poses_trabalho(juntas, caixas, pares)

    print(f"\nMatriz de colisão: {AMOSTRAS} poses sorteadas, {len(pares)} pares não adjacentes")
    contagem = matriz_de_colisao(juntas, caixas, pares)
    for (a, b), n in sorted(contagem.items(), key=lambda kv: -kv[1]):
        print(f"  {a:15s} x {b:15s} {n:5d} colisões ({100 * n / AMOSTRAS:.1f} %)")
    nunca = [p for p, n in contagem.items() if n == 0]
    zero = {jt["nome"]: 0.0 for jt in juntas}
    padrao = pares_em_colisao(juntas, caixas, zero, pares)
    conferir("pose zero (garra fechada) sem colisão entre elos", not padrao, str(padrao) if padrao else "")

    poses = ler_poses_daniel()
    texto = srdf(adjacentes, acoplados, nunca, poses, ctrl and {"garra": ctrl["garra_controller"]})
    verificar_nomes(texto, juntas, {n for n in links}, ctrl, poses, caixas, pares)

    arquivos = {
        f"{ROBO}.srdf": texto,
        f"{ROBO}.urdf.xacro": urdf_xacro(),
        f"{ROBO}.ros2_control.xacro": ros2_control_xacro(comandadas, mimicas),
        "initial_positions.yaml": "# ARQUIVO GERADO por cad/gerar_moveit_config.py: pose em que o hardware\n"
                                  "# falso começa (braço esticado, garra fechada).\ninitial_positions:\n"
                                  + "".join(f"  {n}: 0.0\n" for n in comandadas),
        "joint_limits.yaml": joint_limits(comandadas, juntas),
        "moveit_controllers.yaml": moveit_controllers(ctrl),
        "ros2_controllers.yaml": "# ARQUIVO GERADO por cad/gerar_moveit_config.py: CÓPIA de\n"
                                 "# braco2_description/config/controladores.yaml (Daniel). Mudar lá e gerar de novo.\n"
                                 + open(CONTROLADORES, encoding="utf-8").read(),
        "pilz_cartesian_limits.yaml": pilz(juntas),
        "kinematics.yaml": KINEMATICS,
    }
    with tempfile.TemporaryDirectory() as tmp:
        for nome, conteudo in arquivos.items():
            with open(os.path.join(tmp, nome), "w", encoding="utf-8") as f:
                f.write(conteudo)
        verificar_xacro(tmp)

    if FALHAS:
        sys.exit(f"\n{len(FALHAS)} verificação(ões) falharam: nada foi gravado.")
    os.makedirs(CFG, exist_ok=True)
    for nome, conteudo in arquivos.items():
        with open(os.path.join(CFG, nome), "w", encoding="utf-8") as f:
            f.write(conteudo)
    print(f"\nGravados em {os.path.relpath(CFG, PROJETO)}/: {', '.join(arquivos)}")
    print(f"Colisão desligada: {len(adjacentes)} pares adjacentes + {len(acoplados)} acoplados {acoplados}"
          f" + {len(nunca)} que nunca colidem {nunca}")


if __name__ == "__main__":
    main()
