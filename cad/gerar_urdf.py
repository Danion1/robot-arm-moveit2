#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gerar_urdf.py
=============
Escreve ros2/braco2_description/urdf/braco.urdf a partir do que os outros
scripts já calcularam, sem número digitado à mão aqui:

  - origens das juntas, frames e malhas .... malhas_urdf.py (ORIGEM, PAI)
  - massa, centro de massa e inércia ....... resultados/massas.json (massa.py)
  - limites de ângulo ....................... base.py (ombro para baixo) e a faixa
                                              de ±90° verificada em elo.py/garra.py
  - effort e velocity ....................... dados_servos_dynamixel (datasheet, 12 V)
  - curso e raio do pinhão da garra ......... garra.py
  - TCP ..................................... montagem.py (L4)

Por que gerado (decisão do Daniel, 2026-10-13): ele escreveu à mão o world e o
base_link para aprender a ler um URDF; o resto é cópia de números (7 elos,
7 tensores), onde o erro entraria. O trecho dele é conferido aqui: o arquivo
gerado tem que concordar com a versão manual (verificação 1).

Diferenças: colisão em malha separada (D1), garra com 2 prismáticas (mimic),
limites pelas verificações do CAD em vez de varredura de colisão.

Verificações (param antes de gravar):
  1. world + base_link iguais aos da versão escrita à mão (se ela existir e abrir);
  2. o URDF lido de volta (render_pelo_urdf.ler_urdf/transformadas, da biblioteca)
     leva o TCP ao mesmo ponto da fórmula do montagem.py, em várias poses;
  3. massas e tensores do arquivo = massas.json; soma = massa total;
  4. poses de trabalho do montagem.py dentro dos limites;
  5. check_urdf (ROS 2) aceita o arquivo.
  7. colisão: uma <box> por caixa do caixas_colisao.json, com as mesmas medidas;
  6. braco_gazebo.urdf.xacro (o mesmo braço + <ros2_control> + plugin do Gazebo):
     o xacro expande, o check_urdf aceita, e o bloco <ros2_control> cobre
     exatamente as juntas móveis, com comando de POSIÇÃO (Decisões, 2026-10-14)
     e a mimic sem comando.
Depois: previews/urdf_render.png, montado pelo URDF e pelos STL do pacote.

Uso (de dentro de cad/):  python gerar_urdf.py
"""

import contextlib
import io
import json
import math
import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import usar_lib  # noqa: F401
from check_bbox import exigir_tudo_ok
import dados_servos_dynamixel as sv
import render_pelo_urdf as ru

import base as b
import garra as ga
import junta as j
import malhas_urdf as mu
import montagem as mo
import servo_step

PACOTE = "braco2_description"
PKG = os.path.join(j.PROJETO, "ros2", PACOTE)
URDF = os.path.join(PKG, "urdf", "braco.urdf")
URDF_GZ = os.path.join(PKG, "urdf", "braco_gazebo.urdf.xacro")
MM = 0.001

CORES = {"base_link": ("cinza", "0.55 0.55 0.58 1.0"), "link1": ("azul_escuro", "0.30 0.30 0.75 1.0"),
         "link2": ("azul", "0.20 0.45 0.80 1.0"), "link3": ("verde", "0.20 0.65 0.40 1.0"),
         "link4": ("laranja", "0.95 0.55 0.15 1.0"), "mordente_cima": ("vermelho", "0.85 0.20 0.20 1.0"),
         "mordente_baixo": ("vermelho", "0.85 0.20 0.20 1.0")}

# (nome, filho, eixo, servo, descrição); o pai e a origem vêm de malhas_urdf
REVOLUTAS = [
    ("junta_base", "link1", (0, 0, 1), "XM430-W350-T", "base: gira em torno de Z"),
    ("junta_ombro", "link2", (0, -1, 0), "XM430-W350-T", "ombro: positivo levanta o braço"),
    ("junta_cotovelo", "link3", (0, -1, 0), "XM430-W350-T", "cotovelo: positivo levanta o antebraço"),
    ("junta_punho", "link4", (0, -1, 0), "XC430-W240-T", "punho: positivo levanta a mão"),
]


def limites_graus(servo):
    """{junta: (min, max)} em graus. Ombro para baixo: último ângulo livre da
    varredura do base.py (passo de 2°) antes de bater na torre."""
    with contextlib.redirect_stdout(io.StringIO()):
        _, baixo = b.verificar(b.montar(servo), servo)
    return {"junta_base": (-180.0, 180.0),             # servo 0..360°; torre gira 360° livre (base.py, 5)
            "junta_ombro": (-(baixo - 2.0), 90.0),     # base.py, 6
            "junta_cotovelo": (-90.0, 90.0),           # faixa verificada (elo.py)
            "junta_punho": (-90.0, 90.0)}              # faixa verificada (elo.py, garra.py)


def para_dentro(graus):
    lo, hi = math.radians(graus[0]), math.radians(graus[1])
    return math.ceil(lo * 1e5) / 1e5, math.floor(hi * 1e5) / 1e5      # para dentro: valor cheio fica fora do limite


def fmt(v):
    s = f"{v:.6f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def colisao_em_caixas(lista):
    """Uma <collision> por caixa de resultados/caixas_colisao.json (mm, frame do elo).

    Antes era um STL só com a união das caixas (D1). Com o motor Bullet-Featherstone
    (mundo do pick and place, por causa da mimic) a malha vira o seu FECHO CONVEXO,
    que fechava o vão entre os dedos (a cremalheira atravessa o centro da garra) e
    levantava o objeto (2026-10-14). Caixa primitiva é exata em qualquer motor e é
    o mesmo modelo que o gerar_moveit_config.py usa (SAT). Mantém o espírito da D1:
    caixa POR PEÇA, nada de contorno convexo do elo."""
    out = []
    for x0, y0, z0, x1, y1, z1 in lista:
        c = [(x0 + x1) / 2 * MM, (y0 + y1) / 2 * MM, (z0 + z1) / 2 * MM]
        t = [(x1 - x0) * MM, (y1 - y0) * MM, (z1 - z0) * MM]
        out += ['    <collision>',
                f'      <origin xyz="{fmt(c[0])} {fmt(c[1])} {fmt(c[2])}" rpy="0 0 0"/>',
                f'      <geometry><box size="{fmt(t[0])} {fmt(t[1])} {fmt(t[2])}"/></geometry>',
                '    </collision>']
    return out


def gerar(massas, limites, caixas):
    o = ['<?xml version="1.0"?>',
         '<!-- ARQUIVO GERADO por cad/gerar_urdf.py: não editar à mão; mude o CAD ou os',
         '     dados e gere de novo. O trecho world + base_link foi escrito primeiro à',
         '     mão pelo Daniel (commit no Git) e é conferido pelo gerador. -->',
         '<robot name="braco2">', '',
         '  <!-- Pose zero: braço esticado na horizontal, apontando para +X.',
         '       +X frente, +Y esquerda, +Z cima (REP-103); base_link na mesa, sob o eixo da base.',
         '       Malhas no frame da PRÓPRIA junta (origin 0 0 0) e em mm (scale 0.001).',
         '       Visual = peças detalhadas; colisão = uma <box> por peça (decisão D1; caixas_colisao.json). -->', '']
    for nome, rgba in dict(CORES.values()).items():
        o.append(f'  <material name="{nome}"><color rgba="{rgba}"/></material>')
    o += ['', '  <!-- world: link vazio que segura a base (o braço é fixado na mesa). O KDL',
          '       não aceita inércia no link raiz, e o MoveIt2 precisa de um frame fixo. -->',
          '  <link name="world"/>',
          '  <joint name="world_to_base" type="fixed">',
          '    <parent link="world"/>', '    <child link="base_link"/>',
          '    <origin xyz="0 0 0" rpy="0 0 0"/>', '  </joint>', '']

    for link in mu.ORIGEM:
        d = massas[link]
        c, t = d["com_m"], d["tensor_kg_m2"]
        mesh = lambda arq: (f'<mesh filename="package://{PACOTE}/meshes/{arq}.stl" '
                            f'scale="0.001 0.001 0.001"/>')
        o += [f'  <!-- ===================== {link} ===================== -->',
              f'  <link name="{link}">',
              '    <visual>', '      <origin xyz="0 0 0" rpy="0 0 0"/>',
              f'      <geometry>{mesh(link)}</geometry>',
              f'      <material name="{CORES[link][0]}"/>', '    </visual>',
              *colisao_em_caixas(caixas[link]),
              '    <inertial>',
              f'      <origin xyz="{c[0]:.6f} {c[1]:.6f} {c[2]:.6f}" rpy="0 0 0"/>',
              f'      <mass value="{d["massa_kg"]:.6f}"/>',
              f'      <inertia ixx="{t[0][0]:.8e}" ixy="{t[0][1]:.8e}" ixz="{t[0][2]:.8e}" '
              f'iyy="{t[1][1]:.8e}" iyz="{t[1][2]:.8e}" izz="{t[2][2]:.8e}"/>',
              '    </inertial>', '  </link>', '']

    def origem(filho):
        d = [(a - p) * MM for a, p in zip(mu.ORIGEM[filho], mu.ORIGEM[mu.PAI[filho]])]
        return f'<origin xyz="{fmt(d[0])} {fmt(d[1])} {fmt(d[2])}" rpy="0 0 0"/>'

    for nome, filho, eixo, modelo, desc in REVOLUTAS:
        s = sv.SERVOS[modelo]
        lo, hi = para_dentro(limites[nome])
        o += [f'  <!-- {desc}. Servo {modelo}: effort = stall a 12 V ({s["torque_stall_nm"]} N.m),',
              f'       velocity = {s["vel_sem_carga_rpm"]:g} rev/min sem carga. Limites: '
              f'{limites[nome][0]:+.0f} a {limites[nome][1]:+.0f} graus. -->',
              f'  <joint name="{nome}" type="revolute">',
              f'    <parent link="{mu.PAI[filho]}"/>', f'    <child link="{filho}"/>',
              f'    {origem(filho)}', f'    <axis xyz="{eixo[0]} {eixo[1]} {eixo[2]}"/>',
              f'    <limit lower="{lo:.5f}" upper="{hi:.5f}" effort="{s["torque_stall_nm"]}" '
              f'velocity="{sv.vel_rad_s(modelo):.3f}"/>', '  </joint>', '']

    # Garra: o pinhão (raio primitivo R) empurra as duas cremalheiras; cada uma
    # leva metade do torque: F = stall / (2 R). Velocidade linear = ω · R.
    xc = sv.SERVOS["XC430-W240-T"]
    forca = xc["torque_stall_nm"] / (2 * ga.R * MM)
    vel = sv.vel_rad_s("XC430-W240-T") * ga.R * MM
    curso = ga.ABERTURA / 2 * MM
    o += [f'  <!-- Garra: pinhão m{ga.M_ENG:g} z{ga.Z_ENG} (R = {ga.R:g} mm) no XC430-W240-T. Cada mordente anda',
          f'       0 a {curso * 1000:g} mm (0 = fechada); effort = stall / (2 R) = {forca:.1f} N,',
          f'       velocity = ω·R. O de baixo copia o de cima (mimic): no robô real é o mesmo',
          '       pinhão, então só junta_garra é comandada. -->']
    for nome, filho, eixo, mimic in (("junta_garra", "mordente_cima", (0, 1, 0), False),
                                      ("junta_garra_espelho", "mordente_baixo", (0, -1, 0), True)):
        o += [f'  <joint name="{nome}" type="prismatic">',
              f'    <parent link="link4"/>', f'    <child link="{filho}"/>',
              f'    {origem(filho)}', f'    <axis xyz="{eixo[0]} {eixo[1]} {eixo[2]}"/>',
              f'    <limit lower="0" upper="{fmt(curso)}" effort="{forca:.1f}" velocity="{vel:.4f}"/>']
        if mimic:
            o.append('    <mimic joint="junta_garra" multiplier="1" offset="0"/>')
        o += ['  </joint>', '']

    o += ['  <!-- TCP: frame sem geometria entre os dedos, no meio do comprimento deles.',
          f'       É o ponto que a cinemática inversa posiciona (L4 = {mo.L4:g} mm do punho). -->',
          '  <link name="tcp"/>', '  <joint name="junta_tcp" type="fixed">',
          '    <parent link="link4"/>', '    <child link="tcp"/>',
          f'    <origin xyz="{fmt(mo.L4 * MM)} 0 0" rpy="0 0 0"/>', '  </joint>', '', '</robot>']
    return "\n".join(o) + "\n"


def gerar_gazebo():
    """braco_gazebo.urdf.xacro: inclui o braco.urdf e acrescenta só o que é de
    SIMULAÇÃO. O braco.urdf fica sendo a descrição pura (RViz2, testes da IK);
    no robô real, troca-se este arquivo, não a descrição."""
    juntas = [nome for nome, *_ in REVOLUTAS] + ["junta_garra"]
    o = ['<?xml version="1.0"?>',
         '<!-- ARQUIVO GERADO por cad/gerar_urdf.py: não editar à mão. -->',
         '<robot name="braco2" xmlns:xacro="http://www.ros.org/wiki/xacro">', '',
         '  <!-- Caminho do .yaml dos controladores. O padrão aponta para o pacote',
         '       instalado; a verificação do gerador passa outro caminho. -->',
         '  <xacro:arg name="controladores" default="$(find braco2_description)/config/controladores.yaml"/>', '',
         '  <!-- A descrição do braço, sem mudança (mesma pasta). -->',
         '  <xacro:include filename="braco.urdf"/>', '',
         '  <!-- ros2_control: diz QUAIS juntas o controller_manager comanda e COMO.',
         '       hardware = o simulador (GazeboSimSystem); no braço real, aqui entraria',
         '       o driver dos DYNAMIXEL, e o resto (controladores, MoveIt2) não muda.',
         '       Comando de POSIÇÃO: é o modo padrão dos dois servos, e o XC430 nem tem',
         '       modo de torque (e-Manual ROBOTIS; Decisões, 2026-10-14).',
         '       Estado: posição, velocidade e ESFORÇO, para medir o torque que cada',
         '       junta faz contra a gravidade e comparar com o stall. -->',
         '  <ros2_control name="braco2_gazebo" type="system">',
         '    <hardware>', '      <plugin>gz_ros2_control/GazeboSimSystem</plugin>', '    </hardware>']
    for nome in juntas:
        o += [f'    <joint name="{nome}">', '      <command_interface name="position"/>',
              '      <state_interface name="position"/>', '      <state_interface name="velocity"/>',
              '      <state_interface name="effort"/>', '    </joint>']
    o += ['    <!-- Mimic: copia junta_garra (tag <mimic> do braco.urdf); só se lê, não se comanda. -->',
          '    <joint name="junta_garra_espelho">', '      <state_interface name="position"/>',
          '      <state_interface name="velocity"/>', '    </joint>',
          '  </ros2_control>', '',
          '  <!-- Plugin que liga o Gazebo ao ros2_control: sobe o controller_manager',
          '       dentro do simulador e carrega o .yaml dos controladores. -->',
          '  <gazebo>',
          '    <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">',
          '      <parameters>$(arg controladores)</parameters>',
          '    </plugin>', '  </gazebo>', '', '</robot>']
    return "\n".join(o) + "\n"


def verificar_gazebo(texto_urdf, texto_gz):
    ok = True
    print("\n6. braco_gazebo.urdf.xacro")
    with tempfile.TemporaryDirectory() as d:
        for nome, t in (("braco.urdf", texto_urdf), ("braco_gazebo.urdf.xacro", texto_gz)):
            with open(os.path.join(d, nome), "w", encoding="utf-8") as f:
                f.write(t)
        saida = os.path.join(d, "expandido.urdf")
        r = subprocess.run(["bash", "-c", "source /opt/ros/jazzy/setup.bash && "
                            f"xacro '{d}/braco_gazebo.urdf.xacro' controladores:=/teste.yaml > '{saida}' && "
                            f"check_urdf '{saida}'"], capture_output=True, text=True)
        ok &= j.conferir("xacro expande e check_urdf aceita",
                         r.returncode == 0 and "Successfully Parsed" in r.stdout,
                         (r.stdout + r.stderr).strip().splitlines()[-1] if r.returncode else "")
        if r.returncode:
            return ok
        raiz = ET.parse(saida).getroot()
    moveis = {jt.get("name") for jt in raiz.findall("joint") if jt.get("type") != "fixed"}
    rc = {jt.get("name"): jt for jt in raiz.find("ros2_control").findall("joint")}
    ok &= j.conferir("<ros2_control> cobre exatamente as juntas móveis", set(rc) == moveis,
                     str(set(rc) ^ moveis) * (set(rc) != moveis))
    comandos = {n: [c.get("name") for c in jt.findall("command_interface")] for n, jt in rc.items()}
    mimic = {jt.get("name") for jt in raiz.findall("joint") if jt.find("mimic") is not None}
    ok &= j.conferir("comando de posição em todas, menos na mimic",
                     all(comandos[n] == ([] if n in mimic else ["position"]) for n in rc), str(comandos))
    ok &= j.conferir("esforço legível nas juntas comandadas",
                     all("effort" in [s.get("name") for s in rc[n].findall("state_interface")]
                         for n in rc if n not in mimic), "")
    ok &= j.conferir("plugin do Gazebo aponta para o .yaml passado",
                     raiz.find("gazebo/plugin/parameters").text == "/teste.yaml", "")
    return ok


def elementos(raiz):
    """Dicionário comparável de world, a junta fixa e o base_link."""
    out = {}
    for el in raiz.iter():
        if el.tag in ("link", "joint") and el.get("name") in {"world", "base_link"} | {
                jt.get("name") for jt in raiz.findall("joint") if jt.find("child").get("link") == "base_link"}:
            dentro_colisao = {id(x) for c in el.findall("collision") for x in c.iter()}
            for sub in el.iter():
                if id(sub) in dentro_colisao:      # colisão: conferida na verificação 7
                    continue
                chave = (el.tag, el.get("name") if el.tag == "link" else "junta_fixa", sub.tag)
                if sub.tag in ("mesh", "origin", "mass", "inertia", "parent", "child"):
                    attr = {k: v for k, v in sub.attrib.items() if k != "name"}
                    if sub.tag == "origin":
                        attr.setdefault("rpy", "0 0 0")        # rpy omitido = 0 0 0 (padrão do URDF)
                    out.setdefault(chave, []).append(attr)
    return out


def iguais(a, b):
    try:
        va = [float(x) for x in a.split()]
        vb = [float(x) for x in b.split()]
        return all(abs(x - y) <= 1e-3 * max(abs(x), abs(y)) + 1e-9 for x, y in zip(va, vb)) and len(va) == len(vb)
    except ValueError:
        return a == b


def verificar(texto, massas, limites, caixas):
    ok = True
    print("1. world + base_link = versão escrita à mão")
    try:
        manual = elementos(ET.parse(URDF).getroot())
        gerado = elementos(ET.fromstring(texto))
        dif = [k for k in set(manual) | set(gerado)
               if len(manual.get(k, [])) != len(gerado.get(k, []))
               or any(set(m) != set(g) or not all(iguais(m[x], g[x]) for x in m)
                      for m, g in zip(manual.get(k, []), gerado.get(k, [])))]
        ok &= j.conferir("mesmas malhas, origens, massa e tensor (5 algarismos)", not dif, str(dif) * bool(dif))
    except (ET.ParseError, FileNotFoundError) as erro:
        print(f"  (versão manual ausente ou inválida, não comparada: {erro})")

    print("\n2. TCP lido do URDF (biblioteca) = fórmula do montagem.py")
    with tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False) as f:
        f.write(texto)
    _, juntas = ru.ler_urdf(f.name, {PACOTE: PKG})
    for q in ((0, 0, 0, 0), (30, 20, -45, 10), (-60, 70, -40, -20), (90, -10, 80, 45)):
        ang = {n: math.radians(a) for (n, *_), a in zip(REVOLUTAS, q)}
        m = ru.transformadas(juntas, ang)["tcp"]
        tcp = [m[i][3] / MM for i in range(3)]
        erro = math.dist(tcp, mo.tcp_formula(q))
        ok &= j.conferir(f"q = {q}", erro < 1e-6, f"erro {erro:.1e} mm")

    print("\n3. Inércia no arquivo = massas.json")
    raiz = ET.fromstring(texto)
    total = 0.0
    for link in raiz.findall("link"):
        ine = link.find("inertial")
        if ine is None:
            continue
        d = massas[link.get("name")]
        m = float(ine.find("mass").get("value"))
        i = ine.find("inertia").attrib
        t = d["tensor_kg_m2"]
        ok &= j.conferir(link.get("name"), abs(m - d["massa_kg"]) < 1e-6 and all(
            abs(float(i[k]) - t[a][c]) < 1e-12 for k, (a, c) in
            {"ixx": (0, 0), "ixy": (0, 1), "ixz": (0, 2), "iyy": (1, 1), "iyz": (1, 2), "izz": (2, 2)}.items()),
            f"{m * 1000:.1f} g")
        total += m
    ok &= j.conferir("soma = massa total", abs(total - sum(d["massa_kg"] for d in massas.values())) < 1e-6,
                     f"{total * 1000:.0f} g")

    print("\n4. Poses de trabalho dentro dos limites")
    for nome, pose in mo.POSES_TRABALHO.items():
        fora = [n for (n, *_), a in zip(REVOLUTAS, pose[:4]) if not limites[n][0] <= a <= limites[n][1]]
        ok &= j.conferir(f"{nome} {pose[:4]}", not fora, str(fora) * bool(fora))
    for n, (lo, hi) in limites.items():
        print(f"  {n:15s} {lo:+7.1f} a {hi:+6.1f} graus")

    print("\n7. Colisão do URDF = caixas_colisao.json (uma <box> por caixa)")
    raiz = ET.fromstring(texto)
    pior, contagem = 0.0, True
    for link in raiz.findall("link"):
        boxes = link.findall("collision")
        esperado = caixas.get(link.get("name"), [])
        contagem &= len(boxes) == len(esperado)
        for col, (x0, y0, z0, x1, y1, z1) in zip(boxes, esperado):
            c = [float(v) / MM for v in col.find("origin").get("xyz").split()]
            t = [float(v) / MM for v in col.find("geometry/box").get("size").split()]
            lido = [c[i] - t[i] / 2 for i in range(3)] + [c[i] + t[i] / 2 for i in range(3)]
            pior = max(pior, max(abs(a - b) for a, b in zip(lido, (x0, y0, z0, x1, y1, z1))))
    ok &= j.conferir("mesmo número de caixas por elo e mesmas medidas", contagem and pior < 0.001,
                     f"maior diferença {pior:.4f} mm")

    print("\n5. check_urdf (ROS 2)")
    r = subprocess.run(["bash", "-c", f"source /opt/ros/jazzy/setup.bash && check_urdf '{f.name}'"],
                       capture_output=True, text=True)
    os.unlink(f.name)
    ok &= j.conferir("check_urdf aceita", r.returncode == 0 and "Successfully Parsed" in r.stdout,
                     (r.stdout + r.stderr).strip().splitlines()[-1] if r.returncode else "")
    return ok, r.stdout


def imagem():
    import vtk
    links, juntas = ru.ler_urdf(URDF, {PACOTE: PKG})
    mundo = ru.transformadas(juntas, {"junta_base": math.radians(35), "junta_ombro": math.radians(20),
                                      "junta_cotovelo": math.radians(-70), "junta_punho": math.radians(-30)})
    janela, r = vtk.vtkRenderWindow(), vtk.vtkRenderer()
    janela.SetOffScreenRendering(1)
    janela.SetSize(1400, 1000)
    r.SetBackground(1, 1, 1)
    janela.AddRenderer(r)
    ru.montar_cena(r, links, mundo, mesa=((-0.12, -0.15), (0.32, 0.25)), marcador="tcp")
    ru.camera(r, foco=(0.12, 0, 0.08), posicao=(0.38, -0.38, 0.30), zoom=1.0)
    caminho = os.path.join(j.PROJETO, "previews", "urdf_render.png")
    ru.salvar(janela, caminho)
    print(f"  Imagem: previews/urdf_render.png (montada pelo URDF e pelos STL do pacote)")


if __name__ == "__main__":
    with open(os.path.join(j.PROJETO, "resultados", "massas.json")) as f:
        massas = json.load(f)["elos"]
    servo = servo_step.carregar()
    limites = limites_graus(servo)
    with open(os.path.join(j.PROJETO, "resultados", "caixas_colisao.json")) as f:
        caixas = json.load(f)["caixas"]
    texto = gerar(massas, limites, caixas)
    ok, arvore = verificar(texto, massas, limites, caixas)
    texto_gz = gerar_gazebo()
    ok &= verificar_gazebo(texto, texto_gz)
    exigir_tudo_ok(ok)
    with open(URDF, "w", encoding="utf-8") as f:
        f.write(texto)
    with open(URDF_GZ, "w", encoding="utf-8") as f:
        f.write(texto_gz)
    print(f"\nURDF escrito: ros2/{PACOTE}/urdf/braco.urdf e braco_gazebo.urdf.xacro")
    print(arvore)
    imagem()
