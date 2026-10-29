#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
montagem.py
===========
O braço inteiro, montado a partir dos módulos (base, elo, garra), numa pose
dada pelos ângulos das 4 juntas e pela abertura da garra. Único módulo que
conhece todas as peças (decisão 2026-10-13).

Faz:
  1. cinemática direta pela CADEIA DE SÓLIDOS (a mesma que vai virar URDF)
     comparada com a fórmula à mão (braço planar + giro da base);
  2. colisão entre elos NÃO vizinhos em poses de trabalho (tem que dar zero)
     e numa grade de poses (informativo: dobrar demais bate, e quem evita isso
     depois é o MoveIt2);
  3. STEP do braço inteiro com um nome por elo e por peça, e imagens de lado.

Convenção dos ângulos (graus): q1 = base (positivo gira para a esquerda, +Y);
q2, q3, q4 = ombro, cotovelo, punho, POSITIVO = PARA CIMA (é o contrário do
giro positivo do servo nesse frame, ver frame_do_ombro); 0 = braço esticado
para a frente, horizontal.

Uso (de dentro de cad/):
    python montagem.py
"""

import itertools
import math
import os

import usar_lib  # noqa: F401
import cadquery as cq
from check_bbox import exigir_tudo_ok

import base as b
import elo as e
import garra as ga
import junta as j
import servo_step

L2, L3 = e.COMPRIMENTOS["braco"], e.COMPRIMENTOS["antebraco"]
# TCP = meio do comprimento dos dedos, entre eles; no frame da mão (x_s = 0, z_s = 0)
L4 = ga.Y0 + 19.0 + ga.T_CREM + ga.T_CONECTOR + ga.DEDO_L / 2.0     # 83,5
Z = cq.Vector(0, 0, 1)

POSES_TRABALHO = {          # (q1, q2, q3, q4, abertura): têm que estar livres de colisão
    "zero": (0, 0, 0, 0, 40),
    "pronto": (0, 60, -85, -65, 40),   # espera: garra apontando para a mesa, TCP a ~58 mm
    # (cotovelo a -85 e não -90: no LIMITE exato, o erro de seguimento do Gazebo passa do
    #  limite e o MoveIt2 recusa planejar dali; 2026-10-14)
    "pegar_na_mesa": (20, 40, -70, -60, 20),   # TCP a ~27 mm da mesa, a 158 mm do eixo
    "alto": (-30, 70, -40, -20, 40),
}


def cadeia(q):
    """Funções que levam um sólido do frame de cada elo para o mundo."""
    q1, q2, q3, q4 = q
    def mundo(s):               # elos que giram com a base
        return s.rotate(cq.Vector(), Z, q1)
    def ombro(s):               # frame do servo do ombro (braço)
        return mundo(b.no_ombro(s.rotate(cq.Vector(), Z, -q2)))
    def cotovelo(s):
        return ombro(s.rotate(cq.Vector(), Z, -q3).translate(cq.Vector(0, L2, 0)))
    def punho(s):
        return cotovelo(s.rotate(cq.Vector(), Z, -q4).translate(cq.Vector(0, L3, 0)))
    return {"base_link": lambda s: s, "link1": mundo, "link2": ombro, "link3": cotovelo, "link4": punho}


def pecas_por_elo(servo, abertura):
    """{elo: [(nome, sólido no frame do elo)]}. Servos ficam no elo do CORPO."""
    sol = servo.solids().vals()
    corpo = j.unir([sol[0], sol[17]])
    base = b.montar(servo)
    _, braco = e.elo(L2)
    _, ante = e.elo(L3)
    theta = (ga.ABERTURA - abertura) / (2 * ga.R)
    mv, _ = ga.mecanismo(theta)
    elos = {
        "base_link": [("base", base["base"].val()), ("servo_base", b.no_servo_base(corpo))],
        # link1 no frame do mundo (ele só gira com a base): torre + corpo do ombro
        "link1": [("torre", base["torre"].val()), ("servo_ombro", b.no_ombro(corpo))],
        "link2": [(f"braco_{k}_{i}", s.val()) for k, l in braco.items() for i, s in enumerate(l)]
                 + [("servo_cotovelo", corpo.translate(cq.Vector(0, L2, 0)))],
        "link3": [(f"antebraco_{k}_{i}", s.val()) for k, l in ante.items() for i, s in enumerate(l)]
                 + [("servo_punho", corpo.translate(cq.Vector(0, L3, 0)))],
        "link4": [("chapa_mao_horn", ga.chapa_mao().val()), ("chapa_mao_idler", ga.chapa_mao().mirror("XY").val()),
                  ("servo_garra", ga.g_para_m(ga.servo_em_g(corpo)))]
                 + [(k, ga.g_para_m(s)) for k, s in ga.fixas_g().items()]
                 + [(k, ga.g_para_m(s)) for k, s in mv.items()],
    }
    return elos


def montar(q, servo, abertura=40.0, elos=None):
    elos = elos or pecas_por_elo(servo, abertura)
    tf = cadeia(q)
    return {n: [(nome, tf[n](s)) for nome, s in lista] for n, lista in elos.items()}


def tcp_formula(q):
    """Cinemática direta à mão: no plano do braço, cada ângulo soma; a base gira o plano."""
    q1, q2, q3, q4 = (math.radians(a) for a in q)
    a2, a23, a234 = q2, q2 + q3, q2 + q3 + q4
    r = L2 * math.cos(a2) + L3 * math.cos(a23) + L4 * math.cos(a234)
    z = b.Z_S + L2 * math.sin(a2) + L3 * math.sin(a23) + L4 * math.sin(a234)
    return (r * math.cos(q1), r * math.sin(q1), z)


def tcp_cadeia(q):
    """O mesmo ponto levado pela cadeia de sólidos (um vértice minúsculo no TCP da mão)."""
    ponto = cq.Workplane("XY").box(0.01, 0.01, 0.01).translate((0, L4, 0)).val()
    c = cadeia(q)["link4"](ponto).Center()
    return (c.x, c.y, c.z)


VIZINHOS = {("base_link", "link1"), ("link1", "link2"), ("link2", "link3"), ("link3", "link4")}


def colisoes(montado):
    unidos = {n: j.unir([s for nome, s in l if "parafusos_horn" not in nome]) for n, l in montado.items()}
    out = []
    for a, c in itertools.combinations(unidos, 2):
        if (a, c) in VIZINHOS:
            continue
        v = unidos[a].intersect(unidos[c]).Volume()
        if v > 0.01:
            out.append((a, c, round(v, 1)))
    return out


def verificar(servo):
    ok = True
    print(f"\nL2 = {L2}, L3 = {L3}, L4 (punho -> meio dos dedos) = {L4}; ombro em z = {b.Z_S}")
    print(f"Alcance com o braço esticado (do eixo da base ao TCP): {L2 + L3 + L4:.1f} mm")
    print("\n1. Cinemática direta: cadeia de sólidos x fórmula à mão")
    for q in [(0, 0, 0, 0), (30, 20, -45, 10), (-60, 70, -40, -20), (90, -10, 80, 45)]:
        f, c = tcp_formula(q), tcp_cadeia(q)
        erro = math.dist(f, c)
        ok &= j.conferir(f"q = {q}", erro < 0.01, f"TCP ({c[0]:.1f}, {c[1]:.1f}, {c[2]:.1f}), diferença {erro:.4f} mm")

    elos = pecas_por_elo(servo, 40.0)
    print("\n2. Colisão entre elos não vizinhos e com a mesa nas poses de trabalho")
    for nome, (q1, q2, q3, q4, ab) in POSES_TRABALHO.items():
        el = elos if ab == 40 else pecas_por_elo(servo, ab)
        m = montar((q1, q2, q3, q4), servo, elos=el)
        bate = colisoes(m)
        mesa = cq.Workplane("XY").box(800, 800, 10, centered=(True, True, False)).translate((0, 0, -10)).val()
        bate += [(n, "mesa") for n, l in m.items() if n != "base_link"
                 and j.unir([s for _, s in l]).intersect(mesa).Volume() > 0.01]
        z_tcp = tcp_formula((q1, q2, q3, q4))[2]
        ok &= j.conferir(f"{nome} {q1, q2, q3, q4}, garra {ab} mm", not bate,
                         (str(bate) + " ") * bool(bate) + f"TCP a z = {z_tcp:.1f}")
    print("\n3. (informativo) grade: ombro 0/45/90, cotovelo -120..120, punho -90/0/90")
    n_bate = 0
    for q2, q3, q4 in itertools.product((0, 45, 90), (-120, -60, 0, 60, 120), (-90, 0, 90)):
        bate = colisoes(montar((0, q2, q3, q4), servo, elos=elos))
        if bate:
            n_bate += 1
            print(f"  bate em {(0, q2, q3, q4)}: {bate}")
    print(f"  {n_bate} de 45 poses com colisão entre elos (o MoveIt2 vai evitar essas)")
    return ok


def exportar(servo, q=(0, 60, -90, -60), abertura=20.0):
    pasta = os.path.join(j.PROJETO, "cad_export", "montagem")
    os.makedirs(pasta, exist_ok=True)
    m = montar(q, servo, abertura)
    asm = cq.Assembly(name="braco_completo")
    for elo, lista in m.items():
        sub = cq.Assembly(name=elo)
        for nome, s in lista:
            cor = (j.CORES["servo"] if nome.startswith("servo") else j.CORES["porca"] if "parafuso" in nome or "porca" in nome
                   else j.CORES["ponte"] if any(k in nome for k in ("ponte", "flange", "guia", "base", "torre")) else j.CORES["chapa"])
            sub.add(s, name=nome, color=cq.Color(*cor))
        asm.add(sub, name=elo)
    asm.export(os.path.join(pasta, "braco_completo.step"))
    print(f"  STEP: cad_export/montagem/braco_completo.step (pose {q}, garra {abertura} mm)")


def imagens(servo):
    from render_cad import render
    pasta = os.path.join(j.PROJETO, "previews")
    elos = pecas_por_elo(servo, 20.0)
    for q, nome, pos in (((0, 0, 0, 0), "braco_completo_zero.png", (140, -560, 160)),
                         ((0, 60, -90, -60), "braco_completo_pronto.png", (140, -560, 160)),
                         ((35, 20, -70, -30), "braco_completo_3d.png", (380, -380, 300))):
        m = montar(q, servo, elos=elos)
        itens = []
        for elo, lista in m.items():
            for n, s in lista:
                cor = (j.CORES["servo"] if n.startswith("servo") else (0.85, 0.2, 0.2) if "mordente" in n
                       else j.CORES["ponte"] if any(k in n for k in ("ponte", "flange", "guia", "base")) else
                       (0.3, 0.3, 0.75) if n == "torre" else j.CORES["porca"] if "parafuso" in n or "porca" in n
                       or "pinhao" in n or "cremalheira" in n else j.CORES["chapa"])
                itens.append((s, cor))
        render(itens, os.path.join(pasta, nome), foco=(120, 0, 80), posicao=pos, zoom=1.0)


if __name__ == "__main__":
    servo = servo_step.carregar()
    exigir_tudo_ok(verificar(servo))
    exportar(servo)
    imagens(servo)
