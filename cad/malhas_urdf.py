#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
malhas_urdf.py
==============
Malhas STL por ELO do URDF (não por peça), cada uma no frame do próprio elo:
origem no ponto da junta, eixos do mundo na pose zero (convenção de
todo o CAD). Os frames são os do cabeçalho do massa.py, de
onde saem os <inertial>; malha e inércia TÊM que estar no mesmo frame.

Para cada elo, duas malhas (decisão D1, 2026-10-12):
  <elo>.stl          visual: todas as peças detalhadas (impressas, servo, ferragens)
  <elo>_colisao.stl  colisão: uma caixa por peça (sem parafusos e porcas, que
                     ficam dentro das outras), alinhada aos eixos do elo e
                     MARGEM mm maior de cada lado. Caixa por peça, e não contorno
                     convexo do elo, para não fechar o "U" da forquilha.

Como os frames são obtidos (sem repetir geometria): a peça vai para o mundo
pela cadeia do montagem.py na pose zero e volta pela translação da origem
do elo. Na pose zero todos os elos têm os eixos do mundo, então só translação.

Verificações antes de exportar:
  1. o frame obtido assim é o MESMO do massa.py (pontos de teste, erro < 1e-6 mm);
  2. origens das juntas = conta à mão (Z_HORN_B, Z_S, L2, L3);
  3. o centro de massa do massas.json cai dentro da caixa do elo;
  4. mordentes: aberto - fechado = ±20 mm em Y (eixo da prismática);
  2b. a cadeia do URDF (só a tabela de origens + eixos) reproduz a cadeia do
     montagem.py em poses quaisquer: prova a convenção "eixo -Y, positivo = cima";
  5. colisão: cada caixa contém a sua peça; poses de trabalho sem colisão entre
     elos não vizinhos também com as caixas (senão o MoveIt2 recusaria a pose);
Depois de exportar:
  6. o STL lido de volta tem a caixa do sólido (unidade em mm, lição 2).
  Também grava resultados/caixas_colisao.json (as caixas soltas, para o SRDF).

Uso (de dentro de cad/):  python malhas_urdf.py
Saída: ros2/braco2_description/meshes/ (gerado, fora do Git), previews/urdf_*_pela_cadeia.png
e a tabela de origens das juntas em METROS, para o <joint><origin> do URDF.
"""

import itertools
import json
import os
import struct

import usar_lib  # noqa: F401
import cadquery as cq
from check_bbox import exigir_tudo_ok
from exportar_stl_mm import exportar_stl_mm

import base as b
import junta as j
import montagem as mo
import servo_step

MARGEM = 1.0          # ASSUMIDO: folga da caixa de colisão em cada lado (mm)
# Direto no pacote ROS 2 (uma cópia só; meshes/ é ignorado pelo Git, este script a refaz)
PASTA = os.path.join(j.PROJETO, "ros2", "braco2_description", "meshes")
MORDENTES = ("cremalheira", "mordente")     # peças da garra que andam com cada mordente
# Peças cortadas em fatias horizontais (z no frame do elo) antes da caixa: uma
# caixa só da base (tripé + berço) fechava o vão entre os pés até 50 mm de altura.
FATIAS_Z = {"base": [b.T_BASE]}

# Origem de cada elo no mundo, pose zero (mm). Fonte: cabeçalho do massa.py.
ORIGEM = {
    "base_link": (0.0, 0.0, 0.0),
    "link1": (0.0, 0.0, b.Z_HORN_B),
    "link2": (0.0, 0.0, b.Z_S),
    "link3": (mo.L2, 0.0, b.Z_S),
    "link4": (mo.L2 + mo.L3, 0.0, b.Z_S),
}
ORIGEM["mordente_cima"] = ORIGEM["mordente_baixo"] = ORIGEM["link4"]
PAI = {"link1": "base_link", "link2": "link1", "link3": "link2", "link4": "link3",
       "mordente_cima": "link4", "mordente_baixo": "link4"}


def no_frame(elo, s):
    """Sólido já no MUNDO (pose zero) -> frame do elo."""
    x, y, z = ORIGEM[elo]
    return s.translate(cq.Vector(-x, -y, -z))


def pecas_nos_elos(servo):
    """{elo: [(nome, sólido no frame do elo)]}, garra FECHADA (prismáticas em 0)."""
    m = mo.montar((0, 0, 0, 0), servo, abertura=0.0)
    out = {k: [] for k in ORIGEM}
    for elo, lista in m.items():
        for nome, s in lista:
            dono = elo
            if elo == "link4" and nome.split("_")[0] in MORDENTES:
                dono = "mordente_" + nome.split("_")[-1]
            out[dono].append((nome, no_frame(dono, s)))
    return out


def eh_ferragem(nome):
    return "parafuso" in nome or "porca" in nome


def caixas_colisao(nome, s):
    """Caixas de colisão de uma peça: uma, ou uma por fatia (FATIAS_Z)."""
    cortes = FATIAS_Z.get(nome, [])
    if not cortes:
        return [caixa_colisao(s)]
    bb = s.BoundingBox()
    zs = [bb.zmin - 1] + cortes + [bb.zmax + 1]
    out = []
    for z0, z1 in zip(zs, zs[1:]):
        faixa = cq.Solid.makeBox(1e3, 1e3, z1 - z0, pnt=cq.Vector(-500, -500, z0))
        out.append(caixa_colisao(s.intersect(faixa)))
    return out


def caixa_colisao(s):
    bb = s.BoundingBox()
    c = cq.Vector((bb.xmin + bb.xmax) / 2, (bb.ymin + bb.ymax) / 2, (bb.zmin + bb.zmax) / 2)
    return cq.Solid.makeBox(bb.xlen + 2 * MARGEM, bb.ylen + 2 * MARGEM, bb.zlen + 2 * MARGEM,
                            pnt=cq.Vector(c.x - bb.xlen / 2 - MARGEM, c.y - bb.ylen / 2 - MARGEM,
                                          c.z - bb.zlen / 2 - MARGEM))


def ler_stl_caixa(path):
    """(min, max) dos vértices de um STL binário, lido sem biblioteca."""
    with open(path, "rb") as f:
        f.read(80)
        n = struct.unpack("<I", f.read(4))[0]
        lo, hi = [1e9] * 3, [-1e9] * 3
        for _ in range(n):
            d = struct.unpack("<12fH", f.read(50))
            for v in range(3):
                for k in range(3):
                    x = d[3 + 3 * v + k]
                    lo[k], hi[k] = min(lo[k], x), max(hi[k], x)
    return lo, hi


def verificar(servo, elos, colisao):
    ok = True
    print("1. Frame deste script = frame do massa.py (pontos de teste)")
    tf = mo.cadeia((0, 0, 0, 0))
    f1 = lambda s: s.translate(cq.Vector(0, 0, -b.Z_HORN_B))
    no_link2 = lambda s: s.rotate(cq.Vector(), cq.Vector(1, 1, 1), -120)      # cópia da linha do massa.py
    referencia = {"link1": f1, "link2": no_link2, "link3": no_link2, "link4": no_link2}
    # massa.py: link2/3/4 = frame do servo de cada junta girado por no_link2; aqui, mundo -> origem
    for elo, ref in referencia.items():
        erro = 0.0
        for p in ((0, 0, 0), (10, 0, 0), (0, 10, 0), (0, 0, 10), (7, -3, 5)):
            v = cq.Vertex.makeVertex(*p)
            a = no_frame(elo, tf[elo](v)).Center()
            r = ref(v).Center()
            erro = max(erro, (a - r).Length)
        ok &= j.conferir(f"{elo}", erro < 1e-6, f"erro máx. {erro:.1e} mm")

    print("\n2. Origens das juntas (mundo, pose zero) contra a conta à mão")
    esperado = {"link1": (0, 0, 46.0), "link2": (0, 0, 88.25), "link3": (105, 0, 88.25), "link4": (195, 0, 88.25)}
    for elo, e in esperado.items():
        o = tf[elo](cq.Vertex.makeVertex(0, 0, 0)).Center() if elo != "link1" else cq.Vector(0, 0, b.Z_HORN_B)
        d = (cq.Vector(*ORIGEM[elo]) - cq.Vector(*e)).Length + (0 if elo == "link1" else (o - cq.Vector(*e)).Length)
        ok &= j.conferir(f"{elo} em {e}", d < 1e-6)

    print("\n2b. Cadeia do URDF (origens + eixo -Y) = cadeia do montagem.py, pose qualquer")
    for q in ((30, 20, -45, 10), (-60, 70, -40, -20), (90, -10, 80, 45)):
        tu, tm = cadeia_urdf(q, 0.0), mo.cadeia(q)
        erro = 0.0
        for elo in ("link1", "link2", "link3", "link4"):
            for p in ((0, 0, 0), (30, 0, 0), (0, 20, 0), (0, 0, 10)):
                v = cq.Vertex.makeVertex(*p)
                a = tu[elo](no_frame(elo, tf[elo](v))).Center()     # peça no frame do elo -> URDF -> mundo
                erro = max(erro, (a - tm[elo](v).Center()).Length)
        ok &= j.conferir(f"q = {q}", erro < 1e-6, f"erro máx. {erro:.1e} mm")

    print("\n3. Centro de massa (massas.json) dentro da caixa do elo")
    with open(os.path.join(j.PROJETO, "resultados", "massas.json")) as f:
        massas = json.load(f)["elos"]
    for elo, lista in elos.items():
        bb = j.unir([s for _, s in lista]).BoundingBox()
        c = [x * 1000 for x in massas[elo]["com_m"]]
        dentro = bb.xmin <= c[0] <= bb.xmax and bb.ymin <= c[1] <= bb.ymax and bb.zmin <= c[2] <= bb.zmax
        ok &= j.conferir(f"{elo:15s}", dentro, f"com ({c[0]:.1f}, {c[1]:.1f}, {c[2]:.1f}); caixa x {bb.xmin:.1f}..{bb.xmax:.1f} "
                         f"y {bb.ymin:.1f}..{bb.ymax:.1f} z {bb.zmin:.1f}..{bb.zmax:.1f}")

    print("\n4. Mordentes: aberto (40 mm) - fechado, no frame do link4")
    aberto = {}
    for nome, s in mo.montar((0, 0, 0, 0), servo, abertura=40.0)["link4"]:
        if nome.startswith("mordente_"):
            aberto[nome] = no_frame(nome, s).Center()
    for lado, s in (("cima", 1), ("baixo", -1)):
        fechado = dict(elos[f"mordente_{lado}"])[f"mordente_{lado}"].Center()
        d = aberto[f"mordente_{lado}"] - fechado
        ok &= j.conferir(f"mordente_{lado}: anda {s * 20:+.0f} mm em Y", (d - cq.Vector(0, s * 20, 0)).Length < 1e-6,
                         f"({d.x:.3f}, {d.y:.3f}, {d.z:.3f})")

    print(f"\n5. Colisão (caixa por peça, margem {MARGEM} mm, ASSUMIDO)")
    for elo, lista in elos.items():
        caixas = colisao[elo]
        vol_c = j.unir(caixas).Volume()
        vol_p = j.unir([s for n, s in lista]).Volume()
        fora = j.unir([s for n, s in lista if not eh_ferragem(n)]).cut(j.unir(caixas)).Volume()
        ok &= j.conferir(f"{elo:15s} {len(caixas):2d} caixas contêm as peças", fora < 0.01,
                         f"volume caixas/peças = {vol_c / vol_p:.2f}")
    for nome, (q1, q2, q3, q4, ab) in mo.POSES_TRABALHO.items():
        tf = cadeia_urdf((q1, q2, q3, q4), ab)
        mundo = {elo: j.unir([tf[elo](c) for c in caixas]) for elo, caixas in colisao.items()}
        bate = []
        vizinhos = mo.VIZINHOS | {("link4", "mordente_cima"), ("link4", "mordente_baixo"),
                                  ("mordente_cima", "mordente_baixo")}
        for a, c in itertools.combinations(mundo, 2):
            if (a, c) in vizinhos:
                continue
            v = mundo[a].intersect(mundo[c]).Volume()
            if v > 0.01:
                bate.append((a, c, round(v, 1)))
        ok &= j.conferir(f"pose {nome} {q1, q2, q3, q4}, garra {ab} mm", not bate, str(bate) * bool(bate))
    return ok


def cadeia_urdf(q, abertura):
    """Funções frame do elo -> mundo, montadas SÓ com o que vai no URDF: origem da
    junta (tabela abaixo, rpy = 0) e eixo (base em +Z; ombro, cotovelo e punho em
    -Y, positivo = para cima; mordentes prismáticos em +Y / -Y)."""
    q1, q2, q3, q4 = q
    def junta(pai_tf, filho, eixo, ang=0.0, desloca=0.0):
        d = cq.Vector(*ORIGEM[filho]) - cq.Vector(*ORIGEM[PAI[filho]])
        return lambda s: pai_tf(s.rotate(cq.Vector(), cq.Vector(*eixo), ang)
                                .translate(cq.Vector(*eixo) * desloca).translate(d))
    tf = {"base_link": lambda s: s}
    tf["link1"] = junta(tf["base_link"], "link1", (0, 0, 1), q1)
    tf["link2"] = junta(tf["link1"], "link2", (0, -1, 0), q2)
    tf["link3"] = junta(tf["link2"], "link3", (0, -1, 0), q3)
    tf["link4"] = junta(tf["link3"], "link4", (0, -1, 0), q4)
    tf["mordente_cima"] = junta(tf["link4"], "mordente_cima", (0, 1, 0), desloca=abertura / 2)
    tf["mordente_baixo"] = junta(tf["link4"], "mordente_baixo", (0, -1, 0), desloca=abertura / 2)
    return tf


def tabela_juntas():
    print("\nOrigens para o <joint><origin xyz> do URDF (metros, pai -> filho, rpy = 0 0 0):")
    for filho, pai in PAI.items():
        d = [(a - c) / 1000 for a, c in zip(ORIGEM[filho], ORIGEM[pai])]
        print(f"  {pai:9s} -> {filho:15s} xyz = \"{d[0]:.5g} {d[1]:.5g} {d[2]:.5g}\"")


def imagens(elos, colisao, q=(35, 20, -70, -30), abertura=30.0):
    """Braço montado A PARTIR das malhas dos elos pela cadeia do URDF (como o
    RViz2 vai montar): visual e colisão, na mesma pose e câmera."""
    from render_cad import render
    cores = {"base_link": (0.5, 0.5, 0.5), "link1": (0.3, 0.3, 0.75), "link2": j.CORES["chapa"],
             "link3": (0.2, 0.65, 0.4), "link4": j.CORES["ponte"],
             "mordente_cima": (0.85, 0.2, 0.2), "mordente_baixo": (0.85, 0.2, 0.2)}
    tf = cadeia_urdf(q, abertura)
    pasta = os.path.join(j.PROJETO, "previews")
    for nome, grupos in (("urdf_visual_pela_cadeia.png", {e: [s for _, s in l] for e, l in elos.items()}),
                         ("urdf_colisao_pela_cadeia.png", colisao)):
        render([(tf[e](s), cores[e]) for e, l in grupos.items() for s in l], os.path.join(pasta, nome),
               foco=(120, 0, 80), posicao=(380, -380, 300), zoom=1.0)


if __name__ == "__main__":
    servo = servo_step.carregar()
    elos = pecas_nos_elos(servo)
    colisao = {elo: [c for n, s in lista if not eh_ferragem(n) for c in caixas_colisao(n, s)] for elo, lista in elos.items()}
    exigir_tudo_ok(verificar(servo, elos, colisao))

    print("\nExportando para ros2/braco2_description/meshes/")
    ok = True
    for elo, lista in elos.items():
        vis = j.unir([s for _, s in lista])
        for sufixo, solido in (("", vis), ("_colisao", j.unir(colisao[elo]))):
            path = exportar_stl_mm(cq.Compound.makeCompound([solido]), PASTA, f"{elo}{sufixo}.stl")
            lo, hi = ler_stl_caixa(path)
            bb = solido.BoundingBox()
            d = max(abs(a - c) for a, c in zip(lo + hi, [bb.xmin, bb.ymin, bb.zmin, bb.xmax, bb.ymax, bb.zmax]))
            ok &= j.conferir(f"    STL lido de volta = caixa do sólido, em mm", d < 0.1, f"diferença máx. {d:.3f} mm")
    exigir_tudo_ok(ok)
    # As caixas soltas (o STL de colisão as funde numa malha só): para o
    # gerar_moveit_config.py achar os pares que nunca colidem sem refazer o CAD.
    caixas = {elo: [[round(v, 4) for v in (bb.xmin, bb.ymin, bb.zmin, bb.xmax, bb.ymax, bb.zmax)]
                    for bb in (c.BoundingBox() for c in lista)] for elo, lista in colisao.items()}
    with open(os.path.join(j.PROJETO, "resultados", "caixas_colisao.json"), "w") as f:
        json.dump({"unidade": "mm, frame do elo, garra fechada", "margem_mm": MARGEM, "caixas": caixas}, f, indent=1)
    tabela_juntas()
    imagens(elos, colisao)
