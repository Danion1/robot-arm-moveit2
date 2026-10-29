#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
medir_x430_step.py
==================
Confere o STEP oficial do X430 (XM,H-430_idler.stp) contra a furação
DIGITADA do desenho em PDF, que está em
cad/lib/dados_servos_dynamixel.py (X430_FURACAO).

As duas fontes são da ROBOTIS mas independentes no que importa: o STEP é
geometria, o PDF é cota escrita. O CAD do Projeto 2 usa os números da
tabela; este script prova que eles batem com a geometria real.

O que é conferido (para no primeiro [FALHOU], com código de saída 1):
  1. caixa do corpo contra o datasheet (28,5 x 46,5 x 34, eixo a 11,25);
  2. faces do horn e do idler (2 mm além do corpo) e a saliência central;
  3. cada furo com rosca da tabela existe no STEP: mesmo eixo, mesmo
     centro (0,01 mm), diâmetro da broca de rosca e profundidade até o
     máximo do desenho;
  4. o STEP não tem furo com rosca FORA da tabela;
  5. os rebaixos dos parafusos da carcaça (Ø4,8 x 2,8) nos 4 cantos das
     duas faces;
  6. o raio máximo do corpo do lado da ponta curta (cantos chanfrados),
     contra `x430_raio_max_ponta_curta()`; é o que a ponte da forquilha
     precisa contornar.

Broca de rosca (diâmetro do furo antes de abrir a rosca), tabela usual de
rosca métrica grossa (ISO 2306, classe 6H): M2x0,4 -> 1,6 mm;
M2,5x0,45 -> 2,05 mm. É o que o STEP modela no lugar da rosca.

Uso:
    python medir_x430_step.py
"""

import math
import os
import sys
from collections import defaultdict

import cadquery as cq

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(HERE, "..", "cad", "lib")))
import dados_servos_dynamixel as sv  # noqa: E402

# Caminho, SHA-256 e conferência do arquivo de fabricante: dono é cad/servo_step.py
sys.path.insert(0, os.path.join(HERE, "..", "cad"))
from servo_step import STEP, STEP_SHA256, conferir_arquivo_de_fabricante  # noqa: E402
BROCA_DE_ROSCA = {"M2x0.4": 1.6, "M2.5x0.45": 2.05}   # ISO 2306, ver cabeçalho
TOL = 0.01


ok_geral = True


def conferir(nome, ok, detalhe=""):
    global ok_geral
    print(f"  [{'OK' if ok else 'FALHOU'}] {nome}{(': ' + detalhe) if detalhe else ''}")
    ok_geral &= ok


def cilindros(solido):
    """Agrupa as faces cilíndricas por (eixo, posição da linha, diâmetro).
    Devolve {chave: [ (min, max) ao longo do eixo, ... ]} só dos FUROS
    (faces côncavas: a normal aponta para o eixo)."""
    g = defaultdict(list)
    for f in solido.Faces():
        if f.geomType() != "CYLINDER":
            continue
        cil = f._geomAdaptor().Cylinder()
        d, p = cil.Axis().Direction(), cil.Axis().Location()
        comp = [abs(d.X()), abs(d.Y()), abs(d.Z())]
        if max(comp) < 0.999:
            continue
        k = comp.index(max(comp))                       # 0 = x, 1 = y, 2 = z
        meio, normal = f.Center(), f.normalAt(f.Center())
        u = cq.Vector(d.X(), d.Y(), d.Z())
        v = meio - cq.Vector(p.X(), p.Y(), p.Z())
        if (v - u * v.dot(u)).dot(normal) >= 0:         # convexo: não é furo
            continue
        pos = [p.X(), p.Y(), p.Z()]
        linha = tuple(round(pos[i], 2) for i in range(3) if i != k)
        bb = f.BoundingBox()
        ext = [(bb.xmin, bb.xmax), (bb.ymin, bb.ymax), (bb.zmin, bb.zmax)][k]
        g[("xyz"[k], linha, round(2 * cil.Radius(), 2))].append(ext)
    return g


def furos_da_tabela():
    """(nome, eixo, linha, ponto na face, sinal da face, rosca, prof. máx.) de cada furo."""
    saida = []
    for nome, f in sv.X430_FURACAO.items():
        if "rosca" not in f:
            continue
        centros = f["centros"] if "centros" in f else sv.x430_centros_pcd(nome)
        for c in centros:
            if f["eixo"].endswith("z"):
                k, sinal = 2, (1 if c[2] > 0 else -1)
            elif f["eixo"].endswith("y"):
                k, sinal = 1, (1 if c[1] > 0 else -1)
            else:
                k, sinal = 0, (1 if c[0] > 0 else -1)
            linha = tuple(round(c[i], 2) for i in range(3) if i != k)
            saida.append((nome, "xyz"[k], linha, c[k], sinal, f["rosca"], f["prof_max_mm"]))
    return saida


conferir_arquivo_de_fabricante(STEP, STEP_SHA256)
montagem = cq.importers.importStep(STEP)
corpo = montagem.solids().vals()[0]      # o sólido 0 é o servo; os outros são parafusos, anéis e a capa do cabo
print(f"STEP: {os.path.basename(STEP)} ({len(montagem.solids().vals())} sólidos; o 0 é o servo)")

print("\n1. Caixa do corpo contra o datasheet (mm)")
bb = corpo.BoundingBox()
conferir("largura x = 28,5", abs(bb.xmin + 14.25) < TOL and abs(bb.xmax - 14.25) < TOL,
         f"{bb.xmin:.2f}..{bb.xmax:.2f}")
conferir("comprimento y = 46,5, eixo a 11,25 da ponta curta",
         abs(bb.ymin + 35.25) < TOL and abs(bb.ymax - 11.25) < TOL, f"{bb.ymin:.2f}..{bb.ymax:.2f}")

print("\n2. Faces planas ao longo do eixo (z)")
planos = defaultdict(float)
for f in corpo.Faces():
    if f.geomType() == "PLANE" and abs(abs(f.normalAt(f.Center()).z) - 1) < 1e-6:
        planos[round(f.Center().z, 2)] += f.Area()
for z, o_que in ((17.0, "face do corpo, lado do horn (34/2)"), (-17.0, "face do corpo, lado do idler"),
                 (19.0, "face de fora do horn (+2, desenho)"), (-19.0, "face de fora do idler"),
                 (17.0 + 2.0 + sv.X430_HORN_SALIENCIA_CENTRAL["altura_mm"], "topo da saliência Ø8 (+2,2)")):
    conferir(f"z = {z:+.1f}: {o_que}", planos.get(round(z, 2), 0) > 1.0)

print("\n3. Furos com rosca da tabela (centro, eixo, broca, profundidade)")
furos = cilindros(corpo)
usados = set()
for nome, eixo, linha, face, sinal, rosca, prof_max in furos_da_tabela():
    chave = (eixo, linha, BROCA_DE_ROSCA[rosca])
    trechos = [e for e in furos.get(chave, []) if (e[0] + e[1]) * sinal > 0]   # do lado certo
    if not trechos:
        conferir(f"{nome} {linha}", False, f"não achei furo Ø{chave[2]} em {eixo} do lado {sinal:+d}")
        continue
    usados.add(chave)
    fundo = min(e[0] for e in trechos) if sinal > 0 else max(e[1] for e in trechos)
    prof = abs(face - fundo)
    conferir(f"{nome} {eixo}={face:+.2f} em {linha}", prof <= prof_max + TOL,
             f"Ø{chave[2]} ({rosca}), profundidade {prof:.2f} (máx. {prof_max})")

print("\n4. Nenhum furo com rosca fora da tabela")
sobra = [k for k in furos if k[2] in BROCA_DE_ROSCA.values() and k not in usados]
# Os furos M2,5 dos parafusos da carcaça continuam em Ø2,05 dentro do corpo:
# são esperados e conferidos no item 5.
cantos = {("z", c, 2.05) for c in sv.X430_FURACAO["parafusos_carcaca"]["centros_xy"]}
sobra = [k for k in sobra if k not in cantos]
conferir("todos os furos Ø1,6 e Ø2,05 do STEP estão na tabela", not sobra, str(sobra) if sobra else "")

print("\n5. Rebaixos dos parafusos da carcaça")
pc = sv.X430_FURACAO["parafusos_carcaca"]
for xy in pc["centros_xy"]:
    # set(): o kernel divide cada cilindro em duas metades, com o mesmo trecho
    trechos = set((round(a, 2), round(b, 2)) for a, b in furos.get(("z", xy, pc["rebaixo_d_mm"]), []))
    lados = sorted(round(b - a, 2) for a, b in trechos)
    conferir(f"canto {xy}", lados == [pc["rebaixo_prof_mm"]] * 2,
             f"Ø{pc['rebaixo_d_mm']}, profundidade nas duas faces = {lados}")

print("\n6. Envelope do lado da ponta curta (chanfro dos cantos)")
# Amostra todas as arestas: pega também o meio de arcos, não só os vértices.
r_max = 0.0
for e in corpo.Edges():
    for i in range(41):
        p = e.positionAt(i / 40)
        if p.y > 0:
            r_max = max(r_max, math.hypot(p.x, p.y))
r_tab = sv.x430_raio_max_ponta_curta()
conferir("raio máximo do corpo, lado curto", abs(r_max - r_tab) <= TOL,
         f"STEP {r_max:.3f}, tabela {r_tab:.3f} (chanfro {sv.X430_CHANFRO_PONTA_CURTA} mm)")

print()
if not ok_geral:
    raise SystemExit("FALHOU: a tabela X430_FURACAO não bate com o STEP oficial.")
print("Tudo bateu: a furação digitada do PDF é a geometria do STEP.")
