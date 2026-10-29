#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
base.py
=======
Base fixa + torre giratória, no FRAME DO MUNDO (REP-103: +X frente, +Y
esquerda, +Z cima, mesa em Z = 0, eixo de giro da base em X = Y = 0).

  base (base_link): placa em tripé presa na mesa + berço com duas paredes
      que seguram o servo da junta 1 (XM430, eixo vertical, horn para cima)
      pelos 8 furos M2,5 das laterais (X430_FURACAO "lateral_m25").
  torre (link1): UMA peça em L sobre o horn da base (8× M2). A perna
      deitada é o prato giratório; a perna em pé segura o servo do OMBRO
      pela ponta longa (4× M2,5, igual ao flange do elo). O ombro fica
      deitado, eixo em Y, ponta longa para trás (-X): assim a pose zero do
      braço é horizontal, para a frente, e a forquilha tem os ±90° em
      torno dela (a ponte passa pela ponta CURTA, ver junta.py).

Por que a torre é uma peça só, impressa DE LADO (perfil em L na mesa,
28,5 mm de altura de impressão): o torque do ombro chega como momento em
torno de Y e dobra o L no plano XZ. Impressa de lado, esse plano é o das
camadas (forte). A versão com chapas laterais + blocos (linguagem do elo)
foi descartada nas contas: a faixa de chapa sob o servo do ombro só pode
ter 12 mm de altura (FS 2,3). Custo: os furos ficam horizontais na
impressão (saem levemente ovais; folga ASSUMIDA igual às outras).

Uso (de dentro de cad/):
    python base.py
"""

import math
import os

import usar_lib  # noqa: F401
import cadquery as cq
from OCP.BRepAdaptor import BRepAdaptor_Surface
import dados_servos_dynamixel as sv
from check_bbox import check_bbox, check_volume, exigir_tudo_ok
from exportar_stl_mm import exportar_stl_mm

import elo as e
import junta as j
import parafusos_iso as pi
import resistencia_forquilha as rf
import servo_step

# ---------------------------------------------------------------------------
# Parâmetros
# ---------------------------------------------------------------------------
GAP = 2.0             # ASSUMIDO: peça que gira sobre a face do servo (mesmo vão da chapa no horn)
T_BASE = 6.0          # ASSUMIDO: placa da base
Z_B = T_BASE + GAP + abs(sv.X430_FURACAO["idler_m2"]["z_face"])   # 27: centro do servo da base (idler 2 mm acima da placa)
Z_HORN_B = Z_B + j.Z_HORN                                          # 46: face do horn da base

# Berço: paredes nas laterais do servo da base
T_PAREDE = 5.0        # ASSUMIDO: dá M2,5x6 com rebaixo 1,5 (conta abaixo)
# Aqui a cabeça pode ficar para fora (lado de fora do berço, nada passa ali)
OPC_LAT = max(pi.rebaixo_para_rosca_cega("M2.5", T_PAREDE, 3.0, e.FOLGA_FUNDO_M25,
                                         e.ENGAJ_MIN_M25, e.PRESA_MIN_M25), key=lambda o: o["presa"])
Z_TOPO_PAREDE = Z_B + 12.0   # ASSUMIDO: 6 acima da fileira de furos de cima (z = +6 do servo)

# Tripé (fixação na mesa, ASSUMIDO aparafusado): 3 braços
R_CUBO = 42.0         # ASSUMIDO: contém o berço (canto em hypot(35,25; 19,25) = 40,2)
R_PE = 75.0           # ASSUMIDO: alcance dos furos de fixação
W_PE = 22.0           # ASSUMIDO: largura de cada braço do tripé
ANG_PES = (60.0, 180.0, 300.0)   # nenhum pé para a frente: o pé da frente parava o cotovelo a 40°
D_FURO_MESA = pi.furo_passagem("M3")        # 3,4: parafuso ou grampo M3 na mesa
# Passagem do cabo do servo da base (D3): rasgo na placa embaixo do servo, ASSUMIDO
RASGO_CABO = (-28.0, -8.0, 20.0)            # x0, x1, largura em y

# Torre (perfil em L no plano XZ, largura em Y = 28,5)
T_PRATO = 12.0        # altura do prato giratório (a perna deitada)
Z_PRATO_TOPO = Z_HORN_B + T_PRATO                         # 58
ELEVA_OMBRO = 14.0    # Aprovado (Daniel, 2026-10-13): com 0 a ponte do braço bate no prato a 12° para baixo;
                      # com 14 a mesa para o braço (44°) antes da base (56°). Ver verificação 6.
Z_S = Z_PRATO_TOPO + GAP + ELEVA_OMBRO + sv.X430_LARGURA / 2.0   # eixo do ombro
X_COLUNA = -sv.X430_EIXO_A_PONTA_LONGA                    # -35,25: face da ponta longa do ombro
D_COLUNA = 20.0       # espessura da coluna em X (flexão no plano, conta abaixo)
Z_COLUNA_TOPO = Z_S + sv.X430_LARGURA / 2.0               # 88,5
R_FILETE_L = 2.0      # canto de dentro do L; com 2 fica abaixo do servo (z < 60)

# Parafusos do prato no horn da base (M2, rosca do horn 2,0, mesma régua)
OPC_HORN_B = max((o for o in pi.rebaixo_para_rosca_cega("M2", T_PRATO, j.PROF_ROSCA_HORN,
                                                         j.FOLGA_FUNDO_HORN, j.ENGAJ_MIN_HORN, 3.0)
                  if o["saliencia_cabeca"] <= 0), key=lambda o: o["presa"])
PROF_BOLSO_CENTRAL = 5.5   # passa a saliência do horn (2,2) e a cabeça do parafuso central (até 4,7)
# Parafusos da coluna no ombro (M2,5 na ponta longa)
OPC_OMBRO = max((o for o in pi.rebaixo_para_rosca_cega("M2.5", D_COLUNA, 3.0, e.FOLGA_FUNDO_M25,
                                                        e.ENGAJ_MIN_M25, e.PRESA_MIN_M25)
                 if o["saliencia_cabeca"] <= 0), key=lambda o: o["presa"])


# ---------------------------------------------------------------------------
# Posição dos servos no mundo
# ---------------------------------------------------------------------------
def no_servo_base(s):
    """Frame do servo -> mundo, servo da base: x_s -> -Y, y_s -> +X, z_s -> +Z."""
    return s.rotate(cq.Vector(), cq.Vector(0, 0, 1), -90).translate(cq.Vector(0, 0, Z_B))


def no_ombro(s):
    """Frame do servo -> mundo, ombro: x_s -> +Z, y_s -> +X, z_s -> +Y
    (rotação de -120° em torno de (1,1,1)), eixo em (0, ·, Z_S)."""
    return s.rotate(cq.Vector(), cq.Vector(1, 1, 1), -120).translate(cq.Vector(0, 0, Z_S))


def furos_lateral_mundo():
    """(x, y, z) no mundo dos 8 furos M2,5 das laterais do servo da base."""
    return [(y, -x, Z_B + z) for x, y, z in sv.X430_FURACAO["lateral_m25"]["centros"]]


def furos_horn_b_xy():
    """Os 8 furos do horn, girados como o servo da base (x_s -> -Y, y_s -> +X)."""
    return [(y, -x) for x, y in j.furos_horn_xy()]


def furos_ombro_yz():
    """(y, z) no mundo das 4 roscas da ponta longa do ombro (x_s -> Z, z_s -> Y)."""
    return [(z, Z_S + x) for x, _, z in sv.X430_FURACAO["ponta_longa_m25"]["centros"]]


# ---------------------------------------------------------------------------
# Peças
# ---------------------------------------------------------------------------
def placa_base():
    p = cq.Workplane("XY").circle(R_CUBO).extrude(T_BASE)
    for a in ANG_PES:
        braco = (cq.Workplane("XY").center(R_PE / 2.0, 0).rect(R_PE, W_PE).extrude(T_BASE)
                 .union(cq.Workplane("XY").center(R_PE, 0).circle(W_PE / 2.0).extrude(T_BASE)))
        p = p.union(braco.rotate(cq.Vector(), cq.Vector(0, 0, 1), a))
    furos = [(R_PE * math.cos(math.radians(a)), R_PE * math.sin(math.radians(a))) for a in ANG_PES]
    p = p.cut(cq.Workplane("XY").pushPoints(furos).circle(D_FURO_MESA / 2.0).extrude(T_BASE))
    x0, x1, w = RASGO_CABO
    p = p.cut(cq.Workplane("XY").center((x0 + x1) / 2.0, 0).rect(x1 - x0, w).extrude(T_BASE))
    # Paredes do berço
    meia = sv.X430_LARGURA / 2.0
    for s in (1, -1):
        y0 = s * meia
        parede = (cq.Workplane("XY", origin=(0, 0, T_BASE))
                  .center((-sv.X430_EIXO_A_PONTA_LONGA + sv.X430_EIXO_A_PONTA_CURTA) / 2.0, y0 + s * T_PAREDE / 2.0)
                  .rect(sv.X430_COMPRIMENTO, T_PAREDE).extrude(Z_TOPO_PAREDE - T_BASE))
        p = p.union(parede)
        pts = [(x, z) for x, y, z in furos_lateral_mundo() if y * s > 0]
        fora = cq.Workplane("XZ", origin=(0, y0 + s * T_PAREDE, 0))   # normal do XZ = -Y
        p = p.cut(fora.pushPoints(pts).circle(e.D_FURO_M25 / 2.0).extrude(s * T_PAREDE))
        p = p.cut(fora.pushPoints(pts).circle(e.D_REBAIXO_M25 / 2.0).extrude(s * OPC_LAT["rebaixo"]))
    return p


def torre():
    meia = j.W_CHAPA / 2.0
    prato = (cq.Workplane("XY", origin=(0, 0, Z_HORN_B)).circle(meia).extrude(T_PRATO)
             .union(cq.Workplane("XY", origin=(0, 0, Z_HORN_B))
                    .center((X_COLUNA - D_COLUNA) / 2.0, 0).rect(-(X_COLUNA - D_COLUNA), j.W_CHAPA)
                    .extrude(T_PRATO)))
    coluna = (cq.Workplane("XY", origin=(0, 0, Z_PRATO_TOPO))
              .center(X_COLUNA - D_COLUNA / 2.0, 0).rect(D_COLUNA, j.W_CHAPA)
              .extrude(Z_COLUNA_TOPO - Z_PRATO_TOPO))
    t = prato.union(coluna)
    t = t.edges("|Y").edges(cq.selectors.NearestToPointSelector((X_COLUNA, 0, Z_PRATO_TOPO))).fillet(R_FILETE_L)
    # Horn da base: 8 furos + rebaixo por cima, bolsão central por baixo
    topo = cq.Workplane("XY", origin=(0, 0, Z_PRATO_TOPO))
    pts = furos_horn_b_xy()
    t = t.cut(topo.pushPoints(pts).circle(j.D_FURO_M2 / 2.0).extrude(-T_PRATO))
    t = t.cut(topo.pushPoints(pts).circle(j.D_REBAIXO_M2 / 2.0).extrude(-OPC_HORN_B["rebaixo"]))
    t = t.cut(cq.Workplane("XY", origin=(0, 0, Z_HORN_B)).circle(j.D_FURO_CENTRAL / 2.0)
              .extrude(PROF_BOLSO_CENTRAL))
    # Ombro: 4 M2,5 em X, rebaixo pela face de trás da coluna
    tras = cq.Workplane("YZ", origin=(X_COLUNA - D_COLUNA, 0, 0))     # normal do YZ = +X
    pts = furos_ombro_yz()
    t = t.cut(tras.pushPoints(pts).circle(e.D_FURO_M25 / 2.0).extrude(D_COLUNA))
    t = t.cut(tras.pushPoints(pts).circle(e.D_REBAIXO_M25 / 2.0).extrude(OPC_OMBRO["rebaixo"]))
    return t


# ---------------------------------------------------------------------------
# Verificações
# ---------------------------------------------------------------------------
# Torre, volume à mão (diário 2026-10-13):
#   prato (55,25·28,5 + pi·14,25²/2)·12 + coluna 20·28,5·44,5 + filete 2²(1-pi/4)·28,5
#   - 8·[pi·1,2²·12 + pi·(2,2²-1,2²)·rebaixo_horn] - pi·4,5²·5,5
#   - 4·[pi·1,45²·20 + pi·(2,75²-1,45²)·rebaixo_ombro]
TORRE_BBOX = (-55.25, 14.25, -14.25, 14.25, 46.0, 102.5)
BASE_BBOX = (-86.0, 48.5, -75.95, 75.95, 0.0, 39.0)   # pé de trás: -75 - 11; pés a ±60°: 75·cos60 + 11, 75·sin60 + 11
TORRE_VOL = 46054.94


def secao_prato_fs():
    """FS da flexão do prato em torno de Y na seção pelo eixo do horn (pior:
    bolsão central embaixo + 2 furos em y = ±8), momento = stall do ombro."""
    w, h = j.W_CHAPA, T_PRATO
    # retângulos (largura, altura, z do centro) a partir da base do prato; negativos = vazios
    r = [(w, h, h / 2), (-j.D_FURO_CENTRAL, PROF_BOLSO_CENTRAL, PROF_BOLSO_CENTRAL / 2),
         (-2 * j.D_FURO_M2, h, h / 2)]
    a = sum(b * t for b, t, _ in r)
    zc = sum(b * t * z for b, t, z in r) / a
    i = sum(b * t ** 3 / 12 + b * t * (z - zc) ** 2 for b, t, z in r)
    m = rf.TORQUE_PROJETO_NM * 1000.0
    return rf.ESCOAMENTO_MPA / (m * max(zc, h - zc) / i)


def verificar(pecas, servo):
    ok = True
    print("\n1. Caixa e volume")
    ok &= check_bbox("torre", pecas["torre"], TORRE_BBOX)
    ok &= check_volume("torre", pecas["torre"], TORRE_VOL, tol=0.2)
    ok &= check_bbox("base", pecas["base"], BASE_BBOX, tol=0.1)

    print("\n2. Interfaces lidas do sólido")
    def cilindros(solido, d):
        out = []
        for f in solido.val().Faces():
            if f.geomType() == "CYLINDER":
                c = BRepAdaptor_Surface(f.wrapped).Cylinder()   # _geomAdaptor() falha em face aparada
                if abs(2 * c.Radius() - d) < 1e-3:
                    p, v = c.Axis().Location(), c.Axis().Direction()
                    out.append(((round(p.X(), 2), round(p.Y(), 2), round(p.Z(), 2)),
                                (round(abs(v.X())), round(abs(v.Y())), round(abs(v.Z())))))
        return out
    furos = cilindros(pecas["torre"], j.D_FURO_M2)
    ok &= j.conferir("8 furos do prato sobre o horn da base",
                     {(round(x, 2), round(y, 2)) for x, y in furos_horn_b_xy()}
                     <= {(p[0], p[1]) for p, v in furos if v == (0, 0, 1)})
    furos = cilindros(pecas["torre"], e.D_FURO_M25)
    ok &= j.conferir("4 furos da coluna sobre as roscas da ponta longa do ombro",
                     {(round(y, 2), round(z, 2)) for y, z in furos_ombro_yz()}
                     <= {(p[1], p[2]) for p, v in furos if v == (1, 0, 0)})
    furos = cilindros(pecas["base"], e.D_FURO_M25)
    ok &= j.conferir("8 furos das paredes sobre as laterais do servo da base",
                     {(round(x, 2), round(z, 2)) for x, _, z in furos_lateral_mundo()}
                     <= {(p[0], p[2]) for p, v in furos if v == (0, 1, 0)})

    print("\n3. Parafusos")
    for nome, o in (("prato -> horn da base M2", OPC_HORN_B), ("coluna -> ombro M2,5", OPC_OMBRO),
                    ("paredes -> servo da base M2,5", OPC_LAT)):
        ok &= j.conferir(f"{nome}x{o['comprimento']}", o["saliencia_cabeca"] <= (1.0 if o is OPC_LAT else 0),
                         f"rosca {o['engajamento']}, presa {o['presa']}, rebaixo {o['rebaixo']}")

    print("\n4. Resistência (stall 4,1 N·m; FS mínimo 3)")
    m = rf.TORQUE_PROJETO_NM * 1000.0
    ok &= j.conferir("prato: flexão na seção do horn", secao_prato_fs() >= 3, f"FS {secao_prato_fs():.2f}")
    fs = rf.ESCOAMENTO_MPA / (6 * m / (j.W_CHAPA * T_PRATO ** 2))
    ok &= j.conferir("prato: flexão na raiz da coluna", fs >= 3, f"FS {fs:.2f}")
    fs = rf.ESCOAMENTO_MPA / (6 * m / (j.W_CHAPA * D_COLUNA ** 2))
    ok &= j.conferir("coluna: flexão", fs >= 3, f"FS {fs:.2f}")
    # Momento do ombro tomba o prato sobre o horn: tração nos parafusos, apoio na borda do horn
    r_apoio = sv.X430_HORN_DIAMETRO / 2.0
    d = [r_apoio + 8.0 * math.cos(math.radians(45 * k)) for k in range(8)]
    f_max = m * max(d) / sum(x * x for x in d)
    area = math.pi / 4 * (pi.PARAFUSO_ISO4762["M2"]["dk_max"] ** 2 - j.D_FURO_M2 ** 2)
    fs = rf.ESCOAMENTO_MPA / (f_max / area)
    ok &= j.conferir("cabeça M2 do prato no PLA (tombamento)", fs >= 3, f"{f_max:.0f} N, FS {fs:.2f}")
    # Mesmo modelo na coluna -> ombro (apoio na borda da face da ponta longa)
    d = [sv.X430_LARGURA / 2.0 + z for _, z in ((0, 8), (0, 8), (0, -8), (0, -8))]
    f_max = m * max(d) / sum(x * x for x in d)
    area = math.pi / 4 * (pi.PARAFUSO_ISO4762["M2.5"]["dk_max"] ** 2 - e.D_FURO_M25 ** 2)
    fs = rf.ESCOAMENTO_MPA / (f_max / area)
    ok &= j.conferir("cabeça M2,5 da coluna no PLA", fs >= 3, f"{f_max:.0f} N, FS {fs:.2f}")
    massa_n = (3 * sv.SERVOS["XM430-W350-T"]["massa_g"] + 2 * sv.SERVOS["XC430-W240-T"]["massa_g"] + 200) * 9.81e-3
    ok &= j.conferir("peso do braço no horn da base < carga axial do XM430", massa_n < sv.SERVOS["XM430-W350-T"]["carga_axial_n"],
                     f"~{massa_n:.1f} N (servos + 200 g ASSUMIDOS de plástico e garra) < 20 N")

    print("\n5. Giro da base (torre + ombro + braço contra base + servo da base)")
    fixos = j.unir([pecas["base"]] + [no_servo_base(s) for s in (servo.solids().vals()[0], servo.solids().vals()[17])])
    _, braco = e.elo(e.COMPRIMENTOS["braco"])
    est_braco = j.unir([no_ombro(s.val()) for k in ("chapa_horn", "chapa_idler", "ponte", "flange") for s in braco[k]])
    ombro = no_ombro(servo.solids().vals()[0])
    girante = j.unir([pecas["torre"], ombro, est_braco])
    bate = [a for a in range(0, 360, 30) if j.girar(girante, a).intersect(fixos).Volume() > 0.01]
    ok &= j.conferir("torre gira 360° sem bater na base", not bate, str(bate) if bate else "")

    print("\n6. Braço em torno do ombro contra a torre")
    eixo0, eixo1 = cq.Vector(0, 0, Z_S), cq.Vector(0, 1, Z_S)
    cotovelo = no_ombro(servo.solids().vals()[0].translate(cq.Vector(0, e.COMPRIMENTOS["braco"], 0)))
    movel = est_braco.fuse(cotovelo)
    parado = pecas["torre"].val().fuse(fixos)
    # rotação positiva em torno de +Y leva +X para -Z: positivo = para BAIXO
    bate_cima = [a for a in range(0, -91, -10) if movel.rotate(eixo0, eixo1, a).intersect(parado).Volume() > 0.01]
    ok &= j.conferir("braço sobe até a vertical (90°) sem bater", not bate_cima, str(bate_cima) if bate_cima else "")
    baixo = next((a for a in range(0, 91, 2) if movel.rotate(eixo0, eixo1, a).intersect(parado).Volume() > 0.01), 90)
    mesa = cq.Workplane("XY").box(600, 600, 10, centered=(True, True, False)).translate((0, 0, -10)).val()
    na_mesa = next((a for a in range(0, 91, 2) if movel.rotate(eixo0, eixo1, a).intersect(mesa).Volume() > 0.01), 90)
    # Requisito (Daniel, 2026-10-13): para baixo, quem para o braço é a mesa, nunca a base
    ok &= j.conferir("para baixo, a mesa para o braço antes da torre/base", baixo > na_mesa,
                     f"mesa a {na_mesa}°, torre/base a {baixo}°")
    return ok, baixo


def lista_de_materiais():
    print("\nLista de materiais (base + torre)")
    print("  1x base (placa em tripé + berço), 1x torre (imprimir DE LADO) — PLA")
    print(f"  8x ISO 4762 M2,5x{OPC_LAT['comprimento']} (berço -> servo da base)")
    print(f"  8x ISO 4762 M2x{OPC_HORN_B['comprimento']} (torre -> horn da base)")
    print(f"  4x ISO 4762 M2,5x{OPC_OMBRO['comprimento']} (torre -> ponta longa do ombro)")
    print("  3x parafuso ou grampo M3 (base -> mesa)")


def montar(servo):
    return {"base": placa_base(), "torre": torre()}


def exportar(pecas, servo):
    pasta = os.path.join(j.PROJETO, "cad_export", "base")
    os.makedirs(pasta, exist_ok=True)
    for nome in ("base", "torre"):
        exportar_stl_mm(pecas[nome], pasta, f"{nome}.stl")
        cq.exporters.export(pecas[nome], os.path.join(pasta, f"{nome}.step"))
    mont = cq.Assembly(name="base_torre")
    mont.add(pecas["base"], name="base", color=cq.Color(*j.CORES["ponte"]))
    mont.add(pecas["torre"], name="torre", color=cq.Color(*j.CORES["chapa"]))
    mont.add(no_servo_base(servo.solids().vals()[0]), name="servo_base_referencia",
             color=cq.Color(*j.CORES["servo"]))
    mont.add(no_ombro(servo.solids().vals()[0]), name="servo_ombro_referencia", color=cq.Color(*j.CORES["servo"]))
    mont.export(os.path.join(pasta, "base_montagem.step"))
    print("  STEP: base, torre e base_montagem em cad_export/base/")


def imagem(pecas, servo):
    from render_cad import render
    itens = [(pecas["base"], j.CORES["ponte"]), (pecas["torre"], j.CORES["chapa"])]
    itens += [(no_servo_base(s), j.CORES["servo"]) for s in servo.solids().vals()]
    itens += [(no_ombro(s), j.CORES["servo"]) for s in servo.solids().vals()]
    pasta = os.path.join(j.PROJETO, "previews")
    render(itens, os.path.join(pasta, "base.png"), foco=(0, 0, 40), posicao=(160, 190, 150), zoom=1.1)


if __name__ == "__main__":
    print(f"Servo da base: eixo em z = {Z_B}; horn em z = {Z_HORN_B}; eixo do ombro em z = {Z_S}")
    servo = servo_step.carregar()
    pecas = montar(servo)
    ok, baixo = verificar(pecas, servo)
    exigir_tudo_ok(ok)
    lista_de_materiais()
    exportar(pecas, servo)
    imagem(pecas, servo)
