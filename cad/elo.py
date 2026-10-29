#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
elo.py
======
Elo do braço: a forquilha do junta.py ESTICADA até o servo seguinte, numa
peça única (decisão 2026-10-13). Cada elo tem:
  - 2 chapas iguais (z = ±19..23), presas no horn e no idler do servo de
    trás (interface toda do junta.py);
  - a ponte do junta.py (espaçador perto do servo de trás);
  - um FLANGE (espaçador perto do servo da frente), parafusado nas 4
    roscas M2,5 da ponta LONGA do servo da frente (X430_FURACAO
    "ponta_longa_m25"). As faces do servo da frente perpendiculares ao
    eixo estão ocupadas pelo horn e pelo idler, que carregam o elo
    seguinte nos MESMOS planos z, por isso a ponta longa.

Frame: o do servo de TRÁS (= frame da junta; o elo gira em torno de z).
O servo da frente fica em (0, L, 0), com a ponta longa virada para trás.

Comprimentos (ESCOLHA DE PROJETO, iguais aos da demo, alcance 300 mm):
L2 = 105 (ombro -> cotovelo), L3 = 90 (cotovelo -> punho).

Uso (de dentro de cad/):
    python elo.py          # L2 e L3
    python elo.py 105      # só um
"""

import math
import os
import sys

import usar_lib  # noqa: F401
import cadquery as cq
import dados_servos_dynamixel as sv
from check_bbox import check_bbox, check_volume, exigir_tudo_ok
from exportar_stl_mm import exportar_stl_mm

import junta as j
import parafusos_iso as pi
import resistencia_forquilha as rf
import servo_step

COMPRIMENTOS = {"braco": 105.0, "antebraco": 90.0}   # L2, L3 (demo)

# Flange: 2 fileiras de parafusos chapa -> flange. Com 1 fileira (flange de
# 7,5 como a ponte) o torque do servo da frente dá FS 1,96 no furo da chapa;
# com 2 fileiras a 7,5 uma da outra, FS 4,3 (conta em verificar()).
D_FLANGE = 2 * j.D_PONTE                    # 15,0
DY_FILEIRAS = j.D_PONTE                     # 7,5 entre as fileiras

# Parafusos do flange no servo da frente: M2,5 nas roscas da ponta longa
F_M25 = sv.X430_FURACAO["ponta_longa_m25"]
PROF_ROSCA_M25 = F_M25["prof_max_mm"]       # 3,0
FOLGA_FUNDO_M25 = 0.5    # ASSUMIDO, mesma régua do horn
ENGAJ_MIN_M25 = 2.0      # ASSUMIDO: 4,4 fios de M2,5
PRESA_MIN_M25 = 3.0      # ASSUMIDO: plástico mínimo sob a cabeça
# Entre as opções, a de maior presa com a cabeça ainda embutida (não sai para dentro do elo)
OPCAO_M25 = max((o for o in pi.rebaixo_para_rosca_cega(
    "M2.5", D_FLANGE, PROF_ROSCA_M25, FOLGA_FUNDO_M25, ENGAJ_MIN_M25, PRESA_MIN_M25)
    if o["saliencia_cabeca"] <= 0), key=lambda o: o["presa"])
D_FURO_M25 = pi.furo_passagem("M2.5")       # 2,9
D_REBAIXO_M25 = pi.REBAIXO_DIAMETRO["M2.5"]  # 5,5


def geometria(L):
    """Cotas do elo de comprimento L (distância entre eixos)."""
    y_fim = L - sv.X430_EIXO_A_PONTA_LONGA          # face da ponta longa do servo da frente
    y0 = y_fim - D_FLANGE
    yc = (y0 + y_fim) / 2.0
    filas = (yc - DY_FILEIRAS / 2.0, yc + DY_FILEIRAS / 2.0)
    furos_chapa = [(s * j.X_PARAF_PONTE, y) for y in filas for s in (-1, 1)]
    furos_m25 = [(x, z) for x, _, z in F_M25["centros"]]    # (x, z) na face
    return {"L": L, "y_fim": y_fim, "y0": y0, "filas": filas,
            "furos_chapa": furos_chapa, "furos_m25": furos_m25}


def flange(g):
    """Bloco entre as faces internas das chapas (z = -19..19), y = y0..y_fim."""
    f = (cq.Workplane("XY", origin=(0, (g["y0"] + g["y_fim"]) / 2.0, -j.Z_HORN))
         .rect(j.W_CHAPA, D_FLANGE).extrude(2 * j.Z_HORN))
    # M2,5 para o servo: furo passante em y + rebaixo do lado de dentro do elo (y0)
    tras = cq.Workplane("XZ", origin=(0, g["y0"], 0))      # normal do plano XZ = -y
    pts = [(x, z) for x, z in g["furos_m25"]]
    f = f.cut(tras.pushPoints(pts).circle(D_FURO_M25 / 2.0).extrude(-D_FLANGE))
    f = f.cut(tras.pushPoints(pts).circle(D_REBAIXO_M25 / 2.0).extrude(-OPCAO_M25["rebaixo"]))
    # Parafusos das chapas (z) com porca embutida, rasgo aberto para a lateral (±x)
    for s in (1, -1):
        face = cq.Workplane("XY", origin=(0, 0, s * j.Z_HORN))
        f = f.cut(face.pushPoints(g["furos_chapa"]).circle(j.D_FURO_M2 / 2.0)
                  .extrude(-s * j.PROF_FURO_PONTE))
        z_base = j.Z_ALOJ_BASE if s == 1 else -j.Z_ALOJ_TOPO
        for x, y in g["furos_chapa"]:
            lado = "+x" if x > 0 else "-x"
            f = f.cut(j.alojamento_porca(x, y, z_base, lado, math.copysign(j.W_CHAPA / 2.0, x)))
    return f


def porca_m2_em(x, y, z_centro):
    """Porca M2 com faces para ±y (entra pelo rasgo em x)."""
    p = j.PORCA
    z0 = z_centro - p["m_max"] / 2.0
    return (cq.Workplane("XY", origin=(x, y, z0))
            .polygon(6, p["s"] / math.cos(math.radians(30))).extrude(p["m_max"])
            .cut(cq.Workplane("XY", origin=(x, y, z0)).circle(1.0).extrude(p["m_max"])))


def parafuso_m25_y(x, y_apoio, z):
    """ISO 4762 M2,5 simplificado, haste para +y a partir do apoio da cabeça."""
    d = pi.PARAFUSO_ISO4762["M2.5"]
    cab = cq.Workplane("XZ", origin=(x, y_apoio, z)).circle(d["dk_max"] / 2.0).extrude(d["k_max"])
    haste = cq.Workplane("XZ", origin=(x, y_apoio, z)).circle(d["d"] / 2.0).extrude(-OPCAO_M25["comprimento"])
    return cab.union(haste)


def elo(L):
    """Peças do elo na pose zero, no frame do servo de trás."""
    g = geometria(L)
    ch = j.chapa(y_fim=g["y_fim"], furos_extra_xy=g["furos_chapa"])
    z_apoio = j.Z_HORN + j.T_CHAPA - j.REBAIXO
    pecas = {"chapa_horn": [ch], "chapa_idler": [ch.mirror("XY")],
             "ponte": [j.ponte()], "flange": [flange(g)],
             "parafusos_horn": [], "parafusos_ponte": [], "porcas": [],
             "parafusos_flange": [], "parafusos_m25": []}
    for s in (1, -1):
        for x, y in j.furos_horn_xy():
            pecas["parafusos_horn"].append(j.parafuso_m2(j.OPCAO_HORN["comprimento"], x, y, s * z_apoio, s))
        for x, y in j.furos_ponte_xy():
            pecas["parafusos_ponte"].append(j.parafuso_m2(j.L_PONTE, x, y, s * z_apoio, s))
            pecas["porcas"].append(j.porca_m2(x, s * (j.Z_ALOJ_BASE + j.H_ALOJ / 2.0)))
        for x, y in g["furos_chapa"]:
            pecas["parafusos_flange"].append(j.parafuso_m2(j.L_PONTE, x, y, s * z_apoio, s))
            pecas["porcas"].append(porca_m2_em(x, y, s * (j.Z_ALOJ_BASE + j.H_ALOJ / 2.0)))
    y_apoio_m25 = g["y0"] + OPCAO_M25["rebaixo"]
    for x, z in g["furos_m25"]:
        pecas["parafusos_m25"].append(parafuso_m25_y(x, y_apoio_m25, z))
    return g, pecas


# ---------------------------------------------------------------------------
# Verificações
# ---------------------------------------------------------------------------
# Esperados LITERAIS, calculados à mão (diário 2026-10-13):
#   chapa: [28,5·y_fim + pi·14,25²/2 - 2·(4-pi) - pi·4,5² - 14·pi·1,2²]·4 - 14·pi·(2,2²-1,2²)·0,5
#   flange: 28,5·15·38 - 4·[pi·1,45²·15 + pi·(2,75²-1,45²)·5,5]
#           - 8·[pi·1,2²·5,5 + (½hexa 8,006 + 6,25·4,3)·1,9 - pi·1,2²·1,9]
ESPERADO = {
    105.0: {"chapa_bbox": (-14.25, 14.25, -14.25, 69.75, 19.0, 23.0), "chapa_vol": 8637.94,
            "flange_bbox": (-14.25, 14.25, 54.75, 69.75, -19.0, 19.0)},
    90.0: {"chapa_bbox": (-14.25, 14.25, -14.25, 54.75, 19.0, 23.0), "chapa_vol": 6927.94,
           "flange_bbox": (-14.25, 14.25, 39.75, 54.75, -19.0, 19.0)},
}
FLANGE_VOL = 14810.83


def verificar(g, pecas, servo):
    ok = True
    e = ESPERADO[g["L"]]
    ch, fl = pecas["chapa_horn"][0], pecas["flange"][0]
    print("\n1. Caixa e volume (esperados à mão)")
    ok &= check_bbox("chapa", ch, e["chapa_bbox"])
    ok &= check_volume("chapa", ch, e["chapa_vol"], tol=0.2)
    ok &= check_bbox("flange", fl, e["flange_bbox"])
    ok &= check_volume("flange", fl, FLANGE_VOL, tol=0.2)

    print("\n2. Interfaces lidas do sólido")
    tabela = {(round(x, 2), round(y, 2)) for x, y in j.furos_horn_xy()}
    ok &= j.conferir("8 furos M2 do horn na chapa longa", tabela <= j.furos_no_solido(ch, j.D_FURO_M2))
    alvo = {(round(x, 2), round(y, 2)) for x, y in g["furos_chapa"]}
    ok &= j.conferir("furos chapa <-> flange coaxiais",
                     alvo <= (j.furos_no_solido(ch, j.D_FURO_M2) & j.furos_no_solido(fl, j.D_FURO_M2)))
    # Furos M2,5 do flange contra as roscas do servo da frente (STEP em (0, L, 0))
    furos_m25 = set()
    for f in fl.val().Faces():
        if f.geomType() == "CYLINDER" and abs(f._geomAdaptor().Cylinder().Radius() * 2 - D_FURO_M25) < 1e-3:
            p = f._geomAdaptor().Cylinder().Axis().Location()
            furos_m25.add((round(p.X(), 2), round(p.Z(), 2)))
    roscas = {(round(x, 2), round(z, 2)) for x, _, z in F_M25["centros"]}
    ok &= j.conferir("4 furos M2,5 do flange sobre a ponta longa do servo da frente", roscas <= furos_m25,
                     f"{len(roscas & furos_m25)}/4")

    print("\n3. Parafusos")
    ok &= j.conferir(f"flange -> servo M2,5x{OPCAO_M25['comprimento']}",
                     ENGAJ_MIN_M25 <= OPCAO_M25["engajamento"] <= PROF_ROSCA_M25 - FOLGA_FUNDO_M25,
                     f"rosca {OPCAO_M25['engajamento']} mm, presa {OPCAO_M25['presa']}, rebaixo {OPCAO_M25['rebaixo']}")

    print("\n4. Resistência (stall 4,1 N·m do servo da frente, FS mínimo 3)")
    T = rf.TORQUE_PROJETO_NM * 1000.0
    # chapas -> flange: momento em z repartido entre as 2 chapas, 4 parafusos por chapa;
    # força no parafuso mais longe do centro do grupo: F = (T/2)·r/Σr²
    r = math.hypot(j.X_PARAF_PONTE, DY_FILEIRAS / 2.0)
    F = (T / 2.0) * r / (4 * r * r)
    fs = rf.ESCOAMENTO_MPA / rf.pressao_no_furo(F, rf.D_PARAFUSO_MM, j.PRESA)
    ok &= j.conferir("furo da chapa no flange (esmagamento)", fs >= rf.FATOR_SEGURANCA_MIN, f"{F:.0f} N, FS {fs:.2f}")
    # flange -> servo: momento em z vira tração nos 2 M2,5 de um lado; apoio na borda oposta
    Ft = T / (j.X_PARAF_PONTE + j.W_CHAPA / 2.0) / 2.0
    d = pi.PARAFUSO_ISO4762["M2.5"]
    area = math.pi / 4.0 * (d["dk_max"] ** 2 - D_FURO_M25 ** 2)
    fs = rf.ESCOAMENTO_MPA / (Ft / area)
    ok &= j.conferir("cabeça M2,5 no PLA do flange (arrancamento)", fs >= rf.FATOR_SEGURANCA_MIN,
                     f"{Ft:.0f} N, FS {fs:.2f}")
    r2 = rf.avaliar(j.T_CHAPA, j.W_CHAPA, j.PRESA, 8)
    ok &= j.conferir("chapa ao longo do elo (seção cheia, flexão no plano)", r2["fs"]["flexao_secao_B"] >= rf.FATOR_SEGURANCA_MIN,
                     f"FS {r2['fs']['flexao_secao_B']:.2f}")

    print(f"\n5. Paredes (mínimo {j.PAREDE_MIN})")
    paredes = {
        "porca -> face do flange": D_FLANGE / 2 - DY_FILEIRAS / 2 - j.S_ALOJ / 2,
        "entre as porcas das 2 fileiras": DY_FILEIRAS - j.S_ALOJ,
        "rebaixo da chapa -> ponta da chapa": g["y_fim"] - g["filas"][1] - j.D_REBAIXO_M2 / 2,
        "rebaixo M2,5 -> furo M2 da chapa (em z)": (j.Z_HORN - j.PROF_FURO_PONTE) - 6.0 - D_REBAIXO_M25 / 2,
        "ponte -> flange (vão livre)": g["y0"] - j.Y_FIM,
    }
    for nome, v in paredes.items():
        ok &= j.conferir(nome, v >= j.PAREDE_MIN - 1e-9, f"{v:.2f} mm")

    print("\n6. Colisões nas poses (STEP oficial dos dois servos)")
    sv_tras = j.unir([servo.solids().vals()[0], servo.solids().vals()[17]])
    sv_frente = sv_tras.translate(cq.Vector(0, g["L"], 0))
    estrutura = j.unir(pecas["chapa_horn"] + pecas["chapa_idler"] + pecas["ponte"] + pecas["flange"]
                       + pecas["parafusos_ponte"] + pecas["parafusos_flange"] + pecas["porcas"])
    # 6a. o elo (com o servo da frente) gira em torno do servo de trás
    movel = estrutura.fuse(sv_frente)
    bate = [a for a in range(-90, 91, 15) if j.girar(movel, a).intersect(sv_tras).Volume() > 0.01]
    ok &= j.conferir("elo gira ±90° sem bater no servo de trás", not bate, str(bate) if bate else "")
    # 6b. a forquilha do elo SEGUINTE gira em torno do servo da frente sem bater neste elo
    jp = j.forquilha()
    seguinte = j.unir(jp["chapa_horn"] + jp["chapa_idler"] + jp["ponte"] + jp["parafusos_ponte"]
                      + jp["porcas"]).translate(cq.Vector(0, g["L"], 0))
    eixo = cq.Vector(0, g["L"], 0)

    def gira_seg(a):
        return seguinte.rotate(eixo, eixo + cq.Vector(0, 0, 1), a)

    bate = [a for a in range(-90, 91, 15) if gira_seg(a).intersect(estrutura).Volume() > 0.01]
    ok &= j.conferir("elo seguinte gira ±90° sem bater neste", not bate, str(bate) if bate else "")
    folga = min(gira_seg(a).distance(estrutura) for a in range(-90, 91, 10))
    ok &= j.conferir(f"folga mínima elo seguinte <-> este >= {j.FOLGA_PONTE}", folga >= j.FOLGA_PONTE, f"{folga:.2f} mm")
    return ok


def exportar(nome, g, pecas, servo):
    pasta = os.path.join(j.PROJETO, "cad_export", nome)
    os.makedirs(pasta, exist_ok=True)
    for chave, arq in (("chapa_horn", "chapa"), ("flange", "flange")):
        exportar_stl_mm(pecas[chave][0], pasta, f"{arq}.stl")
        cq.exporters.export(pecas[chave][0], os.path.join(pasta, f"{arq}.step"))
    mont = cq.Assembly(name=nome)
    mont.add(servo, name="servo_tras_referencia", color=cq.Color(*j.CORES["servo"]))
    mont.add(servo, name="servo_frente_referencia", loc=cq.Location(cq.Vector(0, g["L"], 0)),
             color=cq.Color(*j.CORES["servo"]))
    for chave, lista in pecas.items():
        cor = (j.CORES["chapa"] if chave.startswith("chapa") else j.CORES["ponte"] if chave in ("ponte", "flange")
               else j.CORES["porca"] if chave == "porcas" else j.CORES["parafuso"])
        for i, s in enumerate(lista):
            mont.add(s, name=f"{chave}_{i}", color=cq.Color(*cor))
    mont.export(os.path.join(pasta, f"{nome}_montagem.step"))
    print(f"  STEP: chapa, flange e {nome}_montagem em cad_export/{nome}/")


def imagem(nome, g, pecas, servo):
    from render_cad import render
    itens = [(s, j.CORES["servo"]) for s in servo.solids().vals()]
    itens += [(s.translate(cq.Vector(0, g["L"], 0)), j.CORES["servo"]) for s in servo.solids().vals()]
    for chave, lista in pecas.items():
        cor = (j.CORES["chapa"] if chave.startswith("chapa") else j.CORES["ponte"] if chave in ("ponte", "flange")
               else j.CORES["porca"] if chave == "porcas" else j.CORES["parafuso"])
        itens += [(s.val(), cor) for s in lista]
    pasta = os.path.join(j.PROJETO, "previews")
    os.makedirs(pasta, exist_ok=True)
    render(itens, os.path.join(pasta, f"{nome}.png"), foco=(0, g["L"] / 2.0, 0),
           posicao=(170, g["L"] / 2.0 + 120, 140), zoom=1.1)


def lista_de_materiais(nome, pecas):
    print(f"\nLista de materiais ({nome})")
    print("  2x chapa, 1x ponte, 1x flange (impressas, PLA)")
    for chave, desc in (("parafusos_horn", f"ISO 4762 M2x{j.OPCAO_HORN['comprimento']}"),
                        ("parafusos_ponte", f"ISO 4762 M2x{j.L_PONTE} (ponte)"),
                        ("parafusos_flange", f"ISO 4762 M2x{j.L_PONTE} (flange)"),
                        ("parafusos_m25", f"ISO 4762 M2,5x{OPCAO_M25['comprimento']} (flange -> servo)"),
                        ("porcas", "ISO 4032 M2")):
        print(f"  {len(pecas[chave])}x {desc}")


if __name__ == "__main__":
    pedidos = {k: v for k, v in COMPRIMENTOS.items() if len(sys.argv) < 2 or v == float(sys.argv[1])}
    servo = servo_step.carregar()
    for nome, L in pedidos.items():
        g, pecas = elo(L)
        print(f"\n=== {nome}: L = {L} mm, chapa até y = {g['y_fim']}, flange {g['y0']}..{g['y_fim']}, "
              f"M2,5x{OPCAO_M25['comprimento']} no servo ===")
        exigir_tudo_ok(verificar(g, pecas, servo))
        lista_de_materiais(nome, pecas)
        exportar(nome, g, pecas, servo)
        imagem(nome, g, pecas, servo)
