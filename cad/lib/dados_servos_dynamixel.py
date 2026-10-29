#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dados_servos_dynamixel.py
=========================
Dados de datasheet de servos DYNAMIXEL — ÚNICO lugar onde esses números
existem. Os scripts de CAD importam daqui.

Todos os valores foram lidos em 2026-10-01 das páginas do e-manual da
ROBOTIS (fabricante). Nada aqui é estimado, exceto o que está marcado
ASSUMIDO ou LIDO DE GRÁFICO.

A tensão de trabalho é escolha de cada projeto e fica no projeto. Os
valores de torque e velocidade abaixo são os publicados a 12,0 V.

Links de cada fonte: docs/sources.md. Os arquivos do fabricante (STEP, PDF)
não são redistribuídos: ficam em fontes/fabricante/, fora do Git.
"""

import math

SERVOS = {
    # -------------------------------------------------------------------
    # DYNAMIXEL XM430-W350-T
    # Fonte: https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/
    # -------------------------------------------------------------------
    "XM430-W350-T": {
        "fonte": "https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/",
        "massa_g": 82.0,                 # "Weight 82 [g]"
        "dim_mm": (28.5, 46.5, 34.0),    # "Dimensions (W x H x D)"
        "tensao_faixa_v": (10.0, 14.8),  # "Input Voltage 10.0 ~ 14.8 [V] (Recommended: 12.0 [V])"
        "torque_stall_nm": 4.1,          # "4.1 [N.m] (at 12.0 [V], 2.3 [A])"
        "corrente_stall_a": 2.3,
        "vel_sem_carga_rpm": 46.0,       # "46 [rev/min] (at 12.0 [V])"
        "resolucao_pulsos": 4096,        # "Resolution 4096 [pulse/rev]"
        "folga_graus": 0.25,             # "Backlash 15 [arcmin] (0.25 [°])"
        "faixa_posicao_graus": 360.0,    # "Position Control Mode (0 ~ 360 [°])"
        "carga_radial_n": 40.0,          # "Radial Load 40 [N] (10 [mm] away from the horn)"
        "carga_radial_dist_mm": 10.0,
        "carga_axial_n": 20.0,           # "Axial Load 20 [N]"
        # LIDO DE GRÁFICO (aproximado, a olho): fim da curva N-T do
        # "Performance Graph" da mesma página (cópia em
        # fontes/fabricante/xm430_w350_performance_graph.jpg). O gráfico não informa a
        # tensão do ensaio.
        "torque_max_grafico_nm": 3.0,
    },
    # -------------------------------------------------------------------
    # DYNAMIXEL XC430-W240-T
    # Fonte: https://emanual.robotis.com/docs/en/dxl/x/xc430-w240/
    # -------------------------------------------------------------------
    "XC430-W240-T": {
        "fonte": "https://emanual.robotis.com/docs/en/dxl/x/xc430-w240/",
        "massa_g": 65.0,                 # "Weight 65 [g]"
        "dim_mm": (28.5, 46.5, 34.0),    # "Dimensions (W x H x D)"
        "tensao_faixa_v": (6.5, 14.8),   # "Input Voltage 6.5 ~ 14.8 [V] (Recommended: 12.0 [V])"
        "torque_stall_nm": 1.9,          # "1.9 [N.m] (at 12.0 [V], 1.4 [A])"
        "corrente_stall_a": 1.4,
        "vel_sem_carga_rpm": 70.0,       # "70 [rev/min] (at 12.0 [V])"
        "resolucao_pulsos": 4096,
        # A página do XC430 NÃO publica folga (backlash) nem carga radial ou
        # axial. None = dado inexistente, não zero.
        "folga_graus": None,
        "faixa_posicao_graus": 360.0,    # "Position Control Mode (0 ~ 360 [°])"
        "carga_radial_n": None,
        "carga_radial_dist_mm": None,
        "carga_axial_n": None,
        # LIDO DE GRÁFICO (aproximado): fontes/fabricante/xc430_w240_performance_graph.jpg
        "torque_max_grafico_nm": 1.14,
    },
}

# ---------------------------------------------------------------------------
# Geometria do formato X430 (igual pros dois modelos: o e-manual lista o
# XC430 como "same form factor" do XM430, e a caixa W x H x D é idêntica).
# ---------------------------------------------------------------------------
# A caixa externa vem da tabela do datasheet. A posição do eixo e o horn
# NÃO estão na tabela: foram medidos na malha oficial da ROBOTIS (XM430 do
# OpenManipulator-X), e conferidos depois contra o STEP oficial do X430
# (fontes/medir_x430_step.py). Para o XC430 a posição do eixo é herdada do formato,
# não foi conferida num desenho do próprio XC430.
X430_LARGURA = 28.5        # W (mm), datasheet
X430_COMPRIMENTO = 46.5    # H (mm), datasheet — lado comprido, o eixo fica deslocado nele
X430_PROFUNDIDADE = 34.0   # D (mm), datasheet — medida ao longo do eixo de rotação
X430_EIXO_A_PONTA_CURTA = 11.25   # mm do eixo até a ponta mais próxima (medido na malha oficial)
X430_EIXO_A_PONTA_LONGA = 35.25   # mm do eixo até a ponta mais distante (46,5 - 11,25)
X430_HORN_DIAMETRO = 19.5         # mm (medido na malha oficial)
X430_HORN_ALTURA = 2.0            # mm que o horn sobe acima da face do corpo (medido)
# Os dois cantos da ponta CURTA são chanfrados a 45°, 2 x 2 mm, na altura toda
# do corpo (STEP oficial "XM,H-430_idler.stp"; conferido por
# fontes/medir_x430_step.py, item 6). A caixa
# W x H x D acima é só o envelope: o canto vivo NÃO existe na peça.
X430_CHANFRO_PONTA_CURTA = 2.0    # mm (cateto do chanfro de 45°)


def x430_raio_max_ponta_curta():
    """Maior distância do eixo a um ponto do corpo do lado da ponta curta (mm),
    no plano perpendicular ao eixo. Com o chanfro, é a ponta do chanfro na
    lateral: (W/2, ponta_curta - chanfro) -> hypot(14,25; 9,25) = 16,99.
    Sem o chanfro (caixa) seria hypot(14,25; 11,25) = 18,16."""
    return math.hypot(X430_LARGURA / 2.0, X430_EIXO_A_PONTA_CURTA - X430_CHANFRO_PONTA_CURTA)

# Regra de dimensionamento da própria ROBOTIS: "Stable motions are possible
# with robots designed for loads with 1/5 or less of the stall torque."
# Fonte: https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/ (nota da tabela
# de especificações). A nota está na série AX; as páginas da série X dizem
# só que o desempenho real fica mais perto do gráfico N-T que do stall.
FRACAO_STALL_ESTAVEL = 1.0 / 5.0


def vel_rad_s(modelo):
    """Velocidade sem carga a 12 V (a do datasheet), convertida de rev/min pra rad/s."""
    return SERVOS[modelo]["vel_sem_carga_rpm"] * 2.0 * math.pi / 60.0


def resolucao_graus(modelo):
    """Menor passo de posição: 360° dividido pelos pulsos por volta."""
    return 360.0 / SERVOS[modelo]["resolucao_pulsos"]



# ---------------------------------------------------------------------------
# Furação do formato X430 (XM430/XH430, versão com idler)
# ---------------------------------------------------------------------------
# Fonte: desenho oficial ROBOTIS "XM430/XH430 (HINGE)", 2019/03/19, e o STEP
# do mesmo modelo, baixados em 2026-10-10 (fontes/fabricante/, fora do Git;
# origem e SHA-256 em docs/sources.md). Cada número abaixo foi
# DIGITADO a partir das cotas do PDF; o script fontes/medir_x430_step.py
# confere que o STEP tem exatamente estes furos.
#
# Frame do servo (é o mesmo do STEP):
#   origem no eixo de rotação, no meio da espessura de 34 mm;
#   +z = eixo, apontando para o horn (face do horn em z = +17, idler em -17);
#   +y = ao longo do lado comprido, para a ponta CURTA (y = +11,25; a ponta
#        longa fica em y = -35,25);
#   x  = largura (-14,25 .. +14,25).
# Furos com rosca: centro do furo NA FACE, eixo do furo, rosca e
# profundidade máxima de rosca do desenho ("TAP DPx(Max.)").

X430_FURACAO = {
    # "8-M2x.4 TAP DP2.0(Max.)  P.C.D Ø16", no horn (face de fora em z = +19)
    "horn_m2": {"rosca": "M2x0.4", "prof_max_mm": 2.0, "eixo": "+z", "z_face": 19.0,
                "pcd_mm": 16.0, "n": 8, "angulo_inicial_graus": 0.0},
    # mesmo padrão no idler (face de fora em z = -19)
    "idler_m2": {"rosca": "M2x0.4", "prof_max_mm": 2.0, "eixo": "-z", "z_face": -19.0,
                 "pcd_mm": 16.0, "n": 8, "angulo_inicial_graus": 0.0},
    # "2-M2.5x.45 TAP DP3.0(Max.)" na face do horn: x = ±11 (cota 22),
    # y = -8 (cota 8 abaixo do eixo)
    "face_horn_m25": {"rosca": "M2.5x0.45", "prof_max_mm": 3.0, "eixo": "+z",
                      "centros": [(-11.0, -8.0, 17.0), (11.0, -8.0, 17.0)]},
    # "4-M2.5x.45 TAP DP3.0(Max.)" em CADA lateral (x = ±14,25): cotas 4 e 24
    # -> y = -4 e -28; cotas 11 e 12 a partir da face do horn -> z = +6 e -6
    "lateral_m25": {"rosca": "M2.5x0.45", "prof_max_mm": 3.0, "eixo": "±x",
                    "centros": [(s * 14.25, y, z) for s in (-1, 1)
                                for y in (-4.0, -28.0) for z in (6.0, -6.0)]},
    # "4-M2.5x.45 TAP DP3.0(Max.)" na ponta longa (y = -35,25): cota 16 -> x = ±8;
    # cotas 11 e 12 -> z = +6 e -6
    "ponta_longa_m25": {"rosca": "M2.5x0.45", "prof_max_mm": 3.0, "eixo": "-y",
                        "centros": [(x, -35.25, z) for x in (-8.0, 8.0) for z in (6.0, -6.0)]},
    # "2-M2.5x.45 TAP DP3.0(MAX.)" na ponta curta (y = +11,25): cota 11 -> z = +6
    "ponta_curta_m25": {"rosca": "M2.5x0.45", "prof_max_mm": 3.0, "eixo": "+y",
                        "centros": [(-8.0, 11.25, 6.0), (8.0, 11.25, 6.0)]},
    # "4-WB M2.5x12 SCREW" nos cantos das duas faces (cotas 22 e 40, 11,25 e 8):
    # são os parafusos que FECHAM a carcaça, com rebaixo Ø4,8 x 2,8. Não são
    # furo livre. Usar como fixação (trocando por parafuso mais longo) NÃO
    # está no desenho: não fazer sem outra fonte da ROBOTIS.
    "parafusos_carcaca": {"parafuso": "WB M2.5x12", "rebaixo_d_mm": 4.8, "rebaixo_prof_mm": 2.8,
                          "centros_xy": [(-11.0, 8.0), (11.0, 8.0), (-11.0, -32.0), (11.0, -32.0)]},
}
X430_HORN_SALIENCIA_CENTRAL = {"diametro_mm": 8.0, "altura_mm": 2.2}   # "Ø8" e "2.2" do desenho


def x430_centros_pcd(nome):
    """Centros (x, y, z) dos furos de um padrão em círculo de X430_FURACAO."""
    f = X430_FURACAO[nome]
    r = f["pcd_mm"] / 2.0
    return [(round(r * math.cos(math.radians(f["angulo_inicial_graus"] + 360.0 * i / f["n"])), 6),
             round(r * math.sin(math.radians(f["angulo_inicial_graus"] + 360.0 * i / f["n"])), 6),
             f["z_face"]) for i in range(f["n"])]
