#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
garra.py
========
Mão (link do punho) + garra de mordentes paralelos por pinhão e cremalheira
(decisões 2026-10-13: tipo C, m = 2, z = 18, face 8; abertura 0..40 mm).

Dois frames:
  M (mão) = frame do servo do PUNHO (XC430): origem no eixo do punho, +z_s
      eixo, +y_s para a frente. As chapas da mão são as do junta.py (em
      z = ±19..23), presas no horn e no idler do punho, SEM ponte: o corpo do
      servo da garra, preso pelas 8 roscas laterais, é o espaçador.
  G (garra) = X ao longo do curso dos mordentes, Y "para cima" (= x_s),
      Z para a frente (= y_s); origem no eixo do servo da garra, Z = 0 na face
      do horn dele. G -> M: giro de -120° em torno de (1,1,1) (X->z_s, Y->x_s,
      Z->y_s) + translação de Y0 + 19 em y_s.
O servo da garra fica com o eixo para a frente, a ponta curta para cima e a
ponta longa para baixo. O pinhão vai no horn dele; a cremalheira de cima
(dentes para baixo) e a de baixo (dentes para cima) andam em sentidos
opostos ao longo de X. Cada cremalheira corre num trilho de uma GUIA presa no
servo (a de cima na ponta curta, 2× M2,5; a de baixo na ponta longa, 4× M2,5),
com lábio atrás e na frente. Cada MORDENTE (conector + dedo) é parafusado na
cremalheira dele com 2 M2 e porcas embutidas na cremalheira.

Cinemática: θ = giro do pinhão a partir da garra ABERTA (40 mm); a abertura é
g = 40 - 2·r·θ, com r = 18 (raio primitivo). Fecha em θ = 40/36 rad = 63,7°.

Uso (de dentro de cad/):
    python garra.py
"""

import math
import os

import usar_lib  # noqa: F401
import cadquery as cq
from OCP.BRepAdaptor import BRepAdaptor_Surface
import dados_servos_dynamixel as sv
from check_bbox import check_bbox, exigir_tudo_ok
from exportar_stl_mm import exportar_stl_mm

import elo as e
import engrenagem as eg
import junta as j
import parafusos_iso as pi
import resistencia_forquilha as rf
import servo_step

# ---------------------------------------------------------------------------
# Parâmetros
# ---------------------------------------------------------------------------
SERVO_GARRA = "XC430-W240-T"
T_STALL = sv.SERVOS[SERVO_GARRA]["torque_stall_nm"] * 1000.0   # 1900 N·mm (carga = stall)
M_ENG, Z_ENG, B_ENG = 2.0, 18, 8.0                             # decisão (engrenagem.py)
R = M_ENG * Z_ENG / 2.0                                        # 18: raio primitivo
P = math.pi * M_ENG
FOLGA_DENTE = 0.1        # ASSUMIDO, mm por flanco (o teste mede 0,188 de folga no engrenamento)
ABERTURA = 40.0          # Daniel, 2026-10-13
THETA_MAX = ABERTURA / (2 * R)

# Posição do servo da garra no frame da mão: centro em y_s = Y0. O idler dele
# (disco Ø19,5 em y_s = Y0 - 19) tem que ficar fora do raio do corpo do punho
# (16,99) + folga 0,5 para a mão girar: Y0 - 19 >= 17,49 -> Y0 = 37.
Y0 = math.ceil(sv.x430_raio_max_ponta_curta() + j.FOLGA_PONTE + 19.0)   # 37
Y_FIM = Y0 + 10.0        # ponta da chapa da mão (cobre os furos laterais em Y0 ± 6)
CALCO = j.Z_HORN - sv.X430_LARGURA / 2.0   # 4,75: da face interna da chapa à lateral do servo
X_CHAPA_MIN = -33.0      # a chapa desce até cobrir o furo lateral de baixo (x_s = -28)

FOLGA_GUIA = 0.2         # ASSUMIDO: cremalheira <-> trilho e lábios
T_LABIO = 1.5            # lábio de trás e da frente da guia
T_BRACO_GUIA = 3.5       # braço da guia que assenta na ponta do servo
H_CORPO_CREM = 8.0       # corpo da cremalheira (cabe a porca M2 com parede 1,5)
Y_CREM_TOPO = R + 1.25 * M_ENG + H_CORPO_CREM              # 28,5
T_CREM = B_ENG + 0.5                                         # 8,5: 0,5 à frente do pinhão
X_CREM_FIM = 23.0        # a cremalheira vai até aqui (coordenada do mordente): cobre o parafuso em 20
X_PARAF_MORDENTE = (4.0, 20.0)   # parafusos mordente -> cremalheira (alavanca 16)
Y_PARAF_MORDENTE = R + 1.25 * M_ENG + H_CORPO_CREM / 2.0   # 24,5: meio do corpo
DEDO_T, DEDO_L, DEDO_W = 6.0, 30.0, 24.0   # espessura (flexão: conta abaixo), comprimento, largura
T_CONECTOR = 4.0
X_CONECTOR = 22.0

# Opções de parafuso (mesmas regras dos outros módulos)
OPC_LAT = max((o for o in pi.rebaixo_para_rosca_cega("M2.5", j.T_CHAPA + CALCO, 3.0, e.FOLGA_FUNDO_M25,
                                                       e.ENGAJ_MIN_M25, e.PRESA_MIN_M25)
               if o["saliencia_cabeca"] <= 0), key=lambda o: o["presa"])
OPC_PINHAO = max((o for o in pi.rebaixo_para_rosca_cega("M2", B_ENG, j.PROF_ROSCA_HORN, j.FOLGA_FUNDO_HORN,
                                                          j.ENGAJ_MIN_HORN, 3.0)
                  if o["saliencia_cabeca"] <= 0), key=lambda o: o["presa"])
OPC_GUIA = pi.rebaixo_para_rosca_cega("M2.5", T_BRACO_GUIA, 3.0, e.FOLGA_FUNDO_M25, e.ENGAJ_MIN_M25,
                                      T_BRACO_GUIA)[0]


def caixa(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def g_para_m(s):
    """Frame G (garra) -> frame M (mão)."""
    s = s.val() if hasattr(s, "val") else s
    return s.rotate(cq.Vector(), cq.Vector(1, 1, 1), -120).translate(cq.Vector(0, Y0 + 19.0, 0))


def servo_em_g(s):
    """Sólido do STEP (frame do servo) -> frame G: z_g = Z + 19."""
    return s.translate(cq.Vector(0, 0, -19.0))


# ---------------------------------------------------------------------------
# Peças no frame G
# ---------------------------------------------------------------------------
def pinhao():
    p = cq.Workplane("XY").polyline(eg.perfil_pinhao(M_ENG, Z_ENG, FOLGA_DENTE)).close().extrude(B_ENG)
    frente = cq.Workplane("XY", origin=(0, 0, B_ENG))
    pts = j.furos_horn_xy()
    p = p.cut(frente.pushPoints(pts).circle(j.D_FURO_M2 / 2).extrude(-B_ENG))
    p = p.cut(frente.pushPoints(pts).circle(j.D_REBAIXO_M2 / 2).extrude(-OPC_PINHAO["rebaixo"]))
    return p.cut(frente.circle(j.D_FURO_CENTRAL / 2).extrude(-B_ENG))


def cremalheira():
    """Cremalheira de CIMA no frame do mordente (x = 0 na face do dedo):
    primitiva em Y = R, dentes para baixo. O dente que encontra o vão do
    pinhão na garra aberta fica em x = -ABERTURA/2 (então, aberta, ele cai em X = 0)."""
    n = 6
    perfil = eg.perfil_cremalheira(M_ENG, n, H_CORPO_CREM, FOLGA_DENTE)   # dentes +y, primitiva y = 0
    x_dente0 = -ABERTURA / 2.0 - P                                       # dentes em -20 + k·p, k = -1..4
    pts = [(x + x_dente0, R - y) for x, y in perfil]                      # espelha: dentes para -Y
    c = cq.Workplane("XY").polyline(pts).close().extrude(T_CREM)
    x_fim_dentes = x_dente0 + (n - 1) * P + P / 2
    c = c.union(caixa(x_fim_dentes - 0.01, X_CREM_FIM, R + 1.25 * M_ENG, Y_CREM_TOPO, 0, T_CREM))
    # porcas M2 embutidas, entram por trás (prisma sextavado até a face Z = 0)
    z_porca_topo = T_CREM - j.PAREDE_SOBRE_PORCA
    for x in X_PARAF_MORDENTE:
        c = c.cut(cq.Workplane("XY", origin=(x, Y_PARAF_MORDENTE, 0)).polygon(6, j.E_ALOJ)
                  .extrude(z_porca_topo).rotate((x, Y_PARAF_MORDENTE, 0), (x, Y_PARAF_MORDENTE, 1), 90))
        c = c.cut(cq.Workplane("XY", origin=(x, Y_PARAF_MORDENTE, 0)).circle(j.D_FURO_M2 / 2).extrude(T_CREM))
    return c


def mordente():
    """Mordente de CIMA, frame do mordente (face de aperto do dedo em x = 0)."""
    z0 = T_CREM                                                   # encosta na frente da cremalheira
    con = caixa(0, X_CONECTOR, -DEDO_W / 2, Y_PARAF_MORDENTE + 2.0, z0, z0 + T_CONECTOR)
    dedo = caixa(0, DEDO_T, -DEDO_W / 2, DEDO_W / 2, z0 + T_CONECTOR, z0 + T_CONECTOR + DEDO_L)
    m = con.union(dedo)
    frente = cq.Workplane("XY", origin=(0, 0, z0 + T_CONECTOR))
    pts = [(x, Y_PARAF_MORDENTE) for x in X_PARAF_MORDENTE]
    m = m.cut(frente.pushPoints(pts).circle(j.D_FURO_M2 / 2).extrude(-T_CONECTOR))
    return m.cut(frente.pushPoints(pts).circle(j.D_REBAIXO_M2 / 2).extrude(-j.REBAIXO))


def _trilho(x0, x1):
    """Trilho de CIMA: bloco acima da cremalheira + lábio da frente."""
    y_base = Y_CREM_TOPO + FOLGA_GUIA
    bloco = caixa(x0, x1, y_base, y_base + 3.3, -2.0, T_CREM + FOLGA_GUIA + T_LABIO)
    labio = caixa(x0, x1, Y_CREM_TOPO - T_LABIO, y_base, T_CREM + FOLGA_GUIA, T_CREM + FOLGA_GUIA + T_LABIO)
    return bloco.union(labio)


def guia_cima():
    """Braço na ponta curta (2× M2,5) + lábio de trás + trilho."""
    meia = sv.X430_LARGURA / 2.0
    y_curta = sv.X430_EIXO_A_PONTA_CURTA
    z_labio1 = -FOLGA_GUIA
    braco = caixa(-meia, meia, y_curta, y_curta + T_BRACO_GUIA, -17.0, -2.0)
    labio = caixa(-meia, meia, y_curta, Y_CREM_TOPO + FOLGA_GUIA + 3.3, -2.0, z_labio1)
    g = braco.union(labio).union(_trilho(-ABERTURA / 2 - P * 1.5 - 1.5, X_CREM_FIM + ABERTURA / 2 + 2.0))
    furos = [(x, z - 19.0) for x, _, z in sv.X430_FURACAO["ponta_curta_m25"]["centros"]]
    topo = cq.Workplane("XZ", origin=(0, y_curta + T_BRACO_GUIA, 0))       # normal -Y: corta para baixo
    return g.cut(topo.pushPoints(furos).circle(e.D_FURO_M25 / 2).extrude(T_BRACO_GUIA))   # no plano XZ, y local = +Z


def guia_baixo():
    """Braço na ponta longa (4× M2,5) + lábio de trás + trilho (o de cima girado 180°)."""
    meia = sv.X430_LARGURA / 2.0
    y_longa = -sv.X430_EIXO_A_PONTA_LONGA
    trilho = _trilho(-ABERTURA / 2 - P * 1.5 - 1.5, X_CREM_FIM + ABERTURA / 2 + 2.0).rotate((0, 0, 0), (0, 0, 1), 180)
    braco = caixa(-meia, meia, y_longa - T_BRACO_GUIA, y_longa, -29.0, -2.0)
    labio = caixa(-meia, meia, y_longa - T_BRACO_GUIA, -(Y_CREM_TOPO - T_LABIO), -2.0, -FOLGA_GUIA)
    g = braco.union(labio).union(trilho)
    furos = [(x, z - 19.0) for x, _, z in sv.X430_FURACAO["ponta_longa_m25"]["centros"]]
    fundo = cq.Workplane("XZ", origin=(0, y_longa - T_BRACO_GUIA, 0))
    return g.cut(fundo.pushPoints(furos).circle(e.D_FURO_M25 / 2).extrude(-T_BRACO_GUIA))


def chapa_mao():
    """Chapa da mão no frame M: a do junta.py sem ponte + descida até o furo
    lateral de baixo + calço até a lateral do servo da garra."""
    ch = j.chapa(y_fim=Y_FIM, com_ponte=False)
    ch = ch.union(caixa(X_CHAPA_MIN, j.W_CHAPA / 2, Y0 - 10.0, Y_FIM, j.Z_HORN, j.Z_HORN + j.T_CHAPA))
    ch = ch.union(caixa(X_CHAPA_MIN + 1.0, 0.0, Y0 - 10.0, Y_FIM, j.Z_HORN - CALCO, j.Z_HORN))
    pts = [(y, Y0 + z) for x, y, z in sv.X430_FURACAO["lateral_m25"]["centros"] if x > 0]   # x_s = y_g, y_s = Y0 + z_g
    fora = cq.Workplane("XY", origin=(0, 0, j.Z_HORN + j.T_CHAPA))
    ch = ch.cut(fora.pushPoints(pts).circle(e.D_FURO_M25 / 2).extrude(-(j.T_CHAPA + CALCO)))
    return ch.cut(fora.pushPoints(pts).circle(e.D_REBAIXO_M25 / 2).extrude(-OPC_LAT["rebaixo"]))


def mecanismo(theta):
    """Peças MÓVEIS da garra no frame G para o giro θ do pinhão (0 = aberta)."""
    g = ABERTURA - 2 * R * theta
    rot = math.degrees(theta) + 90.0 + 180.0 / Z_ENG          # vão do pinhão em +Y com θ = 0 (ver o teste)
    pin = pinhao().rotate((0, 0, 0), (0, 0, 1), rot)
    crem_c = cremalheira().translate((g / 2, 0, 0))
    mord_c = mordente().translate((g / 2, 0, 0))
    gira = lambda s: s.rotate((0, 0, 0), (0, 0, 1), 180)
    return {"pinhao": pin, "cremalheira_cima": crem_c, "cremalheira_baixo": gira(crem_c),
            "mordente_cima": mord_c, "mordente_baixo": gira(mord_c)}, g


def fixas_g():
    return {"guia_cima": guia_cima(), "guia_baixo": guia_baixo()}


# ---------------------------------------------------------------------------
# Verificações
# ---------------------------------------------------------------------------
def verificar(servo):
    ok = True
    sol = servo.solids().vals()
    ch = chapa_mao()
    fix = fixas_g()
    mov0, _ = mecanismo(0.0)

    print("\n1. Caixas (esperados à mão)")
    ok &= check_bbox("chapa da mão", ch, (-33.0, 14.25, -14.25, 47.0, 14.25, 23.0))
    ok &= check_bbox("cremalheira (frame do mordente, -20-1,5p)", cremalheira(), (-29.42, 23.0, 16.0, 28.5, 0.0, 8.5))
    ok &= check_bbox("mordente", mordente(), (0.0, 22.0, -12.0, 26.5, 8.5, 42.5))
    ok &= check_bbox("guia de cima", fix["guia_cima"], (-30.92, 45.0, 11.25, 32.0, -17.0, 10.2))   # -20 - 1,5p - 1,5

    print("\n2. Interfaces lidas do sólido")
    pts = {(round(x, 2), round(y, 2)) for x, y in j.furos_horn_xy()}
    ok &= j.conferir("8 furos do pinhão sobre o horn da garra", pts <= j.furos_no_solido(pinhao(), j.D_FURO_M2))
    lat = {(round(y, 2), round(Y0 + z, 2)) for x, y, z in sv.X430_FURACAO["lateral_m25"]["centros"] if x > 0}
    ok &= j.conferir("4 furos da chapa sobre as roscas laterais do servo da garra",
                     lat <= j.furos_no_solido(ch, e.D_FURO_M25), f"{len(lat & j.furos_no_solido(ch, e.D_FURO_M25))}/4")
    # parafusos das guias: cada furo tem que cair numa rosca do STEP (lida do sólido do servo)
    roscas = {}
    for f in servo_em_g(sol[0]).Faces():
        if f.geomType() == "CYLINDER":
            c = BRepAdaptor_Surface(f.wrapped).Cylinder()
            if abs(2 * c.Radius() - 2.05) < 1e-3:
                p = c.Axis().Location()
                roscas[(round(p.X(), 1), round(p.Z(), 1))] = True
    guia_xz = [(x, round(z - 19.0, 1)) for x, _, z in sv.X430_FURACAO["ponta_curta_m25"]["centros"]] + \
              [(x, round(z - 19.0, 1)) for x, _, z in sv.X430_FURACAO["ponta_longa_m25"]["centros"]]
    ok &= j.conferir("6 furos das guias sobre roscas M2,5 do STEP", all((round(x, 1), z) in roscas for x, z in guia_xz))

    print("\n3. Parafusos")
    for nome, o in (("chapa+calço -> servo da garra M2,5", OPC_LAT), ("pinhão -> horn M2", OPC_PINHAO),
                    ("guias -> servo M2,5", OPC_GUIA)):
        ok &= j.conferir(f"{nome}x{o['comprimento']}", o["engajamento"] >= (e.ENGAJ_MIN_M25 if "2,5" in nome else j.ENGAJ_MIN_HORN),
                         f"rosca {o['engajamento']}, presa {o['presa']}, rebaixo {o['rebaixo']}")
    ponta = T_CREM + T_CONECTOR - j.REBAIXO - j.L_PONTE
    ok &= j.conferir(f"mordente -> cremalheira M2x{j.L_PONTE}: passa a porca",
                     (T_CREM - j.PAREDE_SOBRE_PORCA - j.PORCA["m_max"]) - ponta >= j.PASSA_PORCA - 1e-9 and ponta >= 0,
                     f"ponta em Z = {ponta:.1f}, porca de Z = {T_CREM - j.PAREDE_SOBRE_PORCA - j.PORCA['m_max']:.1f} "
                     f"a {T_CREM - j.PAREDE_SOBRE_PORCA:.1f}")

    print(f"\n4. Resistência (stall do {SERVO_GARRA}: {T_STALL / 1000} N·m; FS mínimo 3)")
    f, s = eg.tensao_dente(T_STALL, M_ENG, Z_ENG, B_ENG)
    ok &= j.conferir("dente do pinhão (Lewis)", rf.ESCOAMENTO_MPA / s >= 3, f"{f:.1f} N por mordente, FS {rf.ESCOAMENTO_MPA / s:.2f}")
    sig = 6 * f * DEDO_L / (DEDO_W * DEDO_T ** 2)
    ok &= j.conferir("dedo em flexão (força na ponta)", rf.ESCOAMENTO_MPA / sig >= 3, f"FS {rf.ESCOAMENTO_MPA / sig:.2f}")
    # mordente -> cremalheira: força do dedo no meio do dedo, alavanca até a face do conector;
    # tração nos 2 parafusos com apoio na borda do conector
    braco = T_CONECTOR + DEDO_L / 2
    d = [X_CONECTOR - x for x in X_PARAF_MORDENTE]
    ft = f * braco * max(d) / sum(x * x for x in d)
    area = math.pi / 4 * (pi.PARAFUSO_ISO4762["M2"]["dk_max"] ** 2 - j.D_FURO_M2 ** 2)
    ok &= j.conferir("cabeça M2 do mordente no PLA", rf.ESCOAMENTO_MPA / (ft / area) >= 3,
                     f"{ft:.0f} N, FS {rf.ESCOAMENTO_MPA / (ft / area):.2f}")
    fp = T_STALL / (8 * 8.0)
    ok &= j.conferir("furos do pinhão no horn (esmagamento)", rf.ESCOAMENTO_MPA / (fp / (2.0 * OPC_PINHAO["presa"])) >= 3,
                     f"FS {rf.ESCOAMENTO_MPA / (fp / (2.0 * OPC_PINHAO['presa'])):.1f}")

    print("\n5. Mecanismo de 0 a 40 mm (sólidos de verdade)")
    servo_g = j.unir([servo_em_g(sol[0]), servo_em_g(sol[17])])
    fixos = j.unir([fix["guia_cima"], fix["guia_baixo"], servo_g])
    bate, gs = [], []
    for k in range(9):
        th = THETA_MAX * k / 8
        mv, g = mecanismo(th)
        gs.append(g)
        pares = [("pinhao", "cremalheira_cima"), ("pinhao", "cremalheira_baixo"),
                 ("mordente_cima", "mordente_baixo"), ("cremalheira_cima", "mordente_baixo"),
                 ("cremalheira_baixo", "mordente_cima")]
        for a, b in pares:
            if mv[a].val().intersect(mv[b].val()).Volume() > 0.01:
                bate.append((round(g, 1), a, b))
        for nome in ("cremalheira_cima", "cremalheira_baixo", "mordente_cima", "mordente_baixo"):
            if mv[nome].val().intersect(fixos).Volume() > 0.01:
                bate.append((round(g, 1), nome, "fixos"))
        if mv["pinhao"].val().intersect(j.unir([fix["guia_cima"], fix["guia_baixo"]])).Volume() > 0.01:
            bate.append((round(g, 1), "pinhao", "guias"))
    ok &= j.conferir("nada se cruza em 9 posições de 40 a 0 mm", not bate, str(bate[:6]) if bate else "")
    mv, _ = mecanismo(0.0)
    ab = mv["mordente_cima"].val().distance(mv["mordente_baixo"].val())
    ok &= j.conferir("aberta: 40 mm entre as faces dos dedos (medido)", abs(ab - ABERTURA) < 0.05, f"{ab:.2f} mm")
    mv, _ = mecanismo(THETA_MAX)
    fe = mv["mordente_cima"].val().distance(mv["mordente_baixo"].val())
    ok &= j.conferir("fechada: dedos encostam (medido)", fe < 0.05, f"{fe:.3f} mm em θ = {math.degrees(THETA_MAX):.1f}°")

    print("\n6. A mão gira ±90° no punho sem bater no servo do punho")
    punho = j.unir([sol[0], sol[17]])
    mv, _ = mecanismo(0.0)
    mao = j.unir([ch, ch.mirror("XY")] + [g_para_m(s) for s in list(fix.values()) + list(mv.values())]
                 + [g_para_m(servo_em_g(sol[0]))])
    bate = [a for a in range(-90, 91, 15) if j.girar(mao, a).intersect(punho).Volume() > 0.01]
    ok &= j.conferir("sem colisão de -90° a +90°", not bate, str(bate) if bate else "")
    folga = min(j.girar(mao, a).distance(sol[0]) for a in range(-90, 91, 10))
    print(f"  (informativo) menor distância mão <-> corpo do punho: {folga:.2f} mm (inclui chapas no horn: 0 é encostar)")
    return ok


def lista_de_materiais():
    print("\nLista de materiais (mão + garra)")
    print("  2x chapa da mão, 1x pinhão, 2x cremalheira, 2x mordente, 1x guia de cima, 1x guia de baixo (PLA)")
    print(f"  16x ISO 4762 M2x{j.OPCAO_HORN['comprimento']} (chapas -> horn e idler do punho)")
    print(f"  8x ISO 4762 M2,5x{OPC_LAT['comprimento']} (chapas -> laterais do servo da garra)")
    print(f"  8x ISO 4762 M2x{OPC_PINHAO['comprimento']} (pinhão -> horn da garra)")
    print(f"  6x ISO 4762 M2,5x{OPC_GUIA['comprimento']} (guias -> pontas do servo da garra)")
    print(f"  4x ISO 4762 M2x{j.L_PONTE} + 4x porca ISO 4032 M2 (mordentes -> cremalheiras)")


def exportar(servo):
    pasta = os.path.join(j.PROJETO, "cad_export", "garra")
    os.makedirs(pasta, exist_ok=True)
    pecas = {"chapa_mao": chapa_mao(), "pinhao": pinhao(), "cremalheira": cremalheira(),
             "mordente": mordente(), **fixas_g()}
    for nome, s in pecas.items():
        exportar_stl_mm(s, pasta, f"{nome}.stl")
        cq.exporters.export(s, os.path.join(pasta, f"{nome}.step"))
    mv, _ = mecanismo(THETA_MAX / 2)
    mont = cq.Assembly(name="garra")
    sol = servo.solids().vals()
    mont.add(servo, name="servo_punho_referencia", color=cq.Color(*j.CORES["servo"]))
    mont.add(g_para_m(servo_em_g(sol[0])), name="servo_garra_referencia", color=cq.Color(*j.CORES["servo"]))
    mont.add(pecas["chapa_mao"], name="chapa_mao_horn", color=cq.Color(*j.CORES["chapa"]))
    mont.add(pecas["chapa_mao"].mirror("XY"), name="chapa_mao_idler", color=cq.Color(*j.CORES["chapa"]))
    for nome, s in list(fixas_g().items()) + list(mv.items()):
        cor = j.CORES["ponte"] if "guia" in nome else j.CORES["porca"] if "pinhao" in nome or "cremalheira" in nome else j.CORES["chapa"]
        mont.add(g_para_m(s), name=nome, color=cq.Color(*cor))
    mont.export(os.path.join(pasta, "garra_montagem.step"))
    print("  STEP: peças e garra_montagem (meio aberta, 20 mm) em cad_export/garra/")


def imagens(servo):
    from render_cad import render
    sol = servo.solids().vals()
    pasta = os.path.join(j.PROJETO, "previews")
    for th, nome in ((0.0, "garra_aberta.png"), (THETA_MAX, "garra_fechada.png")):
        mv, _ = mecanismo(th)
        itens = [(s, j.CORES["servo"]) for s in sol]
        itens += [(g_para_m(servo_em_g(s)), j.CORES["servo"]) for s in sol]
        itens += [(chapa_mao(), j.CORES["chapa"]), (chapa_mao().mirror("XY"), j.CORES["chapa"])]
        itens += [(g_para_m(s), j.CORES["ponte"]) for s in fixas_g().values()]
        itens += [(g_para_m(s), (0.85, 0.2, 0.2) if "mordente" in k else (0.75, 0.75, 0.70)) for k, s in mv.items()]
        render(itens, os.path.join(pasta, nome), foco=(0, 60, 0), posicao=(140, 190, 120), zoom=1.15)


if __name__ == "__main__":
    print(f"Servo da garra em y_s = {Y0}; pinhão m{M_ENG} z{Z_ENG}; fecha em {math.degrees(THETA_MAX):.1f}°")
    servo = servo_step.carregar()
    exigir_tudo_ok(verificar(servo))
    lista_de_materiais()
    exportar(servo)
    imagens(servo)
