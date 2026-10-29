#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
junta.py
========
Forquilha que sai do servo da junta (ombro ou cotovelo, XM430): duas chapas
planas iguais, uma no horn e outra no idler, unidas por uma ponte que passa
por fora da ponta curta do servo. Tudo no FRAME DO SERVO (dados_servos_dynamixel:
origem no eixo, +z para o horn, +y para a ponta curta), que é também o frame
da junta: a forquilha gira em torno de z, então as malhas já saem centradas
no ponto da junta (convenção de todo o CAD).

Nível buscado: APTO PARA FABRICAÇÃO (impressão em PLA), com as folgas de
impressão marcadas ASSUMIDO até a peça de teste.

Decisões (docs/decisions.md): chapa 28,5 x 4,0 com 8x M2x5 no horn
(resistencia_forquilha.py); chapas planas + ponte; raio da ponte = raio do
corpo pelo STEP (16,99) + folga 0,5; porca embutida (D2) DENTRO DA PONTE,
parafuso atravessando a chapa (porca em pé na chapa de 4 mm não tem parede).

Ordem do script: parâmetros -> peças -> verificações -> (só se tudo OK)
exportação e imagem.

Uso (de dentro de cad/):
    python junta.py
"""

import math
import os

import usar_lib  # noqa: F401
import cadquery as cq
import dados_servos_dynamixel as sv
from check_bbox import check_bbox, check_volume, exigir_tudo_ok
from exportar_stl_mm import exportar_stl_mm

import parafusos_iso as pi
import resistencia_forquilha as rf
import servo_step

AQUI = os.path.dirname(os.path.abspath(__file__))
PROJETO = os.path.dirname(AQUI)

# ---------------------------------------------------------------------------
# Parâmetros
# ---------------------------------------------------------------------------
# Chapa (decisão 2026-10-13, saiu de resistencia_forquilha.py)
T_CHAPA = 4.0
W_CHAPA = sv.X430_LARGURA                       # 28,5: mesma largura do servo
Z_HORN = sv.X430_FURACAO["horn_m2"]["z_face"]   # 19,0: face de fora do horn
RAIO_CANTO = 2.0      # ASSUMIDO: arredonda os cantos de fora (não corta a mão, não descola da mesa)

# Furos (impresso sai menor: série média da ISO 273, ASSUMIDO em parafusos_iso)
D_FURO_M2 = pi.furo_passagem("M2")              # 2,4
D_REBAIXO_M2 = pi.REBAIXO_DIAMETRO["M2"]        # 4,4
D_FURO_CENTRAL = rf.D_FURO_CENTRAL_MM           # 9,0: passa a saliência Ø8 do horn

# Parafusos do horn: M2x5, presa 3,5, rebaixo 0,5 (decisão 2026-10-13).
# Régua ROBOTIS: 1,5 de rosca, 0,5 até o fundo do furo de 2,0.
PROF_ROSCA_HORN = sv.X430_FURACAO["horn_m2"]["prof_max_mm"]   # 2,0
FOLGA_FUNDO_HORN = 0.5
ENGAJ_MIN_HORN = 1.5
FUNDO_MIN = 1.0       # ASSUMIDO só para listar opções; quem decide é o esmagamento
OPCAO_HORN = next(o for o in pi.rebaixo_para_rosca_cega(
    "M2", T_CHAPA, PROF_ROSCA_HORN, FOLGA_FUNDO_HORN, ENGAJ_MIN_HORN, FUNDO_MIN)
    if o["comprimento"] == 5)
PRESA = OPCAO_HORN["presa"]                     # 3,5
REBAIXO = OPCAO_HORN["rebaixo"]                 # 0,5

# Ponte
FOLGA_PONTE = 0.5     # ASSUMIDO (Daniel): impressora ~0,3 + jogo do furo 0,2
PAREDE_MIN = 1.2      # ASSUMIDO: 3 perímetros de bico 0,4
FOLGA_PORCA = 0.3     # ASSUMIDO: no sextavado (entre faces) e na altura do alojamento
PAREDE_SOBRE_PORCA = 2.0   # ASSUMIDO: plástico entre a ponta da ponte e a porca (é ele que a porca puxa)
PASSA_PORCA = 0.5     # ASSUMIDO: quanto a ponta do parafuso passa da porca (pouco mais de 1 fio)
FOLGA_FUNDO_PONTE = 1.0    # ASSUMIDO: furo cego da ponte vai além da ponta do parafuso
X_PARAF_PONTE = 8.0   # ASSUMIDO: ±8 do centro, 2 parafusos por ponta (impede girar)

PORCA = pi.PORCA_ISO4032["M2"]
S_ALOJ = PORCA["s"] + FOLGA_PORCA               # 4,3 entre faces
E_ALOJ = S_ALOJ / math.cos(math.radians(30))    # 4,97 entre cantos (sextavado sem chanfro)
H_ALOJ = PORCA["m_max"] + FOLGA_PORCA           # 1,9


def arredonda_acima(x, passo=0.5):
    return math.ceil(x / passo - 1e-9) * passo


# Raio da ponte: face interna da ponte (decisão "Raio da ponte", 2026-10-13)
R_PONTE = arredonda_acima(sv.x430_raio_max_ponta_curta() + FOLGA_PONTE)   # 17,5
# Espessura da ponte em y: o alojamento da porca (cantos para ±y) + parede dos dois lados
D_PONTE = arredonda_acima(E_ALOJ + 2 * PAREDE_MIN)                        # 7,5
Y_FIM = R_PONTE + D_PONTE                                                 # 25,0
Y_PARAF_PONTE = R_PONTE + D_PONTE / 2.0                                   # 21,25

# Parafuso da ponte: atravessa a chapa (mesma presa e rebaixo do horn), passa a porca
Z_ALOJ_TOPO = Z_HORN - PAREDE_SOBRE_PORCA                                 # 17,0
Z_ALOJ_BASE = Z_ALOJ_TOPO - H_ALOJ                                        # 15,1
L_NECESSARIO = PRESA + (Z_HORN - Z_ALOJ_BASE) + PASSA_PORCA
L_PONTE = min(c for c in pi.PARAFUSO_ISO4762["M2"]["comprimentos"] if c >= L_NECESSARIO - 1e-9)
PROF_FURO_PONTE = (L_PONTE - PRESA) + FOLGA_FUNDO_PONTE

CORES = {"chapa": (0.20, 0.45, 0.80), "ponte": (0.95, 0.55, 0.15),
         "parafuso": (0.15, 0.15, 0.15), "porca": (0.75, 0.75, 0.70),
         "servo": (0.55, 0.55, 0.58)}


# ---------------------------------------------------------------------------
# Peças
# ---------------------------------------------------------------------------
def furos_horn_xy():
    return [(x, y) for x, y, _ in sv.x430_centros_pcd("horn_m2")]


def furos_ponte_xy():
    return [(-X_PARAF_PONTE, Y_PARAF_PONTE), (X_PARAF_PONTE, Y_PARAF_PONTE)]


def chapa(y_fim=Y_FIM, furos_extra_xy=(), com_ponte=True):
    """Chapa do horn em z = 19..23. A do idler é a MESMA peça espelhada
    (o padrão M2 x8 em Ø16 é igual nos dois lados): imprime-se duas.
    O elo (peça única com a forquilha, decisão 2026-10-13) chama com a
    ponta mais longe (`y_fim`) e os furos do flange (`furos_extra_xy`); a mão
    da garra não tem ponte (o servo da garra é o espaçador): `com_ponte=False`."""
    r = W_CHAPA / 2.0
    perfil = (cq.Workplane("XY", origin=(0, 0, Z_HORN))
              .circle(r).extrude(T_CHAPA)
              .union(cq.Workplane("XY", origin=(0, 0, Z_HORN))
                     .center(0, y_fim / 2.0).rect(W_CHAPA, y_fim).extrude(T_CHAPA)))
    perfil = perfil.edges("|Z").edges(f">Y").fillet(RAIO_CANTO)
    topo = cq.Workplane("XY", origin=(0, 0, Z_HORN + T_CHAPA))
    perfil = perfil.cut(topo.circle(D_FURO_CENTRAL / 2.0).extrude(-T_CHAPA))
    for pts in (furos_horn_xy(), furos_ponte_xy() if com_ponte else [], list(furos_extra_xy)):
        if not pts:
            continue
        perfil = perfil.cut(topo.pushPoints(pts).circle(D_FURO_M2 / 2.0).extrude(-T_CHAPA))
        perfil = perfil.cut(topo.pushPoints(pts).circle(D_REBAIXO_M2 / 2.0).extrude(-REBAIXO))
    return perfil


def alojamento_porca(x, y, z_base, abre, ate):
    """Sextavado M2 + rasgo por onde a porca entra deslizando. `abre` = "+y",
    "-y", "+x" ou "-x" (direção do rasgo); `ate` = coordenada da face por onde
    ela entra. As faces planas do sextavado ficam paralelas ao rasgo."""
    eixo = abre[1]
    s = 1.0 if abre[0] == "+" else -1.0
    hexa = cq.Workplane("XY", origin=(x, y, z_base)).polygon(6, E_ALOJ).extrude(H_ALOJ)
    if eixo == "y":   # o polígono nasce com cantos em ±x; gira para os cantos ficarem em ±y
        hexa = hexa.rotate((x, y, 0), (x, y, 1), 90)
    comp = abs(ate + s * 1.0 - (y if eixo == "y" else x))
    if eixo == "y":
        rasgo = cq.Workplane("XY", origin=(x, y + s * comp / 2.0, z_base)).rect(S_ALOJ, comp)
    else:
        rasgo = cq.Workplane("XY", origin=(x + s * comp / 2.0, y, z_base)).rect(comp, S_ALOJ)
    return hexa.union(rasgo.extrude(H_ALOJ))


def ponte():
    """Bloco entre as faces internas das chapas (z = -19..19), face interna
    no raio R_PONTE; 2 porcas M2 embutidas em cada ponta."""
    p = (cq.Workplane("XY", origin=(0, (R_PONTE + Y_FIM) / 2.0, -Z_HORN))
         .rect(W_CHAPA, D_PONTE).extrude(2 * Z_HORN))
    p = p.edges("|Z").edges(">Y").fillet(RAIO_CANTO)
    for s in (1, -1):
        face = cq.Workplane("XY", origin=(0, 0, s * Z_HORN))
        p = p.cut(face.pushPoints(furos_ponte_xy()).circle(D_FURO_M2 / 2.0)
                  .extrude(-s * PROF_FURO_PONTE))
        for x, _ in furos_ponte_xy():
            z_base = Z_ALOJ_BASE if s == 1 else -Z_ALOJ_TOPO
            p = p.cut(alojamento_porca(x, Y_PARAF_PONTE, z_base, "+y", Y_FIM))
    return p


def parafuso_m2(comprimento, x, y, z_apoio, s=1):
    """ISO 4762 simplificado (cabeça e haste lisas): cabeça para +z·s a partir
    de z_apoio, haste para dentro."""
    d = pi.PARAFUSO_ISO4762["M2"]
    cab = cq.Workplane("XY", origin=(x, y, z_apoio)).circle(d["dk_max"] / 2.0).extrude(s * d["k_max"])
    haste = cq.Workplane("XY", origin=(x, y, z_apoio)).circle(d["d"] / 2.0).extrude(-s * comprimento)
    return cab.union(haste)


def porca_m2(x, z_centro):
    """ISO 4032 simplificada (sextavado sem chanfro, furo liso Ø2)."""
    z0 = z_centro - PORCA["m_max"] / 2.0
    return (cq.Workplane("XY", origin=(x, Y_PARAF_PONTE, z0))
            .polygon(6, PORCA["s"] / math.cos(math.radians(30))).extrude(PORCA["m_max"])
            .rotate((x, Y_PARAF_PONTE, 0), (x, Y_PARAF_PONTE, 1), 90)
            .cut(cq.Workplane("XY", origin=(x, Y_PARAF_PONTE, z0)).circle(1.0).extrude(PORCA["m_max"])))


def forquilha():
    """Devolve dict nome -> lista de sólidos, na pose zero."""
    ch = chapa()
    z_apoio = Z_HORN + T_CHAPA - REBAIXO          # 22,5: onde a cabeça apoia
    pecas = {"chapa_horn": [ch], "chapa_idler": [ch.mirror("XY")], "ponte": [ponte()],
             "parafusos_horn": [], "parafusos_ponte": [], "porcas": []}
    for s in (1, -1):
        for x, y in furos_horn_xy():
            pecas["parafusos_horn"].append(parafuso_m2(OPCAO_HORN["comprimento"], x, y, s * z_apoio, s))
        for x, y in furos_ponte_xy():
            pecas["parafusos_ponte"].append(parafuso_m2(L_PONTE, x, y, s * z_apoio, s))
            pecas["porcas"].append(porca_m2(x, s * (Z_ALOJ_BASE + H_ALOJ / 2.0)))
    return pecas


def girar(solido, graus):
    return solido.rotate((0, 0, 0), (0, 0, 1), graus)


def unir(lista):
    """Funde Workplanes ou Shapes num único cq.Shape."""
    formas = [s.val() if hasattr(s, "val") else s for s in lista]
    return formas[0].fuse(*formas[1:]) if len(formas) > 1 else formas[0]


# ---------------------------------------------------------------------------
# Verificações
# ---------------------------------------------------------------------------
def conferir(nome, ok, detalhe=""):
    print(f"  [{'OK' if ok else 'FALHOU'}] {nome}{(': ' + detalhe) if detalhe else ''}")
    return ok


def furos_no_solido(solido, diametro):
    """Centros xy dos furos cilíndricos de um diâmetro, lidos do SÓLIDO
    (pega erro de origem do workplane, que a tabela sozinha não pega)."""
    centros = set()
    for f in solido.val().Faces():
        if f.geomType() == "CYLINDER":
            c = f._geomAdaptor().Cylinder()
            if abs(2 * c.Radius() - diametro) < 1e-3:
                p = c.Axis().Location()
                centros.add((round(p.X(), 2), round(p.Y(), 2)))
    return centros


def verificar(pecas, servo):
    ok = True
    ch, pt = pecas["chapa_horn"][0], pecas["ponte"][0]

    print("\n1. Caixa e volume (esperados calculados à mão, ver o diário 2026-10-13)")
    ok &= check_bbox("chapa", ch, (-14.25, 14.25, -14.25, 25.0, 19.0, 23.0))
    ok &= check_bbox("ponte", pt, (-14.25, 14.25, 17.5, 25.0, -19.0, 19.0))
    # chapa: (28,5x25 + pi·14,25²/2 - 2·2²(1-pi/4) - pi·4,5² - 10·pi·1,2²)·4 - 10·pi·(2,2²-1,2²)·0,5
    ok &= check_volume("chapa", ch, 3630.15, tol=0.1)
    # ponte: (28,5x7,5 - 2·2²(1-pi/4))·38 - 4·(furo 5,5 + alojamento 1,9·(½hexa + rasgo 4,3x3,75) - furo·1,9)
    ok &= check_volume("ponte", pt, 7808.75, tol=0.1)

    print("\n2. Furos da chapa coaxiais com a furação oficial do horn")
    tabela = {(round(x, 2), round(y, 2)) for x, y in furos_horn_xy()}
    lidos = furos_no_solido(ch, D_FURO_M2)
    ok &= conferir("8 furos M2 em Ø16", tabela <= lidos, f"{len(tabela & lidos)}/8 no lugar")
    ok &= conferir("furos da ponte na chapa e na ponte coaxiais",
                   {(round(x, 2), round(y, 2)) for x, y in furos_ponte_xy()} <= (lidos & furos_no_solido(pt, D_FURO_M2)))

    print("\n3. Parafusos")
    eng = OPCAO_HORN["comprimento"] - PRESA
    ok &= conferir("horn M2x5: rosca engajada", ENGAJ_MIN_HORN <= eng <= PROF_ROSCA_HORN - FOLGA_FUNDO_HORN,
                   f"{eng:.1f} mm ({pi.fios_engajados('M2', eng):.1f} fios), fundo a {PROF_ROSCA_HORN - eng:.1f}")
    ponta = Z_HORN + T_CHAPA - REBAIXO - L_PONTE
    ok &= conferir(f"ponte M2x{L_PONTE}: passa a porca", Z_ALOJ_BASE - ponta >= PASSA_PORCA - 1e-9,
                   f"ponta em z={ponta:.1f}, porca até {Z_ALOJ_BASE:.1f}")
    ok &= conferir("ponte: ponta não bate no fundo do furo", ponta > Z_HORN - PROF_FURO_PONTE,
                   f"fundo em z={Z_HORN - PROF_FURO_PONTE:.1f}")

    print("\n4. Resistência (carga = stall 4,1 N·m, FS mínimo 3 ASSUMIDO)")
    r = rf.avaliar(T_CHAPA, W_CHAPA, PRESA, 8)
    ok &= conferir("chapa do horn (resistencia_forquilha.avaliar)", r["passa"],
                   "FS " + ", ".join(f"{k} {v:.2f}" for k, v in r["fs"].items()))
    # Pior caso: o torque inteiro passa pelos 4 parafusos da ponte (se o elo prender na ponte)
    f = rf.forca_por_parafuso(rf.TORQUE_PROJETO_NM * 1000.0, 4, Y_PARAF_PONTE)
    fs = rf.ESCOAMENTO_MPA / rf.pressao_no_furo(f, rf.D_PARAFUSO_MM, PRESA)
    ok &= conferir("esmagamento nos furos da ponte", fs >= rf.FATOR_SEGURANCA_MIN,
                   f"{f:.0f} N por parafuso, FS {fs:.2f}")

    print(f"\n5. Paredes (mínimo {PAREDE_MIN} ASSUMIDO)")
    paredes = {
        "entre rebaixos do horn": 2 * 8.0 * math.sin(math.pi / 8) - D_REBAIXO_M2,
        "rebaixo -> furo central": 8.0 - D_REBAIXO_M2 / 2 - D_FURO_CENTRAL / 2,
        "rebaixo da ponte -> ponta da chapa": Y_FIM - Y_PARAF_PONTE - D_REBAIXO_M2 / 2,
        "alojamento da porca -> face interna da ponte": Y_PARAF_PONTE - E_ALOJ / 2 - R_PONTE,
    }
    for nome, v in paredes.items():
        ok &= conferir(nome, v >= PAREDE_MIN - 1e-9, f"{v:.2f} mm")

    print("\n6. Giro em torno do eixo contra o servo (STEP oficial)")
    fixos = unir([servo.solids().vals()[0], servo.solids().vals()[17]])   # corpo+horn+idler, capa do cabo
    # Parafusos do horn ficam de fora: eles ENTRAM na rosca do horn (Ø2 no furo Ø1,6 do STEP)
    movel = unir(pecas["chapa_horn"] + pecas["chapa_idler"] + pecas["ponte"]
                 + pecas["parafusos_ponte"] + pecas["porcas"])
    ponte_e_porcas = unir(pecas["ponte"] + pecas["porcas"])

    def bate(graus):
        return girar(movel, graus).intersect(fixos).Volume() > 0.01

    angulos = list(range(-90, 91, 10)) + [-57, 57]
    colide = [a for a in angulos if bate(a)]
    ok &= conferir("sem colisão de -90° a +90° (requisito)", not colide, str(colide) if colide else "")
    folgas = {a: girar(ponte_e_porcas, a).distance(servo.solids().vals()[0]) for a in range(-90, 91, 3)}
    folgas.update({a: girar(ponte_e_porcas, a).distance(servo.solids().vals()[0]) for a in (-57, 57)})
    a_min = min(folgas, key=folgas.get)
    ok &= conferir(f"folga radial mínima >= {FOLGA_PONTE}", folgas[a_min] >= FOLGA_PONTE - 0.02,
                   f"{folgas[a_min]:.2f} mm a {a_min}°")
    limite = next((a for a in range(91, 181) if bate(a)), None)
    print(f"  (informativo) primeira colisão girando para +: {limite}°; o limite real vem do elo anterior")
    return ok


def lista_de_materiais():
    print("\nLista de materiais (uma junta)")
    print("  2x chapa (impressa, PLA)            cad_export/junta/chapa.stl")
    print("  1x ponte (impressa, PLA)            cad_export/junta/ponte.stl")
    print(f"  16x parafuso ISO 4762 M2x{OPCAO_HORN['comprimento']} (horn e idler)")
    print(f"  4x parafuso ISO 4762 M2x{L_PONTE} (ponte)")
    print("  4x porca ISO 4032 M2 (embutidas na ponte)")


def exportar(pecas, servo):
    pasta = os.path.join(PROJETO, "cad_export", "junta")
    os.makedirs(pasta, exist_ok=True)
    exportar_stl_mm(pecas["chapa_horn"][0], pasta, "chapa.stl")
    exportar_stl_mm(pecas["ponte"][0], pasta, "ponte.stl")
    cq.exporters.export(pecas["chapa_horn"][0], os.path.join(pasta, "chapa.step"))
    cq.exporters.export(pecas["ponte"][0], os.path.join(pasta, "ponte.step"))
    mont = cq.Assembly(name="junta")
    mont.add(servo, name="servo_XM430_referencia", color=cq.Color(*CORES["servo"]))
    for nome, lista in pecas.items():
        chave = "chapa" if nome.startswith("chapa") else nome.split("_")[0].rstrip("s")
        cor = CORES.get(chave, CORES["parafuso"])
        for i, s in enumerate(lista):
            mont.add(s, name=f"{nome}_{i}", color=cq.Color(*cor))
    mont.export(os.path.join(pasta, "junta_montagem.step"))
    print(f"  STEP: chapa, ponte e junta_montagem em {os.path.relpath(pasta, PROJETO)}/")


def imagens(pecas, servo):
    from render_cad import render
    pasta = os.path.join(PROJETO, "previews")
    os.makedirs(pasta, exist_ok=True)
    itens_servo = [(s, CORES["servo"]) for s in servo.solids().vals()]
    for graus, nome in ((0, "junta.png"), (57, "junta_57graus.png")):
        itens = list(itens_servo)
        for chave, lista in pecas.items():
            cor = CORES["chapa"] if chave.startswith("chapa") else CORES.get(
                {"ponte": "ponte", "porcas": "porca"}.get(chave, "parafuso"))
            itens += [(girar(s.val(), graus), cor) for s in lista]
        render(itens, os.path.join(pasta, nome), foco=(0, 0, 0), posicao=(110, 150, 95), zoom=1.05)


if __name__ == "__main__":
    print(f"R da ponte {R_PONTE} (corpo {sv.x430_raio_max_ponta_curta():.2f} + folga {FOLGA_PONTE}), "
          f"ponte {D_PONTE} mm, chapa até y={Y_FIM}, parafuso da ponte M2x{L_PONTE}")
    servo = servo_step.carregar()
    pecas = forquilha()
    exigir_tudo_ok(verificar(pecas, servo))
    lista_de_materiais()
    print("\nExportação")
    exportar(pecas, servo)
    imagens(pecas, servo)
