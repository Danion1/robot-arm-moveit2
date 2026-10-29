#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
massa.py
========
Massa, centro de massa e tensor de inércia de cada elo do URDF, a partir
dos SÓLIDOS (peças impressas e ferragens) e dos dados da ROBOTIS (servos).
Depois, com esses números: torque estático em cada junta e tombamento da
base solta. Grava resultados/massas.json, de onde o URDF (escrito à mão pelo
Daniel) tira os <inertial>.

Frames dos elos (todos com os eixos do mundo na pose zero: +X frente, +Y
esquerda, +Z cima; o URDF vai usar exatamente estes):
  base_link  origem na mesa, sob o eixo da base
  link1      origem no eixo da base, na face do horn da base (z = 46)
  link2      origem no eixo do ombro (z = 88,25 no mundo)
  link3      origem no eixo do cotovelo (L2 à frente do ombro)
  link4      origem no eixo do punho (L3 à frente do cotovelo)
  mordente_cima / mordente_baixo  = frame do link4 com a garra FECHADA
      (a junta prismática anda em +Y / -Y de 0 a 20 mm)

Hipóteses (ASSUMIDO, aparecem no JSON):
  - PLA maciço (densidade da ficha, 1,24): a peça impressa com preenchimento
    parcial é mais leve, então o torque calculado fica do lado seguro;
  - aço dos parafusos e porcas 7,85 g/cm³ (valor de livro para aço carbono),
    sólido simplificado (sem rosca);
  - o pinhão gira, mas entra como massa do link4 (no URDF ele não é junta);
  - XC430: tensor da tabela do XM430 escalado por 65/82.

Uso (de dentro de cad/):  python massa.py
"""

import json
import math
import os

import usar_lib  # noqa: F401
import cadquery as cq
import dados_materiais as mat
import dados_servos_dynamixel as sv
import massa_inercia as mi

import base as b
import elo as e
import garra as ga
import junta as j
import montagem as mo
import parafusos_iso as pi

ACO_G_CM3 = 7.85         # ASSUMIDO (livro), ver cabeçalho
G = 9.81
CARGA_UTIL_KG = 0.0      # o cálculo dá a carga útil MÁXIMA por junta, não assume uma

# ROBOTIS, "DYNAMIXEL-X The moment of inertia (reference only)", fev. 2023
# (fontes/fabricante/XM430,XH430 Moment of Inertia.pdf, SHA-256 no docs/sources.md).
# Eixos = frame do servo deste projeto (x largura, y para a ponta curta, z saindo
# do horn). DEDUÇÃO: a origem do desenho é a face do horn (z = +19 no nosso frame).
XM430_ROBOTIS = {
    "massa_g": 82.0,
    "cg_mm": (-0.40710295, -15.312099, -16.718279),
    "tensor_g_mm2": ((23746.049, -129.84987, -70.612687),
                     (-129.84987, 13022.591, -1828.4731),
                     (-70.612687, -1828.4731, 21199.469)),
}
Z_ORIGEM_ROBOTIS = 19.0


def servo_corpo(modelo, para_elo):
    """(massa kg, centro m, tensor kg·m²) do servo, levado para o frame do elo.
    `para_elo` = função que leva um sólido do frame do servo para o frame do elo
    (só rotação + translação), usada para levar o centro e girar o tensor."""
    massa_g = sv.SERVOS[modelo]["massa_g"]
    escala = massa_g / XM430_ROBOTIS["massa_g"]
    cg = cq.Vector(XM430_ROBOTIS["cg_mm"][0], XM430_ROBOTIS["cg_mm"][1],
                   XM430_ROBOTIS["cg_mm"][2] + Z_ORIGEM_ROBOTIS)
    # rotação: leva os 3 eixos por para_elo e subtrai a imagem da origem
    o = para_elo(cq.Vertex.makeVertex(0, 0, 0)).Center()
    eixos = [para_elo(cq.Vertex.makeVertex(*v)).Center() - o for v in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]
    rmat = [[eixos[c].toTuple()[r] for c in range(3)] for r in range(3)]       # colunas = eixos do servo no elo
    t = XM430_ROBOTIS["tensor_g_mm2"]
    rt = [[sum(rmat[i][k] * t[k][l] for k in range(3)) for l in range(3)] for i in range(3)]
    tensor = [[sum(rt[i][l] * rmat[jj][l] for l in range(3)) * escala * 1e-9 for jj in range(3)] for i in range(3)]
    c = para_elo(cq.Vertex.makeVertex(cg.x, cg.y, cg.z)).Center()
    return massa_g * 1e-3, (c.x * 1e-3, c.y * 1e-3, c.z * 1e-3), tensor


def parafuso_pontual(rosca, comprimento, ponto_mm, com_porca=False):
    """Massa pontual de um parafuso ISO 4762 simplificado (+ porca), no ponto dado."""
    d = pi.PARAFUSO_ISO4762[rosca]
    vol = math.pi / 4 * (d["d"] ** 2 * comprimento + d["dk_max"] ** 2 * d["k_max"])
    if com_porca:
        p = pi.PORCA_ISO4032[rosca]
        vol += (math.sqrt(3) / 2 * p["s"] ** 2 - math.pi / 4 * d["d"] ** 2) * p["m_max"]
    return vol * 1e-3 * ACO_G_CM3 * 1e-3, tuple(c * 1e-3 for c in ponto_mm), mi.tensor_zero()


def corpos_solidos(solidos, densidade):
    """Funde os sólidos (sobreposição não conta duas vezes) e devolve o corpo."""
    if not solidos:
        return []
    return [mi.propriedades_solido(j.unir(solidos), densidade)]


def elos_no_proprio_frame(servo):
    """{elo: [(massa, centro, tensor)]} no frame de cada elo."""
    sol = servo.solids().vals()
    corpo = j.unir([sol[0], sol[17]])
    pla = mat.PLA["densidade_g_cm3"]
    sobe = lambda dz: (lambda s: s.translate(cq.Vector(0, 0, -dz)))            # mundo -> frame com origem em z = dz
    no_link2 = lambda s: s.rotate(cq.Vector(), cq.Vector(1, 1, 1), -120)       # frame do servo do ombro -> link2
    base = b.montar(servo)
    _, braco = e.elo(e.COMPRIMENTOS["braco"])
    _, ante = e.elo(e.COMPRIMENTOS["antebraco"])
    out = {}

    # base_link: base impressa + servo da base + 8 M2,5 laterais
    out["base_link"] = (corpos_solidos([base["base"].val()], pla)
                        + [servo_corpo("XM430-W350-T", b.no_servo_base)]
                        + [parafuso_pontual("M2.5", b.OPC_LAT["comprimento"], p) for p in b.furos_lateral_mundo()])
    # link1: torre + servo do ombro + parafusos (frame com origem na face do horn da base)
    f1 = sobe(b.Z_HORN_B)
    out["link1"] = (corpos_solidos([f1(base["torre"].val())], pla)
                    + [servo_corpo("XM430-W350-T", lambda s: f1(b.no_ombro(s)))]
                    + [parafuso_pontual("M2", b.OPC_HORN_B["comprimento"], (x, y, b.T_PRATO / 2)) for x, y in b.furos_horn_b_xy()]
                    + [parafuso_pontual("M2.5", b.OPC_OMBRO["comprimento"], (b.X_COLUNA - b.D_COLUNA / 2, y, z - b.Z_HORN_B))
                       for y, z in b.furos_ombro_yz()])

    def elo_corpos(pecas, L, modelo_frente):
        pla_sol = [no_link2(s.val()) for k in ("chapa_horn", "chapa_idler", "ponte", "flange") for s in pecas[k]]
        aco_sol = [no_link2(s.val()) for k in ("parafusos_horn", "parafusos_ponte", "parafusos_flange",
                                                "parafusos_m25", "porcas") for s in pecas[k]]
        frente = lambda s: no_link2(s.translate(cq.Vector(0, L, 0)))
        return (corpos_solidos(pla_sol, pla) + [mi.propriedades_solido(s, ACO_G_CM3) for s in aco_sol]
                + [servo_corpo(modelo_frente, frente)])
    out["link2"] = elo_corpos(braco, e.COMPRIMENTOS["braco"], "XM430-W350-T")     # leva o cotovelo
    out["link3"] = elo_corpos(ante, e.COMPRIMENTOS["antebraco"], "XC430-W240-T")   # leva o punho

    # link4: mão + servo da garra + guias + pinhão + ferragens (no frame do servo do punho -> link2-like)
    mv, _ = ga.mecanismo(ga.THETA_MAX)                                           # fechada = posição 0 das prismáticas
    pla4 = [no_link2(ga.chapa_mao().val()), no_link2(ga.chapa_mao().mirror("XY").val())] + \
           [no_link2(ga.g_para_m(s)) for s in list(ga.fixas_g().values()) + [mv["pinhao"]]]
    m2g = lambda p: ga.g_para_m(cq.Vertex.makeVertex(*p)).Center()             # ponto do frame G -> mão
    para4 = lambda v: no_link2(cq.Vertex.makeVertex(v.x, v.y, v.z)).Center().toTuple()
    pts = []
    for s in (1, -1):
        pts += [("M2", j.OPCAO_HORN["comprimento"], para4(cq.Vector(x, y, s * 21.0))) for x, y in j.furos_horn_xy()]
        pts += [("M2.5", ga.OPC_LAT["comprimento"], para4(cq.Vector(y, ga.Y0 + z, s * 21.0)))
                for x, y, z in sv.X430_FURACAO["lateral_m25"]["centros"] if x > 0]
    pts += [("M2", ga.OPC_PINHAO["comprimento"], para4(m2g((x, y, 4.0)))) for x, y in j.furos_horn_xy()]
    pts += [("M2.5", ga.OPC_GUIA["comprimento"], para4(m2g((x, y, z - 19.0))))
            for nome in ("ponta_curta_m25", "ponta_longa_m25") for x, y, z in sv.X430_FURACAO[nome]["centros"]]
    out["link4"] = (corpos_solidos(pla4, pla)
                    + [servo_corpo("XC430-W240-T", lambda s: no_link2(ga.g_para_m(ga.servo_em_g(s))))]
                    + [parafuso_pontual(r, L, p) for r, L, p in pts])
    # mordentes (fechados): cremalheira + mordente + 2 M2 com porca
    for lado in ("cima", "baixo"):
        sol_m = [no_link2(ga.g_para_m(mv[f"cremalheira_{lado}"])), no_link2(ga.g_para_m(mv[f"mordente_{lado}"]))]
        s = 1 if lado == "cima" else -1
        fer = [parafuso_pontual("M2", j.L_PONTE, para4(m2g((s * x, s * ga.Y_PARAF_MORDENTE, 8.0))), com_porca=True)
               for x in ga.X_PARAF_MORDENTE]
        out[f"mordente_{lado}"] = corpos_solidos(sol_m, pla) + fer
    return out


def torques(res):
    """Torque estático nas juntas de levantar, com o braço esticado na horizontal
    (pior caso da gravidade), e a carga útil máxima pela regra de 1/5 do stall."""
    L2, L3 = e.COMPRIMENTOS["braco"] / 1000, e.COMPRIMENTOS["antebraco"] / 1000
    x0 = {"link2": 0.0, "link3": L2, "link4": L2 + L3, "mordente_cima": L2 + L3, "mordente_baixo": L2 + L3}
    tcp = L2 + L3 + mo.L4 / 1000
    juntas = {"ombro": (0.0, "XM430-W350-T", ["link2", "link3", "link4", "mordente_cima", "mordente_baixo"]),
              "cotovelo": (L2, "XM430-W350-T", ["link3", "link4", "mordente_cima", "mordente_baixo"]),
              "punho": (L2 + L3, "XC430-W240-T", ["link4", "mordente_cima", "mordente_baixo"])}
    out = {}
    for nome, (xj, modelo, elos) in juntas.items():
        tau = sum(res[k]["massa_kg"] * G * (x0[k] + res[k]["com_m"][0] - xj) for k in elos)
        stall = sv.SERVOS[modelo]["torque_stall_nm"]
        lim = stall * sv.FRACAO_STALL_ESTAVEL
        carga = (lim - tau) / (G * (tcp - xj))
        out[nome] = {"torque_nm": tau, "stall_nm": stall, "limite_estavel_nm": lim, "carga_util_max_kg": carga}
    return out


def tombamento(res, alcance_tcp_m):
    """Base solta na mesa, braço esticado para a frente (pose zero) apontando para
    a borda mais próxima do tripé. Devolve (x do centro de massa, x da borda, carga
    que tomba)."""
    massas = []
    z1 = b.Z_HORN_B / 1000
    origem = {"base_link": 0.0, "link1": 0.0, "link2": 0.0, "link3": e.COMPRIMENTOS["braco"] / 1000,
              "link4": (e.COMPRIMENTOS["braco"] + e.COMPRIMENTOS["antebraco"]) / 1000}
    origem["mordente_cima"] = origem["mordente_baixo"] = origem["link4"]
    for k, r in res.items():
        massas.append((r["massa_kg"], origem[k] + r["com_m"][0]))
    m = sum(a for a, _ in massas)
    x = sum(a * c for a, c in massas) / m
    borda = (b.R_PE * math.cos(math.radians(60)) + b.W_PE / 2) / 1000      # 48,5 mm: distância do centro a cada lado do tripé
    # carga na ponta que leva o centro de massa até a borda (negativa = já tomba sem carga)
    carga = (m * (borda - x)) / (alcance_tcp_m - borda)
    return m, x, borda, carga, z1


if __name__ == "__main__":
    from check_bbox import exigir_tudo_ok
    import servo_step
    print("Conferências do kernel e dos dados")
    mi.conferir_kernel()
    servo = servo_step.carregar()
    ok = True
    t = XM430_ROBOTIS["tensor_g_mm2"]
    caixa = mi.inercia_caixa(0.082, 0.0285, 0.0465, 0.034)
    ok &= j.conferir("tensor da ROBOTIS na faixa de uma caixa uniforme do mesmo tamanho (±15%)",
                     all(abs(t[i][i] * 1e-9 / caixa[i][i] - 1) < 0.15 for i in range(3)),
                     " ".join(f"{t[i][i]:.0f}/{caixa[i][i] * 1e9:.0f}" for i in range(3)) + " g·mm²")
    mi.checar_tensor([[x * 1e-9 for x in l] for l in t], "XM430 (ROBOTIS)")

    corpos = elos_no_proprio_frame(servo)
    res = {}
    print("\nElos (massa, centro de massa no frame do elo, diagonal do tensor)")
    for nome, lista in corpos.items():
        m, c, tens = mi.combinar(lista)
        mi.checar_tensor(tens, nome)
        res[nome] = {"massa_kg": m, "com_m": c, "tensor_kg_m2": tens}
        print(f"  {nome:15s} {m * 1000:7.1f} g   com = ({c[0] * 1000:7.1f}, {c[1] * 1000:6.1f}, {c[2] * 1000:6.1f}) mm   "
              f"Ixx/Iyy/Izz = {tens[0][0]:.2e} / {tens[1][1]:.2e} / {tens[2][2]:.2e}")
    total = sum(r["massa_kg"] for r in res.values())
    servos = 3 * sv.SERVOS["XM430-W350-T"]["massa_g"] + 2 * sv.SERVOS["XC430-W240-T"]["massa_g"]
    print(f"  total {total * 1000:.0f} g (servos {servos:.0f} g do datasheet)")
    ok &= j.conferir("massa dos servos dentro do total e total plausível (servos < total < 3x servos)",
                     servos / 1000 < total < 3 * servos / 1000)

    print("\nTorque estático, braço esticado na horizontal (pior caso), sem carga")
    tq = torques(res)
    for nome, r in tq.items():
        ok &= j.conferir(f"{nome}: {r['torque_nm']:.3f} N·m < 1/5 do stall ({r['limite_estavel_nm']:.2f})",
                         r["torque_nm"] < r["limite_estavel_nm"],
                         f"{100 * r['torque_nm'] / r['stall_nm']:.0f}% do stall; carga útil máx. {r['carga_util_max_kg'] * 1000:.0f} g")
    carga = min(r["carga_util_max_kg"] for r in tq.values())
    junta_limite = min(tq, key=lambda k: tq[k]["carga_util_max_kg"])
    print(f"  -> carga útil máxima no braço esticado: {carga * 1000:.0f} g (quem limita: {junta_limite})")

    print("\nTombamento da base SOLTA (braço esticado para a borda mais próxima do tripé)")
    alcance = (e.COMPRIMENTOS["braco"] + e.COMPRIMENTOS["antebraco"] + mo.L4) / 1000
    m, x, borda, ct, _ = tombamento(res, alcance)
    estado = "TOMBA sem carga" if x > borda else f"tomba com {ct * 1000:.0f} g na ponta"
    print(f"  massa {m * 1000:.0f} g, centro de massa a {x * 1000:.1f} mm do eixo; borda a {borda * 1000:.1f} mm -> {estado}")
    print("  (a base é fixada na mesa: condição de uso, Decisões 2026-10-13)")

    exigir_tudo_ok(ok)
    pasta = os.path.join(j.PROJETO, "resultados")
    os.makedirs(pasta, exist_ok=True)
    saida = {"unidades": "kg, m, kg·m²; tensor em torno do centro de massa, eixos do frame do elo",
             "hipoteses": ["PLA maciço 1,24 g/cm³ (ficha SD3D)", "aço 7,85 g/cm³ (ASSUMIDO)",
                           "servos: tabela da ROBOTIS (fev. 2023), origem na face do horn (DEDUÇÃO)",
                           "XC430: tensor do XM430 x 65/82 (ASSUMIDO)", "pinhão como massa do link4"],
             "elos": res, "torques": tq,
             "tombamento_base_solta": {"massa_kg": m, "com_x_m": x, "borda_m": borda,
                                       "carga_que_tomba_kg": ct}}
    with open(os.path.join(pasta, "massas.json"), "w") as f:
        json.dump(saida, f, indent=2)
    print("\nGravado: resultados/massas.json")
