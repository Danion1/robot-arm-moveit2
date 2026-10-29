#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
engrenagem.py
=============
Pinhão de dentes retos (perfil evolvente) e cremalheira, para a garra de
mordentes paralelos (decisão 2026-10-13). Parte de cálculo + perfil 2D; a
peça (com furos do horn etc.) fica no garra.py.

Por que evolvente: é a curva que mantém a força de contato sempre na mesma
direção (ângulo de pressão) enquanto o dente gira, então a cremalheira anda
em velocidade constante para velocidade constante do servo. Ela é gerada
"desenrolando um barbante" de um círculo de base de raio rb:
    x(t) = rb·(cos t + t·sen t),  y(t) = rb·(sen t - t·cos t)
O flanco reto da cremalheira é o caso limite de uma evolvente de raio infinito.

Grandezas (todas em mm, ângulo de pressão 20°, dente normal):
    módulo m (escolha), z dentes
    diâmetro primitivo  d  = m·z        (rolamento sem escorregar com a cremalheira)
    diâmetro de base    db = d·cos(20°)
    adendo m, dedendo 1,25·m -> d_ext = d + 2m, d_raiz = d - 2,5m
    passo p = pi·m (igual no pinhão e na cremalheira)
    cremalheira anda d/2 por radiano do pinhão.
Sem rebaixamento do pé do dente (undercut) com cremalheira: z >= 2/sen²(20°) = 17,1.

Resistência do dente (Lewis): sigma = F / (b·m·Y), F tangencial no primitivo,
b largura da face, Y fator de forma. Tabela de Y para 20°, dente normal:
Shigley, "Mechanical Engineering Design", Tabela 14-2 (cópia não guardada;
segunda fonte PENDENTE). Lewis ignora concentração no pé e dinâmica: por
isso o FS mínimo continua 3 sobre o escoamento do PLA maciço (ASSUMIDO).

Uso: importar; `python engrenagem.py` imprime a tabela de dimensionamento.
"""

import math

ANG_PRESSAO = math.radians(20.0)
Z_MIN_SEM_REBAIXO = math.ceil(2.0 / math.sin(ANG_PRESSAO) ** 2)   # 18 (17,1 arredondado para cima)

# Shigley Tabela 14-2: fator de forma de Lewis Y, 20°, dente normal
LEWIS_Y = {12: 0.245, 13: 0.261, 14: 0.277, 15: 0.290, 16: 0.296, 17: 0.303, 18: 0.309,
           19: 0.314, 20: 0.322, 21: 0.328, 22: 0.331, 24: 0.337, 26: 0.346, 28: 0.353,
           30: 0.359, 34: 0.371, 38: 0.384, 43: 0.397, 50: 0.409, 60: 0.422}
LEWIS_Y_CREMALHEIRA = 0.485   # mesma tabela, linha "rack"


def lewis_y(z):
    """Y da tabela; entre linhas, o da linha de BAIXO (menor Y = conservador)."""
    if z < min(LEWIS_Y):
        raise ValueError(f"z = {z}: fora da tabela de Lewis (mínimo {min(LEWIS_Y)})")
    return LEWIS_Y[max(k for k in LEWIS_Y if k <= z)]


def dimensoes(m, z):
    d = m * z
    return {"m": m, "z": z, "d": d, "db": d * math.cos(ANG_PRESSAO), "d_ext": d + 2 * m,
            "d_raiz": d - 2.5 * m, "passo": math.pi * m, "mm_por_rad": d / 2.0}


def tensao_dente(torque_nmm, m, z, b, n_cremalheiras=2):
    """MPa no pé do dente do PINHÃO (Y do pinhão < Y da cremalheira, ele é o
    crítico). O torque se divide entre as cremalheiras que ele empurra."""
    f = torque_nmm / (m * z / 2.0) / n_cremalheiras
    return f, f / (b * m * lewis_y(z))


def _evolvente(rb, t):
    return (rb * (math.cos(t) + t * math.sin(t)), rb * (math.sin(t) - t * math.cos(t)))


def perfil_pinhao(m, z, folga_flanco=0.0, n=12):
    """Pontos (x, y) do contorno fechado do pinhão, centro na origem, dente 0
    centrado no eixo +x. `folga_flanco` (mm) afina cada flanco no primitivo
    (folga de impressão, ASSUMIDA por quem chama)."""
    g = dimensoes(m, z)
    rb, ra, rf, r = g["db"] / 2, g["d_ext"] / 2, g["d_raiz"] / 2, g["d"] / 2
    inv = lambda a: math.tan(a) - a                       # função evolvente
    # meia espessura angular do dente no primitivo, menos a folga
    meia = (math.pi / (2 * z)) - folga_flanco / r
    # ângulo polar do ponto da evolvente no primitivo
    a_prim = math.acos(rb / r)
    deslocamento = meia + inv(a_prim)                     # gira a evolvente para o flanco cair certo
    t_max = math.sqrt((ra / rb) ** 2 - 1)
    t_min = math.sqrt(max((rf / rb) ** 2 - 1, 0.0))
    flanco = []
    for i in range(n + 1):
        t = t_min + (t_max - t_min) * i / n
        x, y = _evolvente(rb, t)
        ang = math.atan2(y, x)
        rr = math.hypot(x, y)
        flanco.append((rr, deslocamento - ang))           # (raio, ângulo) do flanco de CIMA
    pts = []
    passo = 2 * math.pi / z
    for k in range(z):
        c = k * passo
        if rf < rb:                                       # trecho radial da raiz até a base
            pts.append((rf * math.cos(c - deslocamento), rf * math.sin(c - deslocamento)))
        for rr, a in flanco:                              # flanco de baixo (subindo)
            pts.append((rr * math.cos(c - a), rr * math.sin(c - a)))
        for rr, a in reversed(flanco):                    # flanco de cima (descendo)
            pts.append((rr * math.cos(c + a), rr * math.sin(c + a)))
        if rf < rb:
            pts.append((rf * math.cos(c + deslocamento), rf * math.sin(c + deslocamento)))
    return pts


def perfil_cremalheira(m, n_dentes, altura_corpo, folga_flanco=0.0):
    """Contorno fechado da cremalheira: linha primitiva em y = 0, dentes para
    +y (adendo m), corpo para -y (dedendo 1,25m + altura_corpo). Dente 0 com
    o eixo em x = 0."""
    p = math.pi * m
    tg = math.tan(ANG_PRESSAO)
    meia = p / 4 - folga_flanco                           # meia espessura do dente no primitivo
    ha, hf = m, 1.25 * m
    pts = [(-p / 2, -hf - altura_corpo)]
    for k in range(n_dentes):
        c = k * p
        pts += [(c - p / 2, -hf), (c - meia - hf * tg, -hf), (c - meia + ha * tg, ha),
                (c + meia - ha * tg, ha), (c + meia + hf * tg, -hf)]
    pts += [((n_dentes - 1) * p + p / 2, -hf), ((n_dentes - 1) * p + p / 2, -hf - altura_corpo)]
    return pts


if __name__ == "__main__":
    import usar_lib  # noqa: F401
    import dados_servos_dynamixel as sv
    import resistencia_forquilha as rf
    T = sv.SERVOS["XC430-W240-T"]["torque_stall_nm"] * 1000.0   # 1,9 N·m: carga = stall (mesma regra)
    print(f"Torque de projeto {T / 1000} N·m (stall XC430), PLA {rf.ESCOAMENTO_MPA} MPa, "
          f"FS mínimo {rf.FATOR_SEGURANCA_MIN}; z mínimo sem rebaixo {Z_MIN_SEM_REBAIXO}\n")
    print(f"{'m':>4} {'z':>3} {'d':>5} {'b':>4} {'F/mordente':>10} {'sigma':>6} {'FS':>5}")
    for m in (1.0, 1.25, 1.5, 2.0):
        for z in (18, 20, 24):
            for b in (6.0, 8.0, 10.0):
                f, s = tensao_dente(T, m, z, b)
                fs = rf.ESCOAMENTO_MPA / s
                print(f"{m:4.2f} {z:3d} {m * z:5.1f} {b:4.0f} {f:10.1f} {s:6.1f} {fs:5.2f}"
                      f"{'  ok' if fs >= rf.FATOR_SEGURANCA_MIN else ''}")
