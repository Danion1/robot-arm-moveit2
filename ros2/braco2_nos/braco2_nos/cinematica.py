#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Cinemática direta e inversa do braço do Projeto 2.

Cinemática direta e inversa do braço do Projeto 2, em Python puro (só o
módulo `math`): o nó de pick and place importa daqui, sem CadQuery.

Adaptado de um braço de demonstração anterior (decisão "Cinemática: A",
2026-10-13). O que mudou em relação a ele: as
medidas, os nomes das juntas e os LIMITES (a demo não conferia limites;
aqui `solucoes()` devolve só o que o URDF aceita).

Juntas (mesma convenção do `cad/montagem.py` e do URDF, que usa eixo -Y):
    q1 = base, em torno de Z (positivo = gira para a esquerda, +Y)
    q2 = ombro, q3 = cotovelo, q4 = punho (positivo = levanta o elo)
Pose zero: braço esticado na horizontal, apontando para +X.

Com 4 juntas se escolhem 4 números na ponta: posição (x, y, z) e a
inclinação da garra `phi` (acima da horizontal; -90° = apontando para a mesa).
A direção horizontal da garra não é livre: aponta sempre para fora da base.

Duas cinemáticas diretas INDEPENDENTES, de propósito:
  - fk_analitica(): seno e cosseno somando os elos (a conta feita à mão);
  - fk_matrizes(): matrizes 4x4 junta por junta, como o URDF descreve.

Unidades: metros e radianos (convenção do ROS, REP 103). O CAD usa mm e graus.
"""

import math

# Geometria (m), copiada do URDF gerado (`braco2_description/urdf/braco.urdf`);
# o teste `test_geometria_igual_ao_urdf` confere que continua igual.
Z_JUNTA1 = 0.046             # face do horn da base acima da mesa (origem de junta_base)
Z_OMBRO_NO_LINK1 = 0.04225   # eixo do ombro acima da junta da base (origem de junta_ombro)
Z_OMBRO = Z_JUNTA1 + Z_OMBRO_NO_LINK1   # 0,08825
L2 = 0.105                   # ombro -> cotovelo
L3 = 0.090                   # cotovelo -> punho
L4 = 0.0835                  # punho -> TCP (meio dos dedos)

# Limites (rad), os MESMOS números do URDF, já arredondados para dentro
# (valor cheio fica fora do limite e é rejeitado).
LIMITES = {
    'junta_base': (-3.14159, 3.14159),
    'junta_ombro': (-0.94247, 1.57079),       # -54° (bate na torre) a +90°
    'junta_cotovelo': (-1.57079, 1.57079),    # faixa VERIFICADA no CAD, não o batente
    'junta_punho': (-1.57079, 1.57079),       # idem
}
NOMES = tuple(LIMITES)


# =============================================================================
# 1. CINEMÁTICA DIRETA — fórmula fechada
# =============================================================================

def fk_analitica(q1, q2, q3, q4):
    """
    Ângulos das juntas -> (x, y, z, phi) do TCP.

    No plano do braço cada elo aponta na soma dos ângulos até ele; `r` é o
    quanto o TCP se afasta do eixo da base, e a base só gira esse plano.
    """
    a2 = q2
    a3 = q2 + q3
    phi = q2 + q3 + q4
    r = L2 * math.cos(a2) + L3 * math.cos(a3) + L4 * math.cos(phi)
    z = Z_OMBRO + L2 * math.sin(a2) + L3 * math.sin(a3) + L4 * math.sin(phi)
    return (r * math.cos(q1), r * math.sin(q1), z, phi)


# =============================================================================
# 2. CINEMÁTICA DIRETA — matrizes homogêneas 4x4 (como o URDF)
# =============================================================================

def mat_identidade():
    return [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]


def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def mat_translacao(x, y, z):
    m = mat_identidade()
    m[0][3], m[1][3], m[2][3] = x, y, z
    return m


def mat_rotacao(eixo, angulo):
    """Rotação de `angulo` em torno de um eixo unitário (fórmula de Rodrigues)."""
    ux, uy, uz = eixo
    c, s = math.cos(angulo), math.sin(angulo)
    v = 1.0 - c
    return [
        [c + ux * ux * v,      ux * uy * v - uz * s, ux * uz * v + uy * s, 0.0],
        [uy * ux * v + uz * s, c + uy * uy * v,      uy * uz * v - ux * s, 0.0],
        [uz * ux * v - uy * s, uz * uy * v + ux * s, c + uz * uz * v,      0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


# A cadeia, igual às juntas do URDF: (link filho, origem no pai, eixo).
CADEIA = [
    ('link1', (0.0, 0.0, Z_JUNTA1), (0.0, 0.0, 1.0)),
    ('link2', (0.0, 0.0, Z_OMBRO_NO_LINK1), (0.0, -1.0, 0.0)),
    ('link3', (L2, 0.0, 0.0), (0.0, -1.0, 0.0)),
    ('link4', (L3, 0.0, 0.0), (0.0, -1.0, 0.0)),
]
TCP_NO_LINK4 = (L4, 0.0, 0.0)


def fk_matrizes(q1, q2, q3, q4):
    """
    Ângulos -> {nome do link: matriz 4x4 no mundo}, mais o "tcp".

    Cada junta contribui com a translação até a origem dela (fixa) e depois
    a rotação em torno do eixo dela (o ângulo da junta).
    """
    resultado = {'base_link': mat_identidade()}
    t = mat_identidade()
    for (nome, origem, eixo), q in zip(CADEIA, (q1, q2, q3, q4)):
        t = mat_mul(mat_mul(t, mat_translacao(*origem)), mat_rotacao(eixo, q))
        resultado[nome] = t
    resultado['tcp'] = mat_mul(t, mat_translacao(*TCP_NO_LINK4))
    return resultado


def pose_do_tcp(matrizes):
    """
    (x, y, z, phi) a partir da matriz do TCP, para comparar com fk_analitica.

    phi sai do eixo X da mão (primeira coluna da rotação): componente para
    cima contra componente para fora. O sinal de "para fora" vem da projeção
    na direção radial da base (a mão pode passar da vertical).
    """
    m = matrizes['tcp']
    x, y, z = m[0][3], m[1][3], m[2][3]
    horizontal = math.hypot(m[0][0], m[1][0])
    link1 = matrizes['link1']
    radial = m[0][0] * link1[0][0] + m[1][0] * link1[1][0]
    phi = math.atan2(m[2][0], math.copysign(horizontal, radial))
    return (x, y, z, phi)


# =============================================================================
# 3. CINEMÁTICA INVERSA — analítica
# =============================================================================

class ForaDoAlcance(ValueError):
    """O ponto pedido não pode ser atingido com a inclinação pedida."""


def ik(x, y, z, phi, cotovelo='cima'):
    """
    (x, y, z, phi) do TCP -> (q1, q2, q3, q4), SEM conferir limites.

    Os passos são os da conta à mão (2026-10-14):
      1. q1: a base aponta para o alvo, atan2(y, x).
      2. Recuar do TCP até o punho, L4 para trás ao longo da mão.
      3. Triângulo ombro-cotovelo-punho (lados L2, L3, d): a lei dos cossenos
         dá o cotovelo. Duas soluções: cotovelo para cima (q3 < 0) ou para baixo.
      4. q2 = alfa (direção do punho) - beta (desvio causado pelo cotovelo).
         Aqui beta sai de atan2(L3·sin q3, L2 + L3·cos q3), que é o mesmo
         ângulo da lei dos cossenos usada à mão, mas sem dividir por d
         (não quebra com o punho em cima do ombro).
      5. q4 fecha a soma: phi = q2 + q3 + q4.

    Não trata a solução "de costas" (base girada 180° e ombro além de 90°):
    o limite do ombro (+90°) já a impede neste braço.
    """
    r = math.hypot(x, y)
    q1 = math.atan2(y, x) if r > 1e-12 else 0.0   # sobre o eixo: qualquer q1 serve

    r_punho = r - L4 * math.cos(phi)
    z_punho = (z - Z_OMBRO) - L4 * math.sin(phi)
    d2 = r_punho * r_punho + z_punho * z_punho

    cos_q3 = (d2 - L2 * L2 - L3 * L3) / (2.0 * L2 * L3)
    if cos_q3 > 1.0 + 1e-9 or cos_q3 < -1.0 - 1e-9:
        raise ForaDoAlcance(
            f'punho a {math.sqrt(d2) * 1000:.1f} mm do ombro; o triângulo só fecha de '
            f'{abs(L2 - L3) * 1000:.1f} a {(L2 + L3) * 1000:.1f} mm')
    cos_q3 = max(-1.0, min(1.0, cos_q3))

    q3 = math.acos(cos_q3)
    if cotovelo == 'cima':
        q3 = -q3          # positivo levanta; cotovelo para cima = antebraço descendo
    elif cotovelo != 'baixo':
        raise ValueError("cotovelo deve ser 'cima' ou 'baixo'")

    q2 = math.atan2(z_punho, r_punho) - math.atan2(L3 * math.sin(q3), L2 + L3 * math.cos(q3))
    q4 = phi - q2 - q3
    return (q1, normaliza(q2), q3, normaliza(q4))


def dentro_dos_limites(q):
    return all(lo <= a <= hi for a, (lo, hi) in zip(q, LIMITES.values()))


def solucoes(x, y, z, phi):
    """
    Todas as soluções que o URDF aceita, cotovelo para cima primeiro.

    Lista vazia = inalcançável com essa inclinação (fora do triângulo OU
    fora dos limites). Quem chama escolhe entre as que sobraram.
    """
    validas = []
    for ramo in ('cima', 'baixo'):
        try:
            q = ik(x, y, z, phi, cotovelo=ramo)
        except ForaDoAlcance:
            return []
        if dentro_dos_limites(q) and q not in validas:
            validas.append(q)
    return validas


def normaliza(angulo):
    """Traz o ângulo para a faixa (-pi, pi]."""
    while angulo > math.pi:
        angulo -= 2.0 * math.pi
    while angulo <= -math.pi:
        angulo += 2.0 * math.pi
    return angulo


if __name__ == '__main__':
    # O exemplo feito à mão em 2026-10-14: r = 150, z = 50, garra para baixo.
    graus = math.degrees
    alvo = (0.150, 0.0, 0.050, math.radians(-90))
    print('Alvo (x, y, z, phi):', alvo)
    for ramo in ('cima', 'baixo'):
        q = ik(*alvo, cotovelo=ramo)
        ok = 'dentro' if dentro_dos_limites(q) else 'FORA'
        print(f'  cotovelo {ramo:5}: {tuple(round(graus(v), 2) for v in q)}  ({ok} dos limites)')
    print('  executáveis:', [tuple(round(graus(v), 2) for v in q) for q in solucoes(*alvo)])
