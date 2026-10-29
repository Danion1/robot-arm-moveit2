# -*- coding: utf-8 -*-
"""
massa_inercia — massa, centro de massa e tensor de inércia COMPLETO de um
link feito de vários corpos.

Quando usar: preencher o <inertial> de cada link.

Entradas: os corpos do link, cada um como peça estrutural (sólido do CAD
+ densidade -> propriedades_solido) ou componente comprado (massa de
datasheet + caixa envelope -> inercia_caixa).
Saída de combinar(): massa (kg), centro de massa (m, no frame do link) e
tensor (kg·m², em torno do centro de massa), com ixy, ixz e iyz.

Validação:
  - conferir_kernel(): uma peça em "L" calculada pelo kernel e à mão;
    confere unidade, ponto de referência e SINAL dos produtos de inércia.
    Rodar uma vez por execução antes de confiar no kernel;
  - checar_tensor(): desigualdade triangular, que todo tensor físico cumpre.

Cuidados:
  - peças impressas que se sobrepõem têm que ser fundidas (union) antes,
    senão o volume conta duas vezes;
  - o kernel devolve inércia de volume em mm⁵; aqui já sai convertida;
  - componente comprado tratado como caixa uniforme é HIPÓTESE: marcar
    como ASSUMIDO onde o resultado for mostrado.
  - não usar a forma do P1 (fórmula por peça e só a diagonal): zerar os
    produtos de inércia num link assimétrico está errado.

Origem: BR/compute_mass_inertia.py, extraído em 2026-10-10 (combinar() é
o laço que estava dentro de calcular(), com a mesma ordem de contas).
"""

import cadquery as cq

from formas_basicas import box_minmax

MM = 0.001   # mm -> m
G = 0.001    # g -> kg


def tensor_zero():
    return [[0.0] * 3 for _ in range(3)]


def tensor_soma(a, b):
    return [[a[i][j] + b[i][j] for j in range(3)] for i in range(3)]


def eixos_paralelos(massa_kg, d):
    """m * (|d|² * identidade - d dᵀ), d em m. É ela que cria os produtos de inércia."""
    d2 = d[0] ** 2 + d[1] ** 2 + d[2] ** 2
    return [[massa_kg * ((d2 if i == j else 0.0) - d[i] * d[j]) for j in range(3)]
            for i in range(3)]


def inercia_caixa(massa_kg, dx, dy, dz):
    """Caixa maciça, em torno do próprio centro (dimensões em m)."""
    t = tensor_zero()
    t[0][0] = massa_kg * (dy ** 2 + dz ** 2) / 12.0
    t[1][1] = massa_kg * (dx ** 2 + dz ** 2) / 12.0
    t[2][2] = massa_kg * (dx ** 2 + dy ** 2) / 12.0
    return t


def propriedades_solido(shape, densidade_g_cm3):
    """Massa (kg), centro (m) e tensor (kg.m², em torno do próprio centro)
    de um sólido do CAD (em mm) com densidade uniforme."""
    val = shape.val() if hasattr(shape, "val") else shape
    massa_kg = val.Volume() * 1e-3 * densidade_g_cm3 * G       # mm³ -> cm³ -> g -> kg
    c = val.Center()
    centro = (c.x * MM, c.y * MM, c.z * MM)
    densidade_kg_mm3 = densidade_g_cm3 * 1e-6
    bruto = cq.Shape.matrixOfInertia(val)                      # mm^5, densidade 1
    tensor = [[bruto[i][j] * densidade_kg_mm3 * MM * MM for j in range(3)] for i in range(3)]
    return massa_kg, centro, tensor


def combinar(corpos):
    """corpos = [(massa_kg, centro_m, tensor_proprio), ...] -> (massa, centro, tensor)."""
    massa = sum(cp[0] for cp in corpos)
    com = tuple(sum(cp[0] * cp[1][k] for cp in corpos) / massa for k in range(3))
    tensor = tensor_zero()
    for m, c, t in corpos:
        d = tuple(c[k] - com[k] for k in range(3))
        tensor = tensor_soma(tensor, tensor_soma(t, eixos_paralelos(m, d)))
    return massa, com, tensor


def checar_tensor(tensor, nome):
    ixx, iyy, izz = tensor[0][0], tensor[1][1], tensor[2][2]
    if not (ixx + iyy >= izz and ixx + izz >= iyy and iyy + izz >= ixx):
        raise SystemExit(f"{nome}: tensor de inércia viola a desigualdade triangular")


def conferir_kernel():
    """Peça em 'L' (duas caixas) pelo kernel e à mão. Para o script se não bater."""
    a = box_minmax(0, 60, 0, 10, 0, 10)      # 6000 mm³
    b = box_minmax(0, 10, 10, 50, 0, 10)     # 4000 mm³
    m_k, c_k, t_k = propriedades_solido(a.union(b), 1.0)

    ma, mb = 6000 * 1e-6, 4000 * 1e-6        # kg, densidade 1 g/cm³
    ca, cb = (0.030, 0.005, 0.005), (0.005, 0.030, 0.005)
    m = ma + mb
    c = tuple((ma * x + mb * y) / m for x, y in zip(ca, cb))
    t = tensor_soma(
        tensor_soma(inercia_caixa(ma, 0.060, 0.010, 0.010),
                    eixos_paralelos(ma, tuple(p - q for p, q in zip(ca, c)))),
        tensor_soma(inercia_caixa(mb, 0.010, 0.040, 0.010),
                    eixos_paralelos(mb, tuple(p - q for p, q in zip(cb, c)))))
    erro = max(abs(t[i][j] - t_k[i][j]) for i in range(3) for j in range(3))
    ok = (abs(m - m_k) < 1e-9 and max(abs(p - q) for p, q in zip(c, c_k)) < 1e-9
          and erro < 1e-12 and abs(t[0][1]) > 1e-8)
    print(f"  [{'OK' if ok else 'FALHOU'}] peça em L: ixy à mão = {t[0][1]:.6e}, "
          f"kernel = {t_k[0][1]:.6e} (maior diferença no tensor: {erro:.1e} kg.m²)")
    if not ok:
        raise SystemExit("O kernel não bateu com a conta à mão — nada foi escrito.")
