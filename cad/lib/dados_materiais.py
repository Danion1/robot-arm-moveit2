#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dados_materiais.py
==================
Propriedades de materiais de peças impressas, com fonte. Projetos importam
daqui.

Fonte: "PLA Technical Data Sheet (Polylactic Acid)", SD3D, cópia hospedada
pelo laboratório de impressão 3D de Harvard (lida em 2026-10-01):
  https://3dp.seas.harvard.edu/files/2023/01/PLAMDS.pdf
"""

PLA = {
    "densidade_g_cm3": 1.24,          # "Solid Density, ASTM D792: 1.24 g/cc"
    "modulo_tracao_gpa": 2.3,         # "Tensile Modulus: 2.3 GPa" (ASTM D638)
    "tensao_escoamento_mpa": 35.9,    # "Tensile Strength at Yield: 35.9 MPa"
    # A ficha não traz coeficiente de Poisson. ASSUMIDO 0,35 (valor típico de
    # termoplástico rígido); só entra na conta de torção, via G = E / (2 (1 + v)).
    "poisson": 0.35,
}
# A ficha vale pra material MACIÇO. Peça impressa com preenchimento parcial é
# mais leve e menos rígida; e a rigidez entre camadas é menor que ao longo delas.

# PLA: dados térmicos da MESMA ficha SD3D (lidos em 2026-10-16).
PLA["hdt_c"] = 49.0                   # "Heat Deflection Temperature, ASTM D648: 49 C"
PLA["tg_c"] = 57.0                    # "Glass Transition by DSC, ASTM E1356: 57 C"

# PETG: "Technical datasheet Prusament PETG", versão 1.1 de 2022-02-16 (Prusa
# Polymers), lida em 2026-10-16. Link em docs/sources.md.
# Valores de corpo de prova IMPRESSO, eixo XY (o Z vem entre parênteses).
PETG = {
    "densidade_g_cm3": 1.27,          # "Density 1.27 g/cm3, ISO 1183"
    "modulo_tracao_gpa": 1.5,         # "Tensile Modulus 1.5 ± 0.1 (1.6 ± 0.1) GPa, ISO 527-1"
    "tensao_escoamento_mpa": 47.0,    # "Tensile Yield Strength 47 ± 2 (50 ± 1) MPa, ISO 527-1"
    "hdt_c": 68.0,                    # "Heat Deflection Temperature (0.45 MPa) 68 °C, ISO 75" (1,8 MPa: também 68)
    "bico_c": (240.0, 260.0),         # "Nozzle Temperature 250 ± 10 °C"
    "mesa_c": (70.0, 90.0),           # "Heatbed Temperature 80 ± 10 °C"
    # Sem Poisson na ficha: mesmo ASSUMIDO do PLA, se precisar.
    "poisson": 0.35,
}
