# -*- coding: utf-8 -*-
"""
formas_basicas — caixa, cilindro e esfera posicionados por cota (mm).

Quando usar: qualquer corpo que seja caixa, cilindro ou esfera (placas,
servo como caixa envelope, rodas, colunas).

Entradas: cotas em milímetros. Saída: um cq.Workplane com o sólido já na
posição.

Duas formas de dar a caixa, de propósito:
  - box_minmax (por limites): os números são os mesmos da tabela do
    check_bbox, então é a mais direta de conferir;
  - box_at (por centro e tamanho): mais natural para componente comprado.

Validação: nenhuma própria; quem valida é o check_bbox.

Cuidado: o cilindro do CadQuery nasce com eixo em Z. Para eixo em Y,
girar 90° em torno de X ANTES de transladar (cylinder_y_at já faz isso).

Origem: P1/build_chassi.py (box_at, cylinder_*, sphere_at) e
BR/build_braco.py (box_minmax), extraídos em 2026-10-10.
"""

import cadquery as cq


def box_minmax(x0, x1, y0, y1, z0, z1):
    """Caixa alinhada aos eixos, dada pelos limites."""
    return (cq.Workplane("XY")
            .box(x1 - x0, y1 - y0, z1 - z0)
            .translate(((x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0)))


def box_at(cx, cy, cz, dx, dy, dz):
    """Caixa dx*dy*dz centrada em (cx, cy, cz)."""
    return cq.Workplane("XY").box(dx, dy, dz).translate((cx, cy, cz))


def cylinder_z_at(cx, cy, z_bottom, diameter, height):
    """Cilindro de eixo Z, base em z_bottom, no ponto (cx, cy)."""
    return (cq.Workplane("XY")
            .cylinder(height=height, radius=diameter / 2.0)
            .translate((cx, cy, z_bottom + height / 2.0)))


def cylinder_y_at(cx, cy_center, cz, diameter, width):
    """Cilindro de eixo Y (como uma roda), centrado em (cx, cy_center, cz)."""
    cyl = cq.Workplane("XY").cylinder(height=width, radius=diameter / 2.0)
    cyl = cyl.rotate((0, 0, 0), (1, 0, 0), 90)
    return cyl.translate((cx, cy_center, cz))


def sphere_at(cx, cy, cz, diameter):
    """Esfera de diâmetro `diameter`, centrada em (cx, cy, cz)."""
    return cq.Workplane("XY").sphere(diameter / 2.0).translate((cx, cy, cz))
