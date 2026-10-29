# -*- coding: utf-8 -*-
"""
check_bbox — confere caixa envolvente e volume contra valores esperados.

Quando usar: SEMPRE, antes de exportar qualquer STL.

Entradas: nome, sólido (cq.Workplane ou cq.Shape), esperado e tolerância.
  - check_bbox: esperado = (xmin, xmax, ymin, ymax, zmin, zmax) em mm;
  - check_volume: esperado em mm³. Pega sobreposição entre peças, que a
    caixa envolvente não vê (valeu a pena no braço da demo).
Saída: True/False e uma linha [OK]/[FALHOU] com esperado e obtido. Quem
chama acumula os resultados e termina com SystemExit ANTES de exportar se
algum falhou (exigir_tudo_ok faz isso).

O que importa e não é código: os esperados têm que ser números LITERAIS,
calculados à mão a partir do datasheet ou tirados de um CAD anterior
validado — nunca as mesmas fórmulas que desenham a peça, senão a
verificação confere a fórmula contra ela mesma.

Origem: P1/build_chassi.py e BR/build_braco.py (idênticas), extraído em
2026-10-10.
"""


def bbox_of(shape):
    val = shape.val() if hasattr(shape, "val") else shape
    bb = val.BoundingBox()
    return (round(bb.xmin, 3), round(bb.xmax, 3),
            round(bb.ymin, 3), round(bb.ymax, 3),
            round(bb.zmin, 3), round(bb.zmax, 3))


def check_bbox(name, shape, expected, tol=0.05):
    got = bbox_of(shape)
    ok = all(abs(g - e) <= tol for g, e in zip(got, expected))
    print(f"  [{'OK' if ok else 'FALHOU'}] {name}: esperado={expected} obtido={got}")
    return ok


def check_volume(name, shape, expected, tol=0.5):
    val = shape.val() if hasattr(shape, "val") else shape
    vol = val.Volume()
    ok = abs(vol - expected) < tol
    print(f"  [{'OK' if ok else 'FALHOU'}] {name}: esperado={expected:.2f} obtido={vol:.2f}")
    return ok


def exigir_tudo_ok(all_ok):
    """Para o script antes de qualquer exportação se algo falhou."""
    if not all_ok:
        raise SystemExit("\nVerificação falhou — confira os parâmetros antes de exportar.")
