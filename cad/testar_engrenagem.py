#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Testes do engrenagem.py. O principal é o de ENGRENAMENTO: com a cremalheira
na linha primitiva tangente ao primitivo do pinhão, girar o pinhão de θ e
andar a cremalheira r·θ não pode fazer os dentes se cruzarem (é isso que a
evolvente garante). Teste negativo: aproximar 1 mm tem que cruzar (0,5 fica no limite da folga).

Uso (de dentro de cad/):  python testar_engrenagem.py
"""

import math

import cadquery as cq

import engrenagem as g

M, Z, B = 2.0, 18, 8.0
FOLGA = 0.1      # mm por flanco, igual à que a garra vai usar
ok_geral = True


def conferir(nome, ok, detalhe=""):
    global ok_geral
    print(f"  [{'OK' if ok else 'FALHOU'}] {nome}{(': ' + detalhe) if detalhe else ''}")
    ok_geral &= ok


def pinhao_solido(folga=FOLGA):
    return cq.Workplane("XY").polyline(g.perfil_pinhao(M, Z, folga)).close().extrude(B).val()


def cremalheira_solida(n, folga=FOLGA):
    return cq.Workplane("XY").polyline(g.perfil_cremalheira(M, n, 4.0, folga)).close().extrude(B).val()


print("1. Contas à mão (m = 2, z = 18)")
d = g.dimensoes(M, Z)
conferir("primitivo 36, externo 40, raiz 31, passo 6,283",
         (d["d"], d["d_ext"], d["d_raiz"]) == (36.0, 40.0, 31.0) and abs(d["passo"] - 6.2832) < 1e-3)
conferir("base = 36·cos20 = 33,829", abs(d["db"] - 33.8289) < 1e-3)
conferir("z mínimo sem rebaixo = 18 (2/sen²20 = 17,1)", g.Z_MIN_SEM_REBAIXO == 18)
conferir("Lewis Y(18) = 0,309; Y(23) usa a linha de 22 (0,331)", g.lewis_y(18) == 0.309 and g.lewis_y(23) == 0.331)
f, s = g.tensao_dente(1900.0, M, Z, B)
conferir("F = 1900/18/2 = 52,8 N; sigma = 52,8/(8·2·0,309) = 10,7 MPa",
         abs(f - 52.78) < 0.01 and abs(s - 10.67) < 0.01)

print("\n2. Perfil do pinhão lido do sólido")
p = pinhao_solido(0.0)
bb = p.BoundingBox()
conferir("diâmetro externo 40", abs(max(bb.xmax - bb.xmin, bb.ymax - bb.ymin) - 40.0) < 0.05,
         f"{bb.xmax - bb.xmin:.3f} x {bb.ymax - bb.ymin:.3f}")
# Espessura do dente 0 no primitivo: corte com um anel fino no raio 18
anel = cq.Workplane("XY").circle(18.02).circle(17.98).extrude(B).val()
pedacos = p.intersect(anel).Solids()
conferir("18 dentes cruzam o primitivo", len(pedacos) == 18, f"{len(pedacos)}")
esp = max(s.BoundingBox().ymax - s.BoundingBox().ymin for s in pedacos
          if abs(s.Center().y) < 1 and s.Center().x > 0)
conferir("espessura no primitivo = pi·m/2 = 3,142 (corda)", abs(esp - 2 * 18 * math.sin(math.pi / 36)) < 0.02,
         f"{esp:.3f}")

print("\n3. Engrenamento com a cremalheira (folga 0,1 por flanco)")
r = d["d"] / 2
n = 9


def montar(theta, aproxima=0.0):
    # cremalheira com a linha primitiva em y = r (tangente ao primitivo), dentes para -y
    c = cremalheira_solida(n).mirror("XZ").translate(cq.Vector(-4 * d["passo"] - r * theta, r - aproxima, 0))   # pinhão anti-horário: o topo anda para -x
    return pinhao_solido().rotate(cq.Vector(), cq.Vector(0, 0, 1), math.degrees(theta) + 90.0 + 180.0 / Z), c


angs = [math.radians(a) for a in range(-30, 31, 5)]
vols = [montar(t)[0].intersect(montar(t)[1]).Volume() for t in angs]
conferir("nenhum cruzamento de -30° a +30°", max(vols) < 1e-3, f"máx {max(vols):.4f} mm³")
folgas = [montar(t)[0].distance(montar(t)[1]) for t in angs]
conferir("folga entre dentes pequena (engrena mesmo)", max(folgas) < 0.5, f"{min(folgas):.3f} a {max(folgas):.3f} mm")
v = montar(0.0, aproxima=1.0)
conferir("NEGATIVO: aproximar 1 mm faz os dentes se cruzarem", v[0].intersect(v[1]).Volume() > 0.01)

print()
if not ok_geral:
    raise SystemExit("FALHOU")
print("Todos os testes passaram.")
