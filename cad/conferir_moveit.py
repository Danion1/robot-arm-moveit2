#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
conferir_moveit.py
==================
Segunda metade da conferência MoveIt2 x CAD (moveit2.md, lição 6). Lê o que o
MoveIt2 respondeu (resultados/moveit_teste.json, gravado por
ros2/gravar_moveit.py) e confere por um caminho INDEPENDENTE: os sólidos do
montagem.py (interseção no OpenCASCADE), não as caixas nem a FCL do MoveIt2.

  0. as convenções batem: a cadeia de sólidos (graus, mm) e a cinemática do
     pacote de nós (rad, m), que é a do URDF, levam o TCP ao mesmo ponto;
  1. colisão: o veredito do MoveIt2 (FCL sobre as caixas de colisão, com
     margem de 1 mm) bate com o do CAD? Discordância PERIGOSA = MoveIt2 diz
     "livre" e o CAD diz "colide" (reprova). A outra (MoveIt2 diz "colide",
     CAD "livre") é o lado seguro: mostra-se a que distância os sólidos estão;
  2. trajetórias planejadas e executadas: cada ponto e os intermediários
     (passo de no máximo 1°) livres no CAD, dentro dos limites, dentro da
     velocidade pedida; o robô terminou no alvo. No caso difícil, a reta nas
     juntas tem de colidir também no CAD (senão o caso não prova nada);
  3. IK do MoveIt2 (KDL, só posição): a solução leva o TCP ao ponto pedido;
     o ponto inalcançável tem de ser recusado.

Colisão no CAD = volume da interseção > 0,01 mm³ entre elos NÃO vizinhos
(montagem.VIZINHOS) ou entre um elo e a mesa (topo a mesa_topo_m). Os mordentes
fazem parte do link4 no CAD; contatos do MoveIt2 com mordente_* contam como link4.

Uso (de dentro de cad/; ~4 min com 11 processos):
    python conferir_moveit.py
"""

import itertools
import json
import math
import multiprocessing as mp
import os
import sys

AQUI = os.path.dirname(os.path.abspath(__file__))
PROJETO = os.path.dirname(AQUI)
ENTRADA = os.path.join(PROJETO, "resultados", "moveit_teste.json")
sys.path.insert(0, os.path.join(PROJETO, "ros2", "braco2_nos"))
from braco2_nos import cinematica as cin   # noqa: E402

BRACO = list(cin.NOMES)
VEL_URDF = {"junta_base": 4.817, "junta_ombro": 4.817, "junta_cotovelo": 4.817,
            "junta_punho": 7.330, "junta_garra": 0.1319}   # <limit velocity> do braco.urdf
LIM_GARRA = (0.0, 0.02)
PASSO_MAX = math.radians(1.0)    # entre pontos conferidos da trajetória
PASSO_GARRA = 0.001              # m (abertura total muda 2 mm por ponto)
VOL_MIN = 0.01                   # mm³: abaixo disso é ruído da booleana
NOME_CAD = {"mordente_cima": "link4", "mordente_baixo": "link4"}

# ----------------------------------------------------------------------------
# Colisão pelos sólidos (roda em vários processos; cada um monta o braço uma vez
# por abertura de garra e guarda)
# ----------------------------------------------------------------------------
_servo, _cache = None, {}


def _iniciar():
    global _servo
    import usar_lib  # noqa: F401  (só põe cad/lib/ no caminho de import)
    import servo_step
    _servo = servo_step.carregar()


def _elos(abertura_mm):
    import junta as j
    import montagem as m
    chave = round(abertura_mm, 1)
    if chave not in _cache:
        pecas = m.pecas_por_elo(_servo, chave)
        _cache[chave] = {n: j.unir([s for nome, s in l if "parafusos_horn" not in nome])
                         for n, l in pecas.items()}
    return _cache[chave]


def _sobrepoe_caixa(a, b):
    ba, bb = a.BoundingBox(), b.BoundingBox()
    return not (ba.xmax < bb.xmin or bb.xmax < ba.xmin or ba.ymax < bb.ymin or bb.ymax < ba.ymin
                or ba.zmax < bb.zmin or bb.zmax < ba.zmin)


def colisoes_cad(tarefa):
    """tarefa = (q {junta: rad/m}, z_mesa_m ou None, pares_para_distancia).
    Devolve (pares em colisão [(a, b, mm³)], {par: distância em mm})."""
    import cadquery as cq
    import montagem as m
    q, z_mesa, pedir_dist = tarefa
    graus = [math.degrees(q[j]) for j in BRACO]
    tf = m.cadeia(graus)
    elos = _elos(q.get("junta_garra", 0.0) * 2000.0)
    mundo = {n: tf[n](s) for n, s in elos.items()}
    if z_mesa is not None:
        mundo["mesa"] = cq.Workplane("XY").box(1200, 1200, 20).translate((0, 0, z_mesa * 1000 - 10)).val()
    out = []
    for a, b in itertools.combinations(mundo, 2):
        if (a, b) in m.VIZINHOS or (b, a) in m.VIZINHOS:
            continue
        if not _sobrepoe_caixa(mundo[a], mundo[b]):
            continue
        v = mundo[a].intersect(mundo[b]).Volume()
        if v > VOL_MIN:
            out.append((a, b, round(v, 2)))
    dist = {}
    for a, b in pedir_dist:
        if a in mundo and b in mundo:
            dist[f"{a} x {b}"] = round(mundo[a].distance(mundo[b]), 2)
    return out, dist


# ----------------------------------------------------------------------------
# Conferências
# ----------------------------------------------------------------------------
FALHAS = []


def conferir(nome, ok, info=""):
    print(f"  [{'OK' if ok else 'FALHOU'}] {nome}" + (f": {info}" if info else ""))
    if not ok:
        FALHAS.append(nome)
    return ok


def para_cad(par):
    a, b = (NOME_CAD.get(x, x) for x in par)
    return (a, b)


def convencoes():
    import montagem as m
    print("\n0. Convenções: cadeia de sólidos do CAD x cinemática do URDF (pacote de nós)")
    pior = 0.0
    for q in [(0, 0, 0, 0), (0.3, 0.5, -0.9, -0.6), (-1.2, 1.1, -0.4, 0.8), (2.5, -0.5, 1.2, -1.3)]:
        x, y, z, _ = cin.fk_analitica(*q)
        c = m.tcp_cadeia([math.degrees(a) for a in q])
        pior = max(pior, math.dist(c, (x * 1000, y * 1000, z * 1000)))
    conferir("TCP igual nas duas (4 poses)", pior < 0.01, f"maior diferença {pior:.4f} mm")


def validade(pool, dados, chave, com_mesa):
    itens = dados[chave]
    z = dados["mesa_topo_m"] if com_mesa else None
    tarefas = [(it["q"], z, [para_cad(p) for p in it["contatos"]] if not it["valido"] else [])
               for it in itens]
    resultados = pool.map(colisoes_cad, tarefas, chunksize=4)
    livres = colide = 0
    perigosas, seguras = [], []
    for it, (pares, dist) in zip(itens, resultados):
        livre_cad = not pares
        if livre_cad == it["valido"]:
            livres += livre_cad
            colide += not livre_cad
        elif it["valido"]:
            perigosas.append((it, pares))
        else:
            seguras.append((it, dist))
    print(f"\n1. Colisão, {chave} ({len(itens)} poses): concordam {livres} livres + {colide} em colisão; "
          f"discordam {len(perigosas) + len(seguras)}")
    for it, pares in perigosas:
        print(f"     PERIGOSA: MoveIt2 diz livre, CAD colide {pares}; q = "
              f"{ {k: round(math.degrees(v), 1) if k != 'junta_garra' else v for k, v in it['q'].items()} }")
    for it, dist in seguras:
        print(f"     segura: MoveIt2 diz colide {it['contatos']}, CAD livre; distância no CAD: {dist}")
    if seguras:
        ds = [min(d.values()) for _, d in seguras if d]
        print(f"     -> nas seguras, os sólidos do CAD ficam a {min(ds):.2f} a {max(ds):.2f} mm "
              f"(a caixa de colisão é a caixa da peça + 1 mm de margem)")
    conferir(f"{chave}: nenhuma pose que o MoveIt2 libera colide no CAD", not perigosas, f"{len(perigosas)} perigosas")
    return len(itens), livres + colide, seguras


def trajetorias(pool, dados):
    print("\n2. Trajetórias planejadas e executadas (conferidas no CAD, com a mesa)")
    z = dados["mesa_topo_m"]
    escala = dados["escala_velocidade"]
    lim = dict(cin.LIMITES, junta_garra=LIM_GARRA)
    for mov in dados["movimentos"]:
        prob = []
        if not mov["ok"]:
            conferir(mov["nome"], False, f"MoveIt2 devolveu código {mov['codigo']}")
            continue
        estado = {j: mov["estado_inicial"][j] for j in BRACO + ["junta_garra"]}
        pontos = [dict(estado, **dict(zip(mov["juntas"], p["q"]))) for p in mov["pontos"]]
        if max(abs(pontos[0][j] - estado[j]) for j in estado) > 0.002:
            prob.append("trajetória não começa onde o robô estava")
        confere = [pontos[0]]
        for a, b in zip(pontos, pontos[1:]):
            n = max(1, math.ceil(max(max(abs(b[j] - a[j]) for j in BRACO) / PASSO_MAX,
                                     abs(b["junta_garra"] - a["junta_garra"]) / PASSO_GARRA)))
            confere += [{j: a[j] + k / n * (b[j] - a[j]) for j in a} for k in range(1, n + 1)]
        for q in confere:
            fora = [j for j in q if not lim[j][0] - 1e-6 <= q[j] <= lim[j][1] + 1e-6]
            if fora:
                prob.append(f"fora do limite: {fora}")
        res = pool.map(colisoes_cad, [(q, z, []) for q in confere], chunksize=4)
        bate = [pares for pares, _ in res if pares]
        if bate:
            prob.append(f"{len(bate)} poses colidem no CAD, ex.: {bate[0]}")
        for p in mov["pontos"]:
            for j, v in zip(mov["juntas"], p["v"]):
                if abs(v) > VEL_URDF[j] * escala + 1e-6:
                    prob.append(f"{j} a {abs(v):.3f} (máximo pedido {VEL_URDF[j] * escala:.3f})")
        for j, alvo in mov["alvo"].items():
            tol = 0.0002 if j == "junta_garra" else 0.002
            if abs(mov["estado_final"].get(j, 1e9) - alvo) > tol:
                prob.append(f"{j} parou em {mov['estado_final'].get(j):.4f}, alvo {alvo:.4f}")
        dur = mov["pontos"][-1]["t"]
        conferir(mov["nome"], not prob, f"{len(mov['pontos'])} pontos, {len(confere)} poses no CAD, "
                 f"{dur:.2f} s" + ("; " + "; ".join(sorted(set(prob))[:4]) if prob else ""))

    # O caso difícil só prova algo se a reta colide também no CAD
    cd = dados["caso_dificil"]
    reta = [dict({j: cd["de"][j] + k / 40 * (cd["para"][j] - cd["de"][j]) for j in BRACO}, junta_garra=0.0)
            for k in range(41)]
    res = pool.map(colisoes_cad, [(q, z, []) for q in reta])
    bate = [(k, pares) for k, (pares, _) in enumerate(res) if pares]
    conferir("caso difícil é real: a reta nas juntas colide no CAD", bool(bate),
             f"{len(bate)} de 41 poses, ex.: passo {bate[0][0]}: {bate[0][1]}" if bate else "a reta é livre")


def ik(dados):
    print("\n3. IK do MoveIt2 (KDL, só posição)")
    for r in dados["ik"]:
        x, y, z = r["alvo"]
        dist = math.hypot(math.hypot(x, y), z - cin.Z_OMBRO)
        alcancavel = dist <= cin.L2 + cin.L3 + cin.L4
        if r["codigo"] == 1 and r["q"]:
            px, py, pz, phi = cin.fk_analitica(*r["q"])
            erro = math.dist((px, py, pz), (x, y, z)) * 1000
            dentro = cin.dentro_dos_limites(r["q"])
            conferir(f"alvo {r['alvo']}", erro < 1.0 and dentro,
                     f"TCP a {erro:.3f} mm do alvo, {'dentro' if dentro else 'FORA'} dos limites, "
                     f"garra saiu a {math.degrees(phi):+.1f}° (qualquer: só posição)")
        else:
            conferir(f"alvo {r['alvo']}", not alcancavel,
                     f"recusado (código {r['codigo']}); está a {dist * 1000:.0f} mm do ombro, "
                     f"alcance {(cin.L2 + cin.L3 + cin.L4) * 1000:.1f} mm")


def main():
    with open(ENTRADA, encoding="utf-8") as f:
        dados = json.load(f)
    _iniciar()
    convencoes()
    conferir("mesa entrou na cena do MoveIt2", dados["mesa_na_cena"])
    with mp.Pool(max(1, min(11, (os.cpu_count() or 2) - 1)), initializer=_iniciar) as pool:
        n1, c1, s1 = validade(pool, dados, "validade_sem_mesa", com_mesa=False)
        n2, c2, s2 = validade(pool, dados, "validade_com_mesa", com_mesa=True)
        trajetorias(pool, dados)
    ik(dados)
    print(f"\nResumo: colisão MoveIt2 x CAD concorda em {c1 + c2} de {n1 + n2} poses; "
          f"{len(s1) + len(s2)} discordâncias do lado seguro.")
    if FALHAS:
        sys.exit(f"FALHOU: {FALHAS}")
    print("Tudo conferido.")


if __name__ == "__main__":
    main()
