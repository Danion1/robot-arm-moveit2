#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_cinematica.py
==================
Testes da cinemática (unittest; o pytest do `colcon test` também roda).

    cd ros2/braco2_nos && python3 -m pytest test/test_cinematica.py -v

O que cada grupo prova:
  A. direta contra contas que saem de cabeça (somar comprimentos);
  B. as duas diretas (fórmula e matrizes) concordam;
  C. inversa contra as contas feitas À MÃO pelo Daniel (2026-10-14), limites,
     fora do alcance e ida e volta em 2000 poses aleatórias dentro dos limites;
  D. geometria e limites iguais aos do URDF, e a direta igual à cadeia do URDF.
A conferência contra os SÓLIDOS do CAD fica em `cad/conferir_cinematica.py`
(precisa de CadQuery, que o ROS não tem).
"""

import math
import os
import random
import unittest
import xml.etree.ElementTree as ET

from braco2_nos import cinematica as cin

AQUI = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(AQUI, "..", "..", "braco2_description", "urdf", "braco.urdf")
rad = math.radians
TOL = 1e-9
Z0 = 0.08825          # altura do ombro


class A_DiretaContraContaAMao(unittest.TestCase):

    def confere(self, q_graus, esperado):
        obtido = cin.fk_analitica(*(rad(a) for a in q_graus))
        for o, e in zip(obtido, esperado):
            self.assertAlmostEqual(o, e, delta=TOL)

    def test_pose_zero_esticado_para_frente(self):
        # 105 + 90 + 83,5 = 278,5 mm à frente, na altura do ombro.
        self.confere((0, 0, 0, 0), (0.2785, 0.0, Z0, 0.0))

    def test_esticado_para_cima(self):
        self.confere((0, 90, 0, 0), (0.0, 0.0, Z0 + 0.2785, rad(90)))

    def test_base_a_90_aponta_para_a_esquerda(self):
        self.confere((90, 0, 0, 0), (0.0, 0.2785, Z0, 0.0))

    def test_braco_em_L(self):
        # Braço para cima (105), antebraço e mão na horizontal (90 + 83,5).
        self.confere((0, 90, -90, 0), (0.1735, 0.0, Z0 + 0.105, 0.0))

    def test_mao_apontando_para_baixo(self):
        # Como o "L", mas a mão (83,5) desce.
        self.confere((0, 90, -90, -90), (0.090, 0.0, Z0 + 0.105 - 0.0835, rad(-90)))


class B_DuasDiretasConcordam(unittest.TestCase):

    def test_formula_igual_a_matrizes(self):
        rng = random.Random(7)
        for _ in range(2000):
            q = [rng.uniform(-math.pi, math.pi) for _ in range(4)]
            a = cin.fk_analitica(*q)
            b = cin.pose_do_tcp(cin.fk_matrizes(*q))
            for k in range(3):
                self.assertAlmostEqual(a[k], b[k], delta=1e-12)
            self.assertAlmostEqual(cin.normaliza(a[3] - b[3]), 0.0, delta=1e-9)


class C_Inversa(unittest.TestCase):

    def test_exemplo_a_mao_garra_para_baixo(self):
        # r = 150, z = 50, phi = -90°: conta do Daniel, cotovelo para cima.
        q = cin.ik(0.150, 0.0, 0.050, rad(-90), cotovelo="cima")
        for o, e in zip(q, (0.0, 50.17, -73.33, -66.85)):
            self.assertAlmostEqual(math.degrees(o), e, delta=0.01)

    def test_exemplo_a_mao_so_uma_solucao_executavel(self):
        # Cotovelo para baixo pede punho a -146,73°: fora do limite de ±90°.
        baixo = cin.ik(0.150, 0.0, 0.050, rad(-90), cotovelo="baixo")
        self.assertAlmostEqual(math.degrees(baixo[3]), -146.73, delta=0.01)
        sols = cin.solucoes(0.150, 0.0, 0.050, rad(-90))
        self.assertEqual(len(sols), 1)
        self.assertLess(sols[0][2], 0.0)          # é a do cotovelo para cima

    def test_exercicio_a_mao_cotovelo_passa_do_limite(self):
        # r = 180, z = 88,25, phi = 0: o triângulo fecha (q3 = ±121,28°),
        # mas o cotovelo passa de ±90°. Alcançável no papel, não no braço.
        q = cin.ik(0.180, 0.0, Z0, 0.0, cotovelo="cima")
        self.assertAlmostEqual(math.degrees(q[2]), -121.28, delta=0.01)
        self.assertAlmostEqual(math.degrees(q[1]), 52.85, delta=0.01)
        self.assertEqual(cin.solucoes(0.180, 0.0, Z0, 0.0), [])

    def test_alvo_longe_demais_avisa(self):
        # r = 290: punho a 206,5 mm do ombro, mais que L2 + L3 = 195.
        with self.assertRaises(cin.ForaDoAlcance):
            cin.ik(0.290, 0.0, Z0, 0.0)
        self.assertEqual(cin.solucoes(0.290, 0.0, Z0, 0.0), [])

    def test_alvo_no_limite_do_alcance(self):
        for o in cin.ik(0.2785, 0.0, Z0, 0.0):
            self.assertAlmostEqual(o, 0.0, delta=1e-6)

    def test_ida_e_volta_dentro_dos_limites(self):
        rng = random.Random(11)
        n = 0
        while n < 2000:
            q = tuple(rng.uniform(lo, hi) for lo, hi in cin.LIMITES.values())
            if abs(q[2]) < 1e-3:          # cotovelo reto: os dois ramos coincidem
                continue
            if cin.fk_analitica(0.0, *q[1:])[0] <= 1e-6:
                continue                  # TCP atrás do eixo da base: fora do método
            x, y, z, phi = cin.fk_analitica(*q)
            ramo = "cima" if q[2] < 0 else "baixo"
            q_r = cin.ik(x, y, z, phi, cotovelo=ramo)
            for a, b in zip(cin.fk_analitica(*q_r)[:3], (x, y, z)):
                self.assertAlmostEqual(a, b, delta=1e-9)      # 1 nanômetro
            for a, b in zip(q_r, q):
                self.assertAlmostEqual(cin.normaliza(a - b), 0.0, delta=1e-6)
            if cin.dentro_dos_limites(q_r):   # na borda, o arredondamento pode tirar
                self.assertIn(q_r, cin.solucoes(x, y, z, phi))
            n += 1


def juntas_do_urdf():
    raiz = ET.parse(URDF).getroot()
    return {j.get("name"): j for j in raiz.findall("joint")}


def fk_pelo_urdf(angulos):
    """Posição do TCP lendo origem, rpy e eixo de cada junta do arquivo URDF."""
    juntas = {j.find("child").get("link"): j for j in juntas_do_urdf().values()}
    cadeia, link = [], "tcp"
    while link in juntas:
        cadeia.append(juntas[link])
        link = juntas[link].find("parent").get("link")
    t = cin.mat_identidade()
    for j in reversed(cadeia):
        xyz = [float(v) for v in j.find("origin").get("xyz").split()]
        rpy = [float(v) for v in j.find("origin").get("rpy").split()]
        t = cin.mat_mul(t, cin.mat_translacao(*xyz))
        for eixo, ang in (((0, 0, 1), rpy[2]), ((0, 1, 0), rpy[1]), ((1, 0, 0), rpy[0])):
            t = cin.mat_mul(t, cin.mat_rotacao(eixo, ang))
        if j.get("type") == "revolute":
            eixo = [float(v) for v in j.find("axis").get("xyz").split()]
            t = cin.mat_mul(t, cin.mat_rotacao(eixo, angulos[j.get("name")]))
    return (t[0][3], t[1][3], t[2][3])


@unittest.skipUnless(os.path.isfile(URDF), "URDF não encontrado ao lado deste pacote")
class D_IgualAoUrdf(unittest.TestCase):

    def test_geometria_igual_ao_urdf(self):
        j = juntas_do_urdf()
        def origem(nome):
            return [float(v) for v in j[nome].find("origin").get("xyz").split()]
        self.assertEqual(origem("junta_base"), [0.0, 0.0, cin.Z_JUNTA1])
        self.assertEqual(origem("junta_ombro"), [0.0, 0.0, cin.Z_OMBRO_NO_LINK1])
        self.assertEqual(origem("junta_cotovelo"), [cin.L2, 0.0, 0.0])
        self.assertEqual(origem("junta_punho"), [cin.L3, 0.0, 0.0])
        self.assertEqual(origem("junta_tcp"), [cin.L4, 0.0, 0.0])

    def test_limites_iguais_ao_urdf(self):
        j = juntas_do_urdf()
        for nome, (lo, hi) in cin.LIMITES.items():
            lim = j[nome].find("limit")
            self.assertEqual((float(lim.get("lower")), float(lim.get("upper"))), (lo, hi), nome)

    def test_fk_igual_a_cadeia_do_urdf(self):
        rng = random.Random(3)
        for _ in range(500):
            q = [rng.uniform(-math.pi, math.pi) for _ in range(4)]
            angulos = dict(zip(cin.NOMES, q))
            for a, b in zip(fk_pelo_urdf(angulos), cin.fk_analitica(*q)[:3]):
                self.assertAlmostEqual(a, b, delta=1e-9)


if __name__ == "__main__":
    unittest.main()
