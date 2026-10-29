#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
testar_resistencia_forquilha.py — confere resistencia_forquilha.py contra
conta à mão e contra casos que TÊM que reprovar. Rodar de dentro de cad/:

    python testar_resistencia_forquilha.py
"""

import unittest

import resistencia_forquilha as rf


class ContaAMao(unittest.TestCase):
    def test_exemplo_da_conversa(self):
        # 4,1 N·m, 8 parafusos no raio 8: F = 4100 / 64 = 64,06 N.
        # Chapa presa 3 mm, haste Ø2: 64,06 / 6 = 10,68 MPa.
        f = rf.forca_por_parafuso(4100.0, 8, 8.0)
        self.assertAlmostEqual(f, 64.0625)
        self.assertAlmostEqual(rf.pressao_no_furo(f, 2.0, 3.0), 10.677, 3)

    def test_secao_cheia_e_a_formula_classica(self):
        # Sem furos: sigma = 6 M / (t w²). M 4100, t 3, w 24 -> 24600/1728 = 14,236
        self.assertAlmostEqual(rf.tensao_flexao(4100.0, 24.0, 3.0, []), 14.2361, 3)

    def test_steiner_com_furos(self):
        # w 24, t 3; furo central 9 e dois de 4,4 a +-8:
        #   cheia   3·24³/12            = 3456
        #   central 3·9³/12             =  182,25
        #   laterais 2·3·(4,4³/12+4,4·64) = 2·3·(7,0987+281,6) = 1732,19
        #   sobra 1541,56
        i = rf.inercia_no_plano(24.0, 3.0, [(0, 9.0), (8, 4.4), (-8, 4.4)])
        self.assertAlmostEqual(i, 3456 - 182.25 - 2 * 3 * (4.4 ** 3 / 12 + 4.4 * 64), 6)
        self.assertAlmostEqual(i, 1541.56, 1)


class Criterio(unittest.TestCase):
    def test_chapa_fina_reprova(self):
        # 2,5 mm de chapa, presa 2,5, 4 parafusos: esmagamento 128/5 = 25,6 MPa,
        # FS 1,4. Tem que reprovar.
        self.assertFalse(rf.avaliar(2.5, 24.0, 2.5, 4)["passa"])

    def test_mais_parafusos_mais_presa_ajuda(self):
        a = rf.avaliar(4.0, 24.0, 2.5, 8)["fs"]["esmagamento"]
        b = rf.avaliar(4.0, 24.0, 3.5, 8)["fs"]["esmagamento"]
        c = rf.avaliar(4.0, 24.0, 3.5, 4)["fs"]["esmagamento"]
        self.assertGreater(b, a)
        self.assertAlmostEqual(b / c, 2.0)

    def test_dados_vem_das_fontes(self):
        self.assertEqual(rf.TORQUE_PROJETO_NM, 4.1)
        self.assertEqual(rf.ESCOAMENTO_MPA, 35.9)
        self.assertEqual(rf.D_FURO_CENTRAL_MM, 9.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
