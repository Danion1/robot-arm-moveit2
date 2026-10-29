#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
resistencia_forquilha.py
========================
Que espessura a chapa do HORN da forquilha precisa ter para aguentar o
torque de stall do servo (decisão de 2026-10-13) com fator de segurança 3.

Caminho de carga: o torque entra só pelo horn (o idler gira livre), passa
pelos parafusos M2 para a chapa do horn e dela, dobrando no próprio plano,
até a ponte. As juntas que levantam peso têm eixo horizontal, então a
gravidade também dobra a chapa no plano (lado forte), nunca fora dele.

Duas falhas, ambas conservadoras (todas as aproximações pioram o resultado):

1. Esmagamento do furo. Pior caso: o atrito sob as cabeças não ajuda e o
   torque passa inteiro pela haste empurrando a parede do furo, dividido
   igualmente entre os parafusos:
       F = T / (n r)          força em cada parafuso
       sigma = F / (d presa)  pressão no furo
   Só a espessura PRESA encosta na haste: o rebaixo está vazio.

2. Flexão no plano da chapa. Momento = T inteiro em qualquer seção entre o
   horn e a ponte (a carga real é T x (1 - s/L), sempre menor). Duas seções:
     A) passando pelo eixo, pela linha de furos em x = +-r: largura w menos o
        furo central e os dois furos. Cada furo é descontado com a largura
        do REBAIXO na espessura toda (conservador);
     B) logo depois dos furos, sem furo nenhum: largura w cheia.
       sigma = M c / I,  c = w/2,  I = t w^3/12 - furos (Steiner: t(b^3/12 + b x^2))

Resistência: tensão de escoamento da ficha do PLA (tração, material maciço).
Compressão do PLA costuma ser maior, então usar a de tração no esmagamento
também é conservador. Peça impressa é mais fraca que maciço: é para isso o
fator 3, ASSUMIDO.

O que NÃO entra: a distribuição desigual entre parafusos (com folga no furo,
poucos encostam primeiro), concentração de tensão na borda dos furos,
fluência do PLA, chapa do idler (quase sem carga), cisalhamento da ponte.

Uso (de dentro de cad/):
    python resistencia_forquilha.py
"""

import usar_lib  # noqa: F401
import dados_materiais as mat
import dados_servos_dynamixel as sv

import parafusos_iso as pi

TORQUE_PROJETO_NM = sv.SERVOS["XM430-W350-T"]["torque_stall_nm"]   # 4,1 (decisão 2026-10-13)
ESCOAMENTO_MPA = mat.PLA["tensao_escoamento_mpa"]                  # 35,9
FATOR_SEGURANCA_MIN = 3.0   # ASSUMIDO (decisão 2026-10-13), trocar pela peça de teste

RAIO_PARAFUSOS_MM = sv.X430_FURACAO["horn_m2"]["pcd_mm"] / 2.0     # 8,0
D_PARAFUSO_MM = pi.PARAFUSO_ISO4762["M2"]["d"]                     # 2,0
# Furo central: passa a saliência Ø8 do horn (e a cabeça Ø4,5 do parafuso
# central, que fica dentro dela). Folga diametral de 1,0 ASSUMIDA, mesma
# lógica do furo de passagem (impresso sai menor).
FOLGA_SALIENCIA_MM = 1.0
D_FURO_CENTRAL_MM = sv.X430_HORN_SALIENCIA_CENTRAL["diametro_mm"] + FOLGA_SALIENCIA_MM


def forca_por_parafuso(torque_nmm, n, raio_mm):
    """N, torque dividido igualmente entre n parafusos no raio dado."""
    return torque_nmm / (n * raio_mm)


def pressao_no_furo(forca_n, d_mm, presa_mm):
    """MPa (N/mm²) que a haste faz na parede do furo."""
    return forca_n / (d_mm * presa_mm)


def inercia_no_plano(w_mm, t_mm, furos):
    """
    mm^4, momento de inércia da seção retangular w x t para flexão no plano
    da chapa, menos os furos. `furos` = lista de (posição x do centro, largura
    descontada). Teorema de Steiner: cada furo tira t (b³/12 + b x²).
    """
    i = t_mm * w_mm ** 3 / 12.0
    for x, b in furos:
        i -= t_mm * (b ** 3 / 12.0 + b * x ** 2)
    return i


def tensao_flexao(momento_nmm, w_mm, t_mm, furos):
    """MPa na fibra mais afastada (borda da chapa, c = w/2)."""
    return momento_nmm * (w_mm / 2.0) / inercia_no_plano(w_mm, t_mm, furos)


def avaliar(t_mm, w_mm, presa_mm, n_parafusos, torque_nm=TORQUE_PROJETO_NM):
    """Fator de segurança de cada falha para uma chapa. Devolve dict."""
    T = torque_nm * 1000.0                                   # N·mm
    f = forca_por_parafuso(T, n_parafusos, RAIO_PARAFUSOS_MM)
    s_furo = pressao_no_furo(f, D_PARAFUSO_MM, presa_mm)
    furos_a = [(0.0, D_FURO_CENTRAL_MM),
               (RAIO_PARAFUSOS_MM, pi.REBAIXO_DIAMETRO["M2"]),
               (-RAIO_PARAFUSOS_MM, pi.REBAIXO_DIAMETRO["M2"])]
    s_a = tensao_flexao(T, w_mm, t_mm, furos_a)
    s_b = tensao_flexao(T, w_mm, t_mm, [])
    fs = {"esmagamento": ESCOAMENTO_MPA / s_furo,
          "flexao_secao_A": ESCOAMENTO_MPA / s_a,
          "flexao_secao_B": ESCOAMENTO_MPA / s_b}
    return {"forca_parafuso_n": f, "tensoes_mpa": (s_furo, s_a, s_b),
            "fs": fs, "passa": min(fs.values()) >= FATOR_SEGURANCA_MIN}


if __name__ == "__main__":
    print(f"Torque de projeto {TORQUE_PROJETO_NM} N·m (stall XM430), PLA escoa a "
          f"{ESCOAMENTO_MPA} MPa, FS mínimo {FATOR_SEGURANCA_MIN} (ASSUMIDO)")
    print(f"Furo central Ø{D_FURO_CENTRAL_MM}, rebaixo Ø{pi.REBAIXO_DIAMETRO['M2']}, "
          f"parafusos no raio {RAIO_PARAFUSOS_MM}\n")
    # Régua da ROBOTIS: 1,5 de rosca no horn, 0,5 até o fundo de 2,0.
    # fundo_min = 1,0 só para listar opções; o esmagamento decide de verdade.
    for w in (24.0, sv.X430_LARGURA):
        for n in (8, 4):
            print(f"largura {w} mm, {n} parafusos M2:")
            print(f"  {'chapa':>5} {'parafuso':>8} {'presa':>5} {'rebaixo':>7} "
                  f"{'FS furo':>7} {'FS A':>5} {'FS B':>5}")
            for t in (2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0):
                try:
                    ops = pi.rebaixo_para_rosca_cega("M2", t, 2.0, 0.5, 1.5, 1.0)
                except ValueError:
                    continue
                for o in ops:
                    r = avaliar(t, w, o["presa"], n)
                    fs = r["fs"]
                    marca = "ok" if r["passa"] else "  "
                    print(f"  {t:5.1f}   M2x{o['comprimento']:<3d} {o['presa']:5.1f} "
                          f"{o['rebaixo']:7.1f} {fs['esmagamento']:7.2f} "
                          f"{fs['flexao_secao_A']:5.2f} {fs['flexao_secao_B']:5.2f}  {marca}")
            print()
