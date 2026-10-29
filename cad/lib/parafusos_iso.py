#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
parafusos_iso.py
================
Dados de parafuso, furo de passagem e porca, e a regra de comprimento de
parafuso em furo roscado cego. ÚNICO lugar destes números: os projetos
importam daqui.

Origem: Projeto 2 (decisão de 2026-10-13); movido para a biblioteca em
2026-10-14, no 2º uso real (experimento Eternatus). Testes:
testar_parafusos_iso.py, ao lado.

As normas ISO são pagas. Cada tabela abaixo foi lida em 2026-10-13 numa
reprodução pública e conferida contra uma segunda fonte quando havia.
Só entram as colunas que o projeto usa. Valores MÁXIMOS de cabeça e porca,
porque são o pior caso para folga de alojamento.

Fontes:
  ISO 4762 (allen) — fasten.it, tabela "DIN EN ISO 4762", edição 06-2004:
    https://fasten.it/norms/norm/din_en_iso_4762
    Segunda fonte para M2,5: ficha Würth 0094253 (dk 4,5; k 2,5).
    CUIDADO: a mesma página traz k min 2,66 para M2,5, maior que o k max 2,5
    (erro da página). k min não é usado aqui.
  ISO 273 (furo de passagem) — Optimas, "Tapping sizes" (a página não cita
    a norma): https://optimas.com/technical-resources/tapping-sizes/
    M2 e M2,5 batem com uma reprodução da tabela ISO 273-1979 (164580.com).
    M3 só tem a Optimas.
  ISO 4032 (porca) — fasten.it, tabela "DIN EN ISO 4032", edição 04-2013
    (igual à de 03-2001 nestes tamanhos):
    https://fasten.it/norms/norm/din_en_iso_4032
    Segunda fonte: Fabory (M2,5: m 2, s 5) e tspi.at (M3: 2,4 / 5,5 / 6,01).
  Rebaixo para ISO 4762 (a norma é a DIN 974-1:2008, paga, não acessada) —
    The Engineer's Bible, "Counterbore Hole Size for Socket Head (ISO 4762)"
    (não cita norma): https://engineersbible.com/counterbore-socket-iso/
    Os furos de passagem da mesma tabela batem com a ISO 273 acima.
    M3 confere com a Mechcodex ("DIN 974-style tables"):
    https://mechcodex.com/reference/counterbore-countersink-dimensions
    M2 e M2,5 têm UMA fonte só.
"""

# ---------------------------------------------------------------------------
# ISO 4762 — parafuso allen de cabeça cilíndrica (mm)
# ---------------------------------------------------------------------------
# dk = diâmetro da cabeça, k = altura da cabeça, s = chave allen.
# "comprimentos": lidos da tabela de massa da fasten.it (a página não traz
# lista separada). Conferir disponibilidade no fornecedor antes de comprar.
PARAFUSO_ISO4762 = {
    "M2":   {"d": 2.0, "passo": 0.40, "dk_max": 3.8, "k_max": 2.0, "s": 1.5,
             "comprimentos": (3, 4, 5, 6, 8, 10, 12, 16)},
    "M2.5": {"d": 2.5, "passo": 0.45, "dk_max": 4.5, "k_max": 2.5, "s": 2.0,
             "comprimentos": (4, 5, 6, 8, 10, 12, 16)},
    "M3":   {"d": 3.0, "passo": 0.50, "dk_max": 5.5, "k_max": 3.0, "s": 2.5,
             "comprimentos": (5, 6, 8, 10, 12, 16)},
}

# ---------------------------------------------------------------------------
# ISO 273 — furo de passagem (mm): séries fina, média e grossa
# ---------------------------------------------------------------------------
FURO_PASSAGEM_ISO273 = {
    "M2":   {"fino": 2.2, "medio": 2.4, "grosso": 2.6},
    "M2.5": {"fino": 2.7, "medio": 2.9, "grosso": 3.1},
    "M3":   {"fino": 3.2, "medio": 3.4, "grosso": 3.6},
}

# ASSUMIDO (decisão de 2026-10-12): série média até existir a peça de teste
# impressa. Furo impresso sai menor que o desenhado; a peça de teste dirá
# quanto. Trocar AQUI, e só aqui.
SERIE_FURO_IMPRESSO = "medio"

# ---------------------------------------------------------------------------
# ISO 4032 — porca sextavada (mm)
# ---------------------------------------------------------------------------
# s = medida entre faces (chave), e_min = entre cantos, m_max = altura.
PORCA_ISO4032 = {
    "M2":   {"s": 4.0, "e_min": 4.32, "m_max": 1.6},
    "M2.5": {"s": 5.0, "e_min": 5.45, "m_max": 2.0},
    "M3":   {"s": 5.5, "e_min": 6.01, "m_max": 2.4},
}


# ---------------------------------------------------------------------------
# Rebaixo para a cabeça do ISO 4762 (mm)
# ---------------------------------------------------------------------------
# Só o DIÂMETRO vem da tabela. A profundidade da tabela (cabeça rente) não
# serve aqui: no nosso caso ela sai da regra do parafuso (rebaixo_para_rosca_cega).
# Valor de usinagem em metal. Impresso sai menor: ASSUMIDO até a peça de teste,
# igual ao furo de passagem.
REBAIXO_DIAMETRO = {"M2": 4.4, "M2.5": 5.5, "M3": 6.5}


def furo_passagem(rosca, serie=None):
    """Diâmetro do furo de passagem pela ISO 273. Sem série: a do impresso."""
    return FURO_PASSAGEM_ISO273[rosca][serie or SERIE_FURO_IMPRESSO]


def parafuso_em_rosca_cega(rosca, espessura_presa, prof_max_rosca,
                           folga_fundo, engajamento_min):
    """
    Escolhe o comprimento padrão de parafuso que atravessa uma peça de
    espessura `espessura_presa` (da face de apoio da cabeça até a face do
    furo roscado) e entra num furo roscado cego.

        engajamento = comprimento - espessura_presa

    Regras (todas em mm):
      engajamento <= prof_max_rosca - folga_fundo   (a ponta não bate no fundo)
      engajamento >= engajamento_min                (rosca suficiente para segurar)
    Entre os comprimentos que passam, devolve o MAIOR (mais rosca segurando).

    `engajamento_min` e `folga_fundo` não têm valor padrão de propósito: não
    há fonte para eles, quem chama decide e marca ASSUMIDO.

    Devolve (comprimento, engajamento). Se nenhum comprimento padrão cabe,
    levanta ValueError dizendo a faixa que faltou: aí muda a espessura.
    """
    p = PARAFUSO_ISO4762[rosca]
    eng_max = prof_max_rosca - folga_fundo
    if eng_max < engajamento_min:
        raise ValueError(f"{rosca}: faixa vazia, engajamento máx {eng_max:.2f} "
                         f"< mín {engajamento_min:.2f} mm, nenhuma espessura resolve")
    validos = [L for L in p["comprimentos"]
               if engajamento_min - 1e-9 <= L - espessura_presa <= eng_max + 1e-9]
    if not validos:
        raise ValueError(
            f"{rosca}: nenhum comprimento de {p['comprimentos']} dá engajamento "
            f"entre {engajamento_min:.2f} e {eng_max:.2f} mm com {espessura_presa:.2f} mm "
            f"de peça presa. Espessuras que servem: comprimento - "
            f"[{engajamento_min:.2f} .. {eng_max:.2f}]")
    L = max(validos)
    return L, round(L - espessura_presa, 6)


def rebaixo_para_rosca_cega(rosca, espessura_chapa, prof_max_rosca,
                            folga_fundo, engajamento_min, fundo_min):
    """
    Mesma situação de parafuso_em_rosca_cega, mas a chapa pode ter um
    rebaixo para a cabeça. Assim a espessura da chapa (que vem da
    resistência) e a espessura PRESA sob a cabeça (que vem do parafuso)
    deixam de ser a mesma cota:

        rebaixo   = espessura_chapa - presa
        engajamento = comprimento - presa

    Para cada comprimento padrão, escolhe a presa que dá o MAIOR engajamento
    permitido, sem deixar o fundo do rebaixo mais fino que `fundo_min`
    (plástico que a cabeça aperta; sem valor padrão, quem chama marca ASSUMIDO).

    Devolve uma lista de opções, do parafuso mais curto ao mais longo, cada
    uma um dict: comprimento, presa, engajamento, rebaixo (profundidade) e
    saliencia_cabeca (quanto a cabeça fica para FORA da chapa; negativo =
    cabeça embutida). Escolher entre elas é decisão de projeto de quem chama.
    Sem opção nenhuma: ValueError.
    """
    p = PARAFUSO_ISO4762[rosca]
    eng_max = prof_max_rosca - folga_fundo
    opcoes = []
    for L in p["comprimentos"]:
        presa = max(L - eng_max, fundo_min)
        if presa > min(espessura_chapa, L - engajamento_min) + 1e-9:
            continue
        rebaixo = espessura_chapa - presa
        opcoes.append({"comprimento": L,
                       "presa": round(presa, 6),
                       "engajamento": round(L - presa, 6),
                       "rebaixo": round(rebaixo, 6),
                       "saliencia_cabeca": round(p["k_max"] - rebaixo, 6)})
    if not opcoes:
        raise ValueError(
            f"{rosca}: nenhum comprimento serve numa chapa de {espessura_chapa:.2f} mm "
            f"(engajamento {engajamento_min:.2f}..{eng_max:.2f}, fundo mínimo {fundo_min:.2f})")
    return opcoes


def fios_engajados(rosca, engajamento):
    """Quantos fios de rosca seguram: engajamento / passo."""
    return engajamento / PARAFUSO_ISO4762[rosca]["passo"]
