# -*- coding: utf-8 -*-
"""
servo_step.py
=============
Dono do caminho e do SHA-256 do STEP oficial do X430 (fora do Git: arquivo do
fabricante; como baixar em docs/sources.md). Quem precisa do servo como sólido (medição,
verificação de colisão, imagem, montagem STEP) importa daqui.

Sólidos do STEP (conferido em 2026-10-13): 0 = corpo + horn + idler;
1..16 = parafusos e anéis da carcaça; 17 = capa do cabo (lado do idler,
ponta longa).
"""

import hashlib
import os
import sys

import cadquery as cq

AQUI = os.path.dirname(os.path.abspath(__file__))
STEP = os.path.normpath(os.path.join(AQUI, "..", "fontes", "fabricante", "XM,H-430_idler.stp"))
STEP_SHA256 = "7ff4e39475245d5c1fc4f703e9241fca1a09d57aed920274498dbe2cd5e31e22"


def conferir_arquivo_de_fabricante(caminho, sha256_esperado):
    """Para com mensagem clara se o arquivo de terceiros (fora do Git) faltar
    ou não for a versão registrada no docs/sources.md."""
    if not os.path.isfile(caminho):
        sys.exit(f"ERRO: não achei {caminho}\n"
                 "Arquivo de fabricante, fora do Git: baixar (como em docs/sources.md).")
    sha = hashlib.sha256(open(caminho, "rb").read()).hexdigest()
    if sha != sha256_esperado:
        sys.exit(f"ERRO: {os.path.basename(caminho)} não é a versão registrada no docs/sources.md\n"
                 f"  esperado {sha256_esperado}\n  achado   {sha}")


def carregar():
    """Confere o hash e devolve o Workplane com os 18 sólidos do servo."""
    conferir_arquivo_de_fabricante(STEP, STEP_SHA256)
    return cq.importers.importStep(STEP)
