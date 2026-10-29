# -*- coding: utf-8 -*-
"""
exportar_stl_mm — exporta um STL binário em MILÍMETROS.

Quando usar: depois que o check_bbox passou. Chamar só dentro de
`if __name__ == "__main__":`; importar o script de CAD deve apenas
construir e verificar, porque outros scripts (massa, colisão) o importam.

Entradas: um cq.Compound (todos os corpos do link), a pasta e o nome do
arquivo. Saída: o arquivo, com tolerance=0.05 e angularTolerance=0.25.

Regra que acompanha: STL não
guarda unidade, e os parâmetros unit/outputUnit do exportador não fazem
nada. O arquivo sai em mm e todo <mesh> do URDF leva
scale="0.001 0.001 0.001".

Depois de exportar: copiar para <robô>_description/meshes/. No P1 as
malhas do pacote estavam somente-leitura (ver o CLAUDE.md daquele projeto).

Origem: P1/build_chassi.py e BR/build_braco.py (praticamente idênticas),
extraído em 2026-10-10.
"""

import os

import cadquery as cq


def exportar_stl_mm(compound, pasta, nome_arquivo):
    os.makedirs(pasta, exist_ok=True)
    path = os.path.join(pasta, nome_arquivo)
    cq.exporters.export(compound, path, exportType="STL",
                        tolerance=0.05, angularTolerance=0.25)
    print(f"  Exportado: {nome_arquivo} ({os.path.getsize(path) / 1024.0:.1f} KB)")
    return path
