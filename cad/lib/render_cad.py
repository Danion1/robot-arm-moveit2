# -*- coding: utf-8 -*-
"""
render_cad.py
=============
PNG de sólidos do CadQuery, sem abrir janela (VTK fora de tela), para
VER a peça a cada mudança sem abrir um CAD com interface.

Diferente do render_pelo_urdf da biblioteca: aqui a entrada são os
SÓLIDOS do CAD (antes de existir URDF), não as malhas montadas pelo URDF.
Câmera e gravação reaproveitam as funções de lá.

Uso:
    render([(solido, (r, g, b)), ...], "previews/junta.png",
           foco=(0, 0, 0), posicao=(120, -160, 110))
Unidade: a do sólido (mm). Visual apenas: nada aqui verifica geometria.

Origem: Projeto 2; movido para a biblioteca em 2026-10-14 no 2º uso real
(experimento Eternatus). Prova: os PNG de previews/ do Projeto 2 saíram
idênticos byte a byte antes e depois (git sem diferença).
"""

import vtk
from render_pelo_urdf import camera, salvar


def _polydata(solido, tolerancia=0.05):
    """Triangula o sólido (BRep) e devolve vtkPolyData com normais."""
    vertices, triangulos = solido.tessellate(tolerancia, 0.2)
    pontos = vtk.vtkPoints()
    for v in vertices:
        pontos.InsertNextPoint(v.x, v.y, v.z)
    celulas = vtk.vtkCellArray()
    for a, b, c in triangulos:
        t = vtk.vtkTriangle()
        t.GetPointIds().SetId(0, a)
        t.GetPointIds().SetId(1, b)
        t.GetPointIds().SetId(2, c)
        celulas.InsertNextCell(t)
    pd = vtk.vtkPolyData()
    pd.SetPoints(pontos)
    pd.SetPolys(celulas)
    normais = vtk.vtkPolyDataNormals()
    normais.SetInputData(pd)
    normais.SetFeatureAngle(35.0)
    normais.Update()
    return normais


def render(itens, caminho, foco, posicao, tamanho=(1400, 1000), zoom=1.3, opacidade=None):
    """itens = [(cq.Shape ou cq.Workplane, (r, g, b)), ...]; opacidade =
    {índice: alfa} para deixar, por exemplo, o servo translúcido."""
    janela = vtk.vtkRenderWindow()
    janela.SetOffScreenRendering(1)
    janela.SetSize(*tamanho)
    r = vtk.vtkRenderer()
    r.SetBackground(1.0, 1.0, 1.0)
    janela.AddRenderer(r)
    filtros = []   # o VTK não segura o filtro: sem esta lista o Python o coleta e dá segfault
    for i, (solido, cor) in enumerate(itens):
        val = solido.val() if hasattr(solido, "val") else solido
        filtros.append(_polydata(val))
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(filtros[-1].GetOutputPort())
        ator = vtk.vtkActor()
        ator.SetMapper(mapper)
        p = ator.GetProperty()
        p.SetColor(*cor)
        p.SetAmbient(0.3)
        p.SetSpecular(0.15)
        if opacidade and i in opacidade:
            p.SetOpacity(opacidade[i])
        r.AddActor(ator)
    camera(r, foco=foco, posicao=posicao, zoom=zoom)
    salvar(janela, caminho)
    print(f"  Imagem: {caminho}")
