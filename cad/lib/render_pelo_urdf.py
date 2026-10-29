# -*- coding: utf-8 -*-
"""
render_pelo_urdf — imagem PNG do robô montado do jeito que o RViz2
montaria, sem abrir janela (VTK fora de tela).

Quando usar: conferência visual antes de considerar pronto. Lê o URDF e os
STL do pacote _description (não os sólidos do CAD): origem e eixo de cada
junta e o scale de cada malha saem do próprio URDF. Por isso pega malha
com translação embutida ou na unidade errada (os dois erros
clássicos de malha).

Uso típico:
    links, juntas = ler_urdf(urdf, {"meu_robo_description": pasta_do_pacote})
    mundo = transformadas(juntas, {"junta1": 0.0, ...})   # rad
    janela, r = vtk.vtkRenderWindow(), vtk.vtkRenderer() ...
    montar_cena(r, links, mundo, mesa=((-0.09, -0.16), (0.34, 0.16)), marcador="tcp")
    camera(r, foco=(0.12, 0, 0.10), posicao=(0.55, -0.75, 0.42))
    salvar(janela, "preview.png")
Saída de transformadas(): {link: matriz 4x4 no mundo, em m}. Também serve
como cinemática direta lida do URDF, independente da do projeto.

Validação: nenhuma automática; é inspeção visual. O projeto deve imprimir
alguma posição (ex.: TCP) tirada de transformadas() e comparar com a
própria cinemática.

Cuidados:
  - camera.SetViewUp(0, 0, 1) e posição explícita; o padrão do VTK olha de
    cima e achata tudo;
  - a mesa precisa de luz ambiente alta, senão sai escura;
  - rpy de origem aplicado como Rz·Ry·Rx (convenção do URDF);
  - todo link com <visual> precisa de <material> com <color> no URDF.

Origem: BR/render_preview.py, extraído em 2026-10-10. As matrizes vieram de
BR/cinematica.py; mesa, marcador e câmera, que eram fixos do braço,
viraram parâmetros.
"""

import math
import xml.etree.ElementTree as ET

import vtk


def mat_identidade():
    return [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]


def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def mat_translacao(x, y, z):
    m = mat_identidade()
    m[0][3], m[1][3], m[2][3] = x, y, z
    return m


def mat_rotacao(eixo, angulo):
    """Rotação em torno de um eixo unitário (fórmula de Rodrigues)."""
    ux, uy, uz = eixo
    c, s = math.cos(angulo), math.sin(angulo)
    v = 1.0 - c
    return [
        [c + ux * ux * v,      ux * uy * v - uz * s, ux * uz * v + uy * s, 0.0],
        [uy * ux * v + uz * s, c + uy * uy * v,      uy * uz * v - ux * s, 0.0],
        [uz * ux * v - uy * s, uz * uy * v + ux * s, c + uz * uz * v,      0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def ler_urdf(caminho, pacotes):
    """pacotes = {"nome_do_pacote": pasta}, para resolver package://."""
    raiz = ET.parse(caminho).getroot()
    cores = {m.get("name"): [float(v) for v in m.find("color").get("rgba").split()[:3]]
             for m in raiz.findall("material") if m.find("color") is not None}
    links = {}
    for link in raiz.findall("link"):
        visual = link.find("visual")
        if visual is None:
            links[link.get("name")] = None
            continue
        mesh = visual.find("geometry/mesh")
        arquivo = mesh.get("filename")
        for nome, pasta in pacotes.items():
            arquivo = arquivo.replace(f"package://{nome}", pasta)
        links[link.get("name")] = {
            "arquivo": arquivo,
            "escala": [float(v) for v in mesh.get("scale").split()],
            "cor": cores[visual.find("material").get("name")],
        }
    juntas = []
    for j in raiz.findall("joint"):
        juntas.append({
            "nome": j.get("name"), "tipo": j.get("type"),
            "pai": j.find("parent").get("link"), "filho": j.find("child").get("link"),
            "xyz": [float(v) for v in j.find("origin").get("xyz").split()],
            "rpy": [float(v) for v in j.find("origin").get("rpy").split()],
            "eixo": ([float(v) for v in j.find("axis").get("xyz").split()]
                     if j.find("axis") is not None else None),
        })
    return links, juntas


def transformadas(juntas, angulos):
    """{link: matriz 4x4 no mundo}, a partir da raiz (o link que não é filho de ninguém)."""
    filhos = {j["filho"] for j in juntas}
    raiz = next(j["pai"] for j in juntas if j["pai"] not in filhos)
    mundo = {raiz: mat_identidade()}
    pendentes = list(juntas)
    while pendentes:
        for j in pendentes:
            if j["pai"] in mundo:
                t = mat_mul(mundo[j["pai"]], mat_translacao(*j["xyz"]))
                for eixo, ang in (((0, 0, 1), j["rpy"][2]), ((0, 1, 0), j["rpy"][1]),
                                  ((1, 0, 0), j["rpy"][0])):
                    t = mat_mul(t, mat_rotacao(eixo, ang))
                if j["tipo"] == "revolute":
                    t = mat_mul(t, mat_rotacao(j["eixo"], angulos[j["nome"]]))
                mundo[j["filho"]] = t
                pendentes.remove(j)
                break
        else:
            raise RuntimeError("árvore do URDF desconexa")
    return mundo


def _ator(fonte, cor):
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(fonte.GetOutputPort())
    ator = vtk.vtkActor()
    ator.SetMapper(mapper)
    ator.GetProperty().SetColor(*cor)
    return ator


def montar_cena(renderer, links, mundo, mesa=None, marcador=None):
    """mesa = ((x0, y0), (x1, y1)) em m no plano z = 0; marcador = nome de um
    frame de `mundo` que ganha uma bolinha vermelha (ex.: "tcp")."""
    for nome, info in links.items():
        if info is None:
            continue
        reader = vtk.vtkSTLReader()
        reader.SetFileName(info["arquivo"])
        reader.Update()
        matriz = vtk.vtkMatrix4x4()
        for i in range(4):
            for k in range(4):
                matriz.SetElement(i, k, mundo[nome][i][k])
        tf = vtk.vtkTransform()
        tf.SetMatrix(matriz)
        tf.Scale(*info["escala"])          # aplicado antes da pose: mm -> m
        ator = _ator(reader, info["cor"])
        ator.SetUserTransform(tf)
        ator.GetProperty().SetAmbient(0.25)
        ator.GetProperty().SetSpecular(0.15)
        renderer.AddActor(ator)

    if mesa is not None:
        (x0, y0), (x1, y1) = mesa
        plano = vtk.vtkPlaneSource()
        plano.SetOrigin(x0, y0, 0.0)
        plano.SetPoint1(x1, y0, 0.0)
        plano.SetPoint2(x0, y1, 0.0)
        ator = _ator(plano, (0.86, 0.84, 0.80))
        ator.GetProperty().SetAmbient(0.75)    # senão sai escura com a luz rasante
        ator.GetProperty().SetDiffuse(0.25)
        renderer.AddActor(ator)

    if marcador is not None:
        m = mundo[marcador]
        bola = vtk.vtkSphereSource()
        bola.SetCenter(m[0][3], m[1][3], m[2][3])
        bola.SetRadius(0.004)
        bola.SetThetaResolution(24)
        bola.SetPhiResolution(24)
        renderer.AddActor(_ator(bola, (0.9, 0.1, 0.1)))


def camera(renderer, foco, posicao, paralela=False, zoom=1.45):
    cam = renderer.GetActiveCamera()
    cam.SetViewUp(0, 0, 1)
    cam.SetFocalPoint(*foco)
    cam.SetPosition(*posicao)
    if paralela:
        cam.ParallelProjectionOn()
    renderer.ResetCamera()
    cam.Zoom(zoom)
    renderer.ResetCameraClippingRange()


def salvar(janela, caminho):
    janela.Render()
    filtro = vtk.vtkWindowToImageFilter()
    filtro.SetInput(janela)
    filtro.ReadFrontBufferOff()
    filtro.Update()
    writer = vtk.vtkPNGWriter()
    writer.SetFileName(caminho)
    writer.SetInputConnection(filtro.GetOutputPort())
    writer.Write()
    print("Preview salvo em:", caminho)
