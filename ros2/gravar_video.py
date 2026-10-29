#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gravar_video.py
===============
Grava o /camera_video (câmera simulada do pick_and_place.launch.py video:=true)
num MP4 (H.264, toca no navegador e no GitHub) a 30 quadros por segundo de SIMULAÇÃO. Cada quadro da câmera é um passo de
1/30 s do tempo simulado, então o vídeo sai liso mesmo que o Gazebo rode mais devagar
que o relógio.

Ao terminar (Ctrl+C ou --segundos), confere pelo carimbo de tempo de cada quadro se
algum faltou (buraco > 1,5/30 s) ou veio fora de ordem: é a prova de que está liso.

Uso:
    python3 gravar_video.py saida.mp4 [--segundos N]
"""

import argparse
import signal
import sys

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

FPS = 30


class Gravador(Node):
    def __init__(self, saida):
        super().__init__('gravar_video')
        self.saida = saida
        self.ponte = CvBridge()
        self.video = None
        self.carimbos = []
        qos = QoSProfile(depth=500, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Image, '/camera_video', self.quadro, qos)

    def quadro(self, msg):
        img = self.ponte.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.video is None:
            altura, largura = img.shape[:2]
            self.video = cv2.VideoWriter(self.saida, cv2.VideoWriter_fourcc(*'avc1'), FPS, (largura, altura))
            self.get_logger().info(f'gravando {largura}x{altura} em {self.saida}')
        self.video.write(img)
        self.carimbos.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def fechar(self):
        if self.video is not None:
            self.video.release()
        c = self.carimbos
        if len(c) < 2:
            print('nenhum quadro gravado')
            return False
        passos = [b - a for a, b in zip(c, c[1:])]
        buracos = [p for p in passos if p > 1.5 / FPS]
        fora = [p for p in passos if p <= 0]
        print(f'{len(c)} quadros = {len(c) / FPS:.1f} s de vídeo ({c[-1] - c[0]:.1f} s simulados); '
              f'buracos: {len(buracos)}, fora de ordem: {len(fora)}')
        return not buracos and not fora


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('saida')
    ap.add_argument('--segundos', type=float, default=0.0, help='para sozinho depois de N s simulados')
    args = ap.parse_args()
    rclpy.init()
    no = Gravador(args.saida)
    parar = []
    signal.signal(signal.SIGINT, lambda *_: parar.append(1))
    signal.signal(signal.SIGTERM, lambda *_: parar.append(1))
    while not parar and rclpy.ok():
        rclpy.spin_once(no, timeout_sec=0.1)
        if args.segundos and no.carimbos and no.carimbos[-1] - no.carimbos[0] >= args.segundos:
            break
    liso = no.fechar()
    no.destroy_node()
    rclpy.try_shutdown()
    sys.exit(0 if liso else 1)


if __name__ == '__main__':
    main()
