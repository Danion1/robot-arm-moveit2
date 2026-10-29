"""
pick_and_place.py
=================
Pega o objeto num ponto da mesa e o deixa em outro, no Gazebo (pega = decisão C,
docs/decisions.md, 2026-10-14). Escrito com assistência de IA (Claude),
seguindo a sequência que o ros2/conferir_pega.py provou.

Quem faz o quê:
  - cinematica.solucoes() (IK analítica, este pacote): (x, y, z, garra para baixo) -> 4 juntas;
  - MoveIt2 (ação /move_action): planeja e executa até essas juntas, desviando de
    colisões (o objeto NÃO está na cena dele: por isso aproximar e recuar na vertical);
  - /garra_controller/joint_trajectory: abre e fecha a garra (direto, como no robô real);
  - /braco2/prender e /braco2/soltar: junta fixa do Gazebo (DetachableJoint) que
    segura o objeto no transporte, ligada SÓ depois de a pega ser conferida.

Conferência da pega (o que o robô real também pode medir, sem câmera): manda-se a
garra 1 mm além do contato esperado. Se ela PARAR ANTES do alvo, perto do contato, há
objeto entre os dedos; se CHEGAR ao alvo, fechou no vazio e o nó aborta SEM prender
(prender sem conferir grudaria o objeto de longe: aconteceu no teste, 2026-10-14).

Uso (com o pick_and_place.launch.py rodando):
    ros2 run braco2_nos pick_and_place
    ros2 run braco2_nos pick_and_place --ros-args -p ciclos:=3      (vai e volta, para o vídeo)
Sai com código 0 se tudo deu certo e 1 se alguma etapa falhou.
"""

import math
import sys

import rclpy
from builtin_interfaces.msg import Duration
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from braco2_nos import cinematica as cin

JUNTAS = list(cin.NOMES)
PRONTO = (0.0, 1.0472, -1.48353, -1.13446)   # pose nomeada "pronto" (0 / 60 / -85 / -65°)
GARRA_ABERTA = 0.020                          # m por dedo (limite do URDF)
MARGEM = 0.001        # caixa de colisão do dedo 1 mm maior (malhas_urdf.MARGEM): contato 1 mm antes
APERTO = 0.001        # alvo 1 mm além do contato: ~3 N (fechar até 0 aperta 28 N e espreme o objeto)
BLOQUEIO = 0.0003    # m: a garra tem de PARAR pelo menos isso antes do alvo (com o cubo para 0,7-1 mm
                     # antes; no vazio chega no alvo, ±0,05 mm). Medido em 2026-10-14.
FOLGA_ABERTA = 0.0005  # m: parar além do contato esperado = objeto maior ou garra travada
BAIXO = math.radians(-90)  # garra apontando para a mesa


class Falha(RuntimeError):
    """Uma etapa não deu certo: o nó para e sai com código 1."""


class PickAndPlace(Node):
    def __init__(self):
        # Relógio da SIMULAÇÃO (/clock): gravando vídeo, o Gazebo roda mais devagar que o
        # relógio de parede, e esperar 2 s de parede leria a garra antes de ela parar.
        super().__init__('pick_and_place', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        # 1. Parâmetros (metros). Padrões = mundo pick_and_place.sdf
        self.declare_parameter('objeto_x', 0.160)
        self.declare_parameter('objeto_y', 0.060)
        self.declare_parameter('destino_x', 0.160)
        self.declare_parameter('destino_y', -0.060)
        self.declare_parameter('largura_objeto', 0.020)
        self.declare_parameter('z_acima', 0.060)     # TCP na aproximação e no recuo
        self.declare_parameter('z_pegar', 0.020)     # TCP ao fechar (pontas dos dedos a ~5 mm da mesa)
        self.declare_parameter('z_soltar', 0.022)    # 2 mm acima: apoiado e preso, o objeto pula ao soltar
        self.declare_parameter('velocidade', 0.5)    # fração da velocidade máxima (juntas e aceleração)
        self.declare_parameter('ciclos', 1)          # 2+ = leva e traz de volta (vídeo)
        p = {n: self.get_parameter(n).get_parameter_value().double_value for n in (
            'objeto_x', 'objeto_y', 'destino_x', 'destino_y', 'largura_objeto',
            'z_acima', 'z_pegar', 'z_soltar', 'velocidade')}
        self.ciclos = self.get_parameter('ciclos').get_parameter_value().integer_value
        self.p = p
        self.contato = p['largura_objeto'] / 2 + MARGEM

        # 2. Comunicação
        self.mover = ActionClient(self, MoveGroup, 'move_action')
        self.pub_garra = self.create_publisher(JointTrajectory, '/garra_controller/joint_trajectory', 10)
        self.pub_prender = self.create_publisher(Empty, '/braco2/prender', 10)
        self.pub_soltar = self.create_publisher(Empty, '/braco2/soltar', 10)
        self.juntas = {}
        self.create_subscription(JointState, '/joint_states', self.ao_receber_estado, 10)

    # ------------------------------------------------------------------ apoio
    def ao_receber_estado(self, msg: JointState):
        # por nome: a ordem do /joint_states não é a do URDF
        self.juntas.update(dict(zip(msg.name, msg.position)))

    def esperar(self, segundos):
        """Espera `segundos` do tempo da simulação, atendendo as mensagens enquanto isso."""
        while self.get_clock().now().nanoseconds == 0:      # /clock ainda não chegou
            rclpy.spin_once(self, timeout_sec=0.02)
        fim = self.get_clock().now().nanoseconds + int(segundos * 1e9)
        while self.get_clock().now().nanoseconds < fim:
            rclpy.spin_once(self, timeout_sec=0.02)

    def publicar(self, pub, msg):
        while pub.get_subscription_count() == 0:     # só publica com alguém ouvindo
            self.esperar(0.1)
        pub.publish(msg)

    def juntas_para(self, x, y, z):
        solucoes = cin.solucoes(x, y, z, BAIXO)
        if not solucoes:
            raise Falha(f'ponto inalcançável com a garra para baixo: ({x:.3f}, {y:.3f}, {z:.3f}) m')
        return solucoes[0]            # cotovelo para cima

    # --------------------------------------------------------------- ações
    def braco(self, descricao, q):
        """Pede ao MoveIt2 para planejar e executar até as juntas q."""
        self.get_logger().info(f'{descricao}: {[round(math.degrees(a), 1) for a in q]} graus')
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = 'braco'
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = self.p['velocidade']
        req.max_acceleration_scaling_factor = self.p['velocidade']
        req.start_state.is_diff = True
        req.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=j, position=float(a), tolerance_above=0.001,
                            tolerance_below=0.001, weight=1.0) for j, a in zip(JUNTAS, q)])]
        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        envio = self.mover.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, envio, timeout_sec=30.0)
        pedido = envio.result()
        if pedido is None or not pedido.accepted:
            raise Falha(f'{descricao}: o MoveIt2 recusou o pedido')
        resultado = pedido.get_result_async()
        rclpy.spin_until_future_complete(self, resultado, timeout_sec=120.0)
        codigo = resultado.result().result.error_code.val
        if codigo != MoveItErrorCodes.SUCCESS:
            raise Falha(f'{descricao}: MoveIt2 devolveu o código {codigo}')
        self.esperar(0.3)

    def garra(self, valor, segundos=1, espera=None):
        """Manda a garra para `valor` (m por dedo) e devolve onde ela parou."""
        traj = JointTrajectory(joint_names=['junta_garra'])
        traj.points = [JointTrajectoryPoint(positions=[float(valor)],
                                            time_from_start=Duration(sec=segundos))]
        self.publicar(self.pub_garra, traj)
        self.esperar(segundos + 1.0 if espera is None else espera)
        return self.juntas['junta_garra']

    # ----------------------------------------------------------- sequência
    def pegar(self, x, y):
        self.braco('acima do objeto', self.juntas_para(x, y, self.p['z_acima']))
        self.braco('descer até o objeto', self.juntas_para(x, y, self.p['z_pegar']))

        alvo = self.contato - APERTO
        parou = self.garra(alvo, espera=2.5)
        self.get_logger().info(
            f'garra parou em {parou * 1000:.2f} mm por dedo (contato esperado {self.contato * 1000:.1f}, '
            f'alvo {alvo * 1000:.1f} mm)')
        if parou < alvo + BLOQUEIO or parou > self.contato + FOLGA_ABERTA:
            motivo = 'fechou no vazio' if parou < alvo + BLOQUEIO else 'parou cedo demais'
            self.garra(GARRA_ABERTA)
            self.braco('recuar sem o objeto', self.juntas_para(x, y, self.p['z_acima']))
            raise Falha(f'pega reprovada: a garra {motivo} ({parou * 1000:.2f} mm por dedo)')
        self.get_logger().info('pega conferida: objeto entre os dedos')

        # o aperto serviu para conferir; agora a garra fica onde parou e a junta fixa segura
        self.garra(parou, espera=1.0)
        self.publicar(self.pub_prender, Empty())
        self.esperar(0.5)
        self.braco('subir com o objeto', self.juntas_para(x, y, self.p['z_acima']))

    def soltar(self, x, y):
        self.braco('levar ao destino', self.juntas_para(x, y, self.p['z_acima']))
        self.braco('descer para soltar', self.juntas_para(x, y, self.p['z_soltar']))
        self.publicar(self.pub_soltar, Empty())
        self.esperar(0.5)
        self.garra(GARRA_ABERTA, segundos=2)      # abrir devagar: rápido, o dedo "chuta" o objeto
        self.braco('recuar na vertical', self.juntas_para(x, y, self.p['z_acima']))

    def executar(self):
        while not self.mover.wait_for_server(timeout_sec=2.0):
            self.get_logger().info('esperando o MoveIt2 (/move_action)...')
        while 'junta_garra' not in self.juntas:
            self.esperar(0.1)

        # o DetachableJoint começa PRESO: soltar antes de tudo
        self.publicar(self.pub_soltar, Empty())
        self.garra(GARRA_ABERTA)
        self.braco('pronto', PRONTO)

        a = (self.p['objeto_x'], self.p['objeto_y'])
        b = (self.p['destino_x'], self.p['destino_y'])
        for ciclo in range(self.ciclos):
            origem, destino = (a, b) if ciclo % 2 == 0 else (b, a)
            self.get_logger().info(f'--- ciclo {ciclo + 1}: de {origem} para {destino} ---')
            self.pegar(*origem)
            self.soltar(*destino)
        self.braco('pronto (fim)', PRONTO)
        self.get_logger().info('pick and place concluído')


def main():
    rclpy.init()
    no = PickAndPlace()
    codigo = 0
    try:
        no.executar()
    except Falha as erro:
        no.get_logger().error(str(erro))
        codigo = 1
    except (KeyboardInterrupt, ExternalShutdownException):
        codigo = 1
    finally:
        no.destroy_node()
        rclpy.try_shutdown()
    sys.exit(codigo)


if __name__ == '__main__':
    main()
