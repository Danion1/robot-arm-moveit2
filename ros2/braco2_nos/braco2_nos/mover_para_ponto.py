import math

from braco2_nos import cinematica as cin
from builtin_interfaces.msg import Duration
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JUNTAS = ['junta_base', 'junta_ombro', 'junta_cotovelo', 'junta_punho']


class MoverParaPonto(Node):
    def __init__(self):
        super().__init__('mover_para_ponto')

        # 1. Declarar e ler os parametros
        self.declare_parameter('x', 0.15)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.05)
        self.declare_parameter('phi_graus', -90.0)

        x = self.get_parameter('x').value
        y = self.get_parameter('y').value
        z = self.get_parameter('z').value
        phi_graus = self.get_parameter('phi_graus').value
        phi_rad = math.radians(phi_graus)

        # 2. Calcular cinematica inversa
        solucoes = cin.solucoes(x, y, z, phi_rad)

        if not solucoes:
            self.get_logger().error(
                f'Ponto inalcançável: x={x}, y={y}, z={z}, phi={phi_graus}°'
            )
            self.alvo_q = None
        else:
            # Escolha a primeira solução(cotovelo para cima)
            self.alvo_q = solucoes[0]
            q_graus = [math.degrees(q) for q in self.alvo_q]
            self.get_logger().info(
                f'Alvo de juntas calculados (graus): '
                f'Base={q_graus[0]:.2f}°, Ombro={q_graus[1]:.2f}°, '
                f'Cotovelo={q_graus[2]:.2f}°, Punho={q_graus[3]:.2f}°'
            )

        # 3. Publisher, subscriber e Timer com os controllers do yaml
        self.pub = self.create_publisher(
            JointTrajectory, '/braco_controller/joint_trajectory', 10
        )

        self.sub = self.create_subscription(
            JointState, '/joint_states', self.ao_receber_estado, 10
        )

        self.timer = self.create_timer(0.5, self.tentar_publicar)

        # Controle do estado e dos logs

        self.publicado = False
        self.ultimo_log_tempo = self.get_clock().now()

    def tentar_publicar(self):
        # Se ja foi publicado ou se o ponto é invalido ou se o controlador ainda não esta ouvindo
        if (
            self.publicado
            or self.alvo_q is None
            or self.pub.get_subscription_count() == 0
        ):
            return

        # Montar mensagem JointTrajectory
        traj = JointTrajectory()
        traj.joint_names = JUNTAS

        ponto = JointTrajectoryPoint()
        ponto.positions = [float(q) for q in self.alvo_q]
        ponto.time_from_start = Duration(sec=3, nanosec=0)

        traj.points.append(ponto)

        # Publicar comando

        self.pub.publish(traj)
        self.publicado = True

        q_graus = [math.degrees(q) for q in self.alvo_q]
        self.get_logger().info(
            f' Trajetória enviada para o controlador! '
            f'Ângulos (deg): {[round(deg, 2) for deg in q_graus]}'
        )

    def ao_receber_estado(self, msg: JointState):
        # Só exibe a telemetria de acompanhemento se a mensagem ja tiver sido enviada
        if not self.publicado or self.alvo_q is None:
            return

        # Mapeamento dinamico para nao depender da ordem da lista(pegadinha)
        posicoes = dict(zip(msg.name, msg.position))
        esforcos = dict(zip(msg.name, msg.effort)) if msg.effort else {}

        # Limitar o log para 1 vez por segundo (Throttle de 1,0s)
        agora = self.get_clock().now()
        if (agora - self.ultimo_log_tempo).nanoseconds < 1e9:
            return
        self.ultimo_log_tempo = agora

        # Comparar posição atual com o alvo em graus
        linhas_log = []
        for i, nome_junta in enumerate(JUNTAS):
            if nome_junta in posicoes:
                atual_deg = math.degrees(posicoes[nome_junta])
                alvo_deg = math.degrees(self.alvo_q[i])
                linhas_log.append(f'{nome_junta}: {atual_deg:.2f}° / Alvo: {alvo_deg:.2f}°')

        esforco_ombro = esforcos.get('junta_ombro', 0.0)

        self.get_logger().info(
            '\n--- Telemetria /joint_states ---\n'
            + '\n'.join(linhas_log)
            + f'\nEsforço no Ombro: {esforco_ombro:.4f} N·m\n'
            + '-' * 32
        )


def main():
    rclpy.init()
    no = MoverParaPonto()
    try:
        rclpy.spin(no)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        no.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
