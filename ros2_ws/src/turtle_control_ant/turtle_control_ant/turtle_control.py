import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, Pose2D
import math

class TurtleControlNode(Node):
    def __init__(self):
        super().__init__('turtle_control_node')

        # Inicializa variáveis
        self.init_variables()

        # Inicializa subscribers e publisher
        self.init_subscribers()
        self.init_publisher()

        # Define um timer para o callback do controle
        self.timer = self.create_timer(0.1, self.pub_callback)

    def init_variables(self):
        """Inicializa as variáveis usadas no controle."""
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.x_goal = 0.0
        self.y_goal = 0.0
        self.goal_received = False
        self.pose_received = False
        self.k_linear = 1.0  # Ganho para controle linear
        self.k_angular = 6.0  # Ganho para controle angular

    def init_subscribers(self):
        """Inicializa os subscribers."""
        # Subscreve ao tópico de pose da tartaruga
	self.create_subscription(
            Pose,
            'ant/turtle1/pose',
            self.pose_callback,
            10
        )

        # Subscreve ao tópico de objetivo (goal)
        self.create_subscription(
            Pose2D,
            'ant/goal',
            self.goal_callback,
            10
        )

    def init_publisher(self):
        """Inicializa o publisher."""
        # Publica comandos de velocidade para a tartaruga
        self.publisher_ = self.create_publisher(Twist, 'ant/turtle1/cmd_vel', 10)

    def pose_callback(self, msg):
        """Callback que atualiza a pose da tartaruga."""
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
        self.pose_received = True
        self.get_logger().info(f'Pose recebida: x={self.x}, y={self.y}, theta={self.theta}')

    def goal_callback(self, msg):
        """Callback que atualiza a posição de destino (goal)."""
        self.x_goal = msg.x
        self.y_goal = msg.y
        self.goal_received = True
        self.get_logger().info(f'Objetivo recebido: x_goal={self.x_goal},y_goal={self.y_goal}')

    def pub_callback(self):
        """Callback principal que publica a velocidade da tartaruga."""
        if not self.pose_received:
            self.get_logger().info('Aguardando pose inicial...')
            return
        if not self.goal_received:
            self.get_logger().info('Aguardando objetivo...')
            return

        # Calcular os erros
        x_error = self.x_goal - self.x
        y_error = self.y_goal - self.y
        distance = math.sqrt(x_error ** 2 + y_error ** 2)
        angle_to_goal = math.atan2(y_error, x_error)

        # Controle proporcional simples
        linear_velocity = self.k_linear * distance
        angular_velocity = self.k_angular * (angle_to_goal - self.theta)

        # Limitar a velocidade angular para evitar oscilações
        angular_velocity = max(min(angular_velocity, 2.0), -2.0)

        # Cria e publica a mensagem de velocidade
        cmd = Twist()
        cmd.linear.x = linear_velocity
        cmd.angular.z = angular_velocity
        self.publisher_.publish(cmd)

        self.get_logger().info(f'Comando publicado: linear={linear_velocity}, angular={angular_velocity}')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControlNode()

    # Mantém o nó rodando até ser interrompido
    rclpy.spin(node)
    # Fechar o nó ao término
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


