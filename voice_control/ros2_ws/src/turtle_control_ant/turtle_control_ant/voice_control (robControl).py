import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math
import speech_recognition as sr
import pyttsx3

class VoiceControlNavigator(Node):
    def __init__(self):
        super().__init__('voice_control_navigator')
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # inicializa engine de texto para fala
        self.engine = pyttsx3.init('espeak')
        self.set_engine_properties()
        self.get_logger().info("Nó de controle por voz inicializado e aguardando comandos.")

        # dicionário de locais com coordenadas (x, y, yaw)
        self.locations = [(0.7, 0.3, 0.0), (1.2, 2.0, 0.0), (-2.0, 1.0, 1.57), (0.0, 0.0, 0.0)] #teste, porta, servidor, origem

        self.run_voice_command_loop()

    def set_engine_properties(self):
        self.engine.setProperty('rate', 210)
        self.engine.setProperty('volume', 1.0)
        voices = self.engine.getProperty('voices')
        for voice in voices:
            if "brazil" in voice.name.lower():
                self.engine.setProperty('voice', voice.id)
                break

    def speak(self, text):
        self.get_logger().info(f"Falando: {text}")
        self.engine.say(text)
        self.engine.runAndWait()

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Converte ângulos de Euler (roll, pitch, yaw) para um quaternion (x, y, z, w).
        """
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return (qx, qy, qz, qw)

    def send_goal(self, location):
        """
        Envia uma meta de navegação ao action server `/navigate_to_pose`.
        """
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = location[0]
        goal_msg.pose.pose.position.y = location[1]
        goal_msg.pose.pose.position.z = 0.0

        qx, qy, qz, qw = self.euler_to_quaternion(0.0, 0.0, location[2])
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        # Envia o goal para o action server
        self.action_client.wait_for_server()
        self.get_logger().info(f"Enviando meta para posição: {location[:2]}, yaw: {location[2]}")
        self.action_client.send_goal_async(goal_msg)

    def listen_command(self):
        r = sr.Recognizer()
        with sr.Microphone() as source:
            self.get_logger().info('Escutando...')
            r.adjust_for_ambient_noise(source, duration=1)
            try:
                audio = r.listen(source, timeout=6, phrase_time_limit=6)
                command = r.recognize_google(audio, language='pt-BR').lower()
                self.get_logger().info(f'Comando reconhecido: {command}')
                return command
            except sr.UnknownValueError:
                self.speak('Desculpe, não consegui entender. Tente novamente.')
            except sr.RequestError as e:
                self.get_logger().info(f'Erro ao se conectar ao serviço de reconhecimento: {e}')
            except sr.WaitTimeoutError:
                self.get_logger().info('Tempo de escuta expirou. Tente novamente.')
        return None

    def run_voice_command_loop(self):
        while rclpy.ok():
            self.get_logger().info('Aguardando comando...')
            command = self.listen_command()
            if command is None:
                continue


            if 'teste' in command:
                location = self.locations[0]
                self.speak(f"Mandando o robô para teste...")
                self.send_goal(location)

            elif 'porta' in command:
                location = self.locations[1]
                self.speak(f"Mandando o robô para a porta...")
                self.send_goal(location)

            elif 'servidor' in command:
                location = self.locations[2]
                self.speak(f"Mandando o robô para o servidor...")
                self.send_goal(location)

            elif 'origem' in command:
                location = self.locations[3]
                self.speak(f"Mandando o robô para a origem...")
                self.send_goal(location)
            else:
                self.speak('Comando não reconhecido. Por favor, tente novamente.')

def main(args=None):
    rclpy.init(args=args)
    navigator = VoiceControlNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
