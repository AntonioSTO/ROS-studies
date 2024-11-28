import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import speech_recognition as sr
import pyttsx3
import time

class VoiceControlTurtlesim(Node):
    def __init__(self):
        super().__init__('voice_control_turtlesim')
        self.publisher_ = self.create_publisher(Int32, 'point', 10)
        self.engine = pyttsx3.init('espeak')
        self.set_engine_properties()
        self.get_logger().info("Nó de controle por voz inicializado e aguardando comandos.")

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
        self.get_logger().info(f"Falando: {text}")  # Log adicionado
        self.engine.say(text)
        self.engine.runAndWait()

    def listen_command(self):
        r = sr.Recognizer()
        with sr.Microphone() as source:
            self.get_logger().info('Escutando...')
            r.energy_threshold = 4000  # Ajuste de limiar de energia para escutar com mais precisão
            r.pause_threshold = 0.8  # Espera menos tempo para considerar uma pausa na fala
            r.adjust_for_ambient_noise(source, duration=1)  # Ajusta o reconhecimento de acordo com o ambiente

            try:
                # Aumenta o tempo de escuta para 5 segundos
                audio = r.listen(source, timeout=6, phrase_time_limit=6)
            except sr.WaitTimeoutError:
                self.get_logger().info('Tempo de escuta expirou. Tente novamente.')
                return None
            except sr.AudioException as e:
                self.get_logger().info(f'Erro de áudio: {e}')
                return None

        try:
            self.get_logger().info('Reconhecendo...')  # Log adicionado
            command = r.recognize_google(audio, language='pt-BR').lower()
            self.get_logger().info(f'Comando reconhecido: {command}')
            return command
        except sr.UnknownValueError:
            self.get_logger().info('Desculpe, não consegui entender. Tente novamente.')
            return None
        except sr.RequestError as e:
            self.get_logger().info(f'Erro ao se conectar ao serviço de reconhecimento: {e}')
            return None

    def run_voice_command_loop(self):
        while rclpy.ok():
            self.get_logger().info('Aguardando comando...')  # Log adicionado antes do speak
            command = self.listen_command()
            if command is None:
                continue

            msg = Int32()

            if 'porta' in command:
                msg.data = 0
                self.speak('Mandando o robô para a porta...')
            elif 'servidor' in command:
                msg.data = 1
                self.speak('Mandando o robô para o servidor...')
            elif 'origem' in command:
                msg.data = 2
                self.speak('Mandando o robô para a origem...')
            else:
                self.speak('Comando não reconhecido. Por favor, tente novamente.')
                continue

            self.publisher_.publish(msg)
            self.get_logger().info(f'Comando publicado: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    voice_control_node = VoiceControlTurtlesim()
    rclpy.spin(voice_control_node)
    voice_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()