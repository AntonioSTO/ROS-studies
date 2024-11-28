import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math
import speech_recognition as sr
import pyttsx3
import logging

from .listener import Speaker_identification


class VoiceControlNavigator(Node):
    def __init__(self):
        super().__init__("voice_control_navigator")
        self.action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        logging.basicConfig(
            level=logging.DEBUG,
            format="%(asctime)s - %(levelname)s - [%(filename)s:%(lineno)d] - %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S",
            force=True,
        )
        self.engine = pyttsx3.init("espeak")
        self.set_engine_properties()
        self.locations = [
            (0.7, 0.3, 0.0),
            (-1.0, -3.0, 0.0),
            (1.0, -1.0, 1.57),
            (0.0, 0.0, 0.0),
        ]
        self.sp = Speaker_identification(threshold=0.65)
        self.recognizer = sr.Recognizer()
        self.run_voice_command_loop()

    def set_engine_properties(self):
        self.engine.setProperty("rate", 210)
        self.engine.setProperty("volume", 1.0)
        voices = self.engine.getProperty("voices")
        for voice in voices:
            if "brazil" in voice.name.lower():
                self.engine.setProperty("voice", voice.id)
                break

    def speak(self, text):
        self.get_logger().info(f"Falando: {text}")
        self.engine.say(text)
        self.engine.runAndWait()

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(
            roll / 2
        ) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(
            roll / 2
        ) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(
            roll / 2
        ) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(
            roll / 2
        ) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return (qx, qy, qz, qw)

    def send_goal(self, location):
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
        self.action_client.wait_for_server()
        self.action_client.send_goal_async(goal_msg)

    def run_voice_command_loop(self):
        with sr.Microphone() as source:
            while True:
                logging.info("Defining operator. Speak...")
                self.sp.recognizer.adjust_for_ambient_noise(source, duration=1)
                audio_operator = self.recognizer.listen(source)
                try:
                    command = self.recognizer.recognize_google(
                        audio_operator,
                        language="pt-BR",
                    ).lower()
                    if "operador" in command:
                        embedding = self.sp.processed_audio.collect_embedding(
                            audio_operator.get_wav_data()
                        )
                        operator_id = self.sp.verify_id(embedding, define_operador=True)
                        logging.info(
                            f"Speak the command to the robot. ID = {operator_id}"
                        )
                        self.operator = True
                    else:
                        logging.info("Unidentified operator word, speak again")
                except sr.UnknownValueError:
                    pass

                if self.operator == True:
                    while True:
                        audio_command = self.recognizer.listen(source)
                        embedding = self.sp.processed_audio.collect_embedding(
                            audio_command.get_wav_data()
                        )
                        try:
                            command_text = self.recognizer.recognize_google(
                                audio_command,
                                language="pt-BR",
                            ).lower()
                            command_id = self.sp.verify_id(
                                embedding,
                                define_operador=False,
                            )
                            if command_id is not None:
                                if command_id == operator_id:
                                    if "teste" in command_text:
                                        location = self.locations[0]
                                        logging.info(f"Command sent by ID = {operator_id} to {location}")
                                        self.send_goal(location)

                                    elif "porta" in command_text:
                                        location = self.locations[1]
                                        logging.info(f"Command sent by ID = {operator_id} to {location}")
                                        self.send_goal(location)

                                    elif "servidor" in command_text:
                                        location = self.locations[2]
                                        logging.info(f"Command sent by ID = {operator_id} to {location}")
                                        self.send_goal(location)

                                    elif "origem" in command_text:
                                        location = self.locations[3]
                                        logging.info(f"Command sent by ID = {operator_id} to {location}")
                                        self.send_goal(location)
                                    else:
                                        logging.info(f"Command not recognized")
                                else:
                                    logging.info(f"Unidentified operator")
                            else:
                                logging.info("Unidentified operator")
                            if "deixar" in command_text:
                                logging.info("Leaving operator")
                                self.operator = False
                                break
                        except sr.UnknownValueError:
                            pass


def main(args=None):
    rclpy.init(args=args)
    navigator = VoiceControlNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()