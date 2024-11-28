import speech_recognition as sr
import logging
from .audiorecognizer import Extract_embeddings
from scipy.spatial.distance import cosine
import numpy as np


class Speaker_identification:
    def __init__(self, threshold=0.65):
        logging.basicConfig(
            level=logging.DEBUG,
            format="%(asctime)s - %(levelname)s - [%(filename)s:%(lineno)d] - %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S",
            force=True,
        )
        self.data_audio = {}
        self.id = 0
        self.operator = False
        self.threshold = threshold
        self.recognizer = sr.Recognizer()
        self.processed_audio = Extract_embeddings()

    def verify_id(self, embedding, define_operador=False):
        embedding = embedding[0]
        max_distance = float("inf")
        if not self.data_audio:
            self.data_audio[self.id] = embedding
            return self.id
        for class_id, class_embedding in self.data_audio.items():
            distance = cosine(embedding, class_embedding)
            if distance < max_distance:
                max_distance = distance
                current_id = class_id
            if max_distance < self.threshold:
                return current_id
        if define_operador:
            self.id += 1
            self.data_audio[self.id] = embedding
            logging.info(f"New id assigned: {self.id}")
            return self.id
        return None

    def listen_command(self):
        with sr.Microphone() as source:
            while True:
                logging.info("Defining operator. Speak...")
                self.recognizer.adjust_for_ambient_noise(source, duration=1)
                audio_operator = self.recognizer.listen(source)
                try:
                    command = self.recognizer.recognize_google(
                        audio_operator,
                        language="pt-BR",
                    ).lower()
                    if "operador" in command:
                        embedding = self.processed_audio.collect_embedding(
                            audio_operator.get_wav_data()
                        )
                        operator_id = self.verify_id(embedding, define_operador=True)
                        logging.info(
                            f"Speak the command to the robot. ID = {operator_id}"
                        )
                        self.operator = True
                    else:
                        logging.info("Unidentified operator word, speak again")
                except sr.UnknownValueError:
                    logging.info("Unable to understand audio.")
                if self.operator == True:
                    while True:
                        audio_command = self.recognizer.listen(source)
                        embedding = self.processed_audio.collect_embedding(
                            audio_command.get_wav_data()
                        )
                        try:
                            command_text = self.recognizer.recognize_google(
                                audio_command,
                                language="pt-BR",
                            ).lower()
                            command_id = self.verify_id(
                                embedding,
                                define_operador=False,
                            )
                            if command_id is not None:
                                if command_id == operator_id:
                                    logging.info(f"Command sent by ID = {operator_id}")
                                else:
                                    logging.info(f"Unidentified operator")
                            else:
                                logging.info("Repeat command")
                            if "deixar" in command_text:
                                logging.info("Leaving operator")
                                self.operator = False
                                break
                        except sr.UnknownValueError:
                            logging.info("Unable to understand audio.")
                        

if __name__ == "__main__":
    sp = Speaker_identification(threshold=0.65)
    sp.listen_command()