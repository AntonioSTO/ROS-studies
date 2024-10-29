import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import cv2
import mediapipe as mp

class FingerCountPublisher(Node):
    def __init__(self):
        super().__init__('finger_count_publisher')
        self.publisher_ = self.create_publisher(Int32, 'finger_count', 10)
        self.timer = self.create_timer(0.1, self.publish_finger_count)
        self.video = cv2.VideoCapture(0)
        self.hand = mp.solutions.hands
        self.Hand = self.hand.Hands(max_num_hands=1)
        self.mpDraw = mp.solutions.drawing_utils

    def publish_finger_count(self):
        check, img = self.video.read()
        if not check:
            return

        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        results = self.Hand.process(imgRGB)
        handPoints = results.multi_hand_landmarks
        h, w, d = img.shape
        pontos = []
        contador = 0

        if handPoints:
            for points in handPoints:
                self.mpDraw.draw_landmarks(img, points, self.hand.HAND_CONNECTIONS)
                for id, cord in enumerate(points.landmark):
                    cx, cy = int(cord.x * w), int(cord.y * h)
                    pontos.append((cx, cy))
                
            dedos = [8, 12, 16, 20]

            if points:
                if pontos[4][0] < pontos[2][0]:
                    contador += 1
                for x in dedos:
                    if pontos[x][1] < pontos[x - 2][1]:
                        contador += 1

        # Publica a contagem de dedos
        msg = Int32()
        msg.data = contador
        self.publisher_.publish(msg)

        # Exibe a contagem de dedos na tela
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(img, str(contador), (100, 100), font, 4, (0, 0, 0), 5)
        cv2.imshow("hands", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.video.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = FingerCountPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
