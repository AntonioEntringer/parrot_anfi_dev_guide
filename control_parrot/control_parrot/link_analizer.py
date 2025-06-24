'''import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16, Int8, UInt8
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy
import time
import csv
import numpy as np
from cv_bridge import CvBridge
import cv2


class LinkSubscriber(Node):

    def __init__(self):
        super().__init__('link_subscriber')

        self.br = CvBridge()
        self.count = 1

        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.rssi_value = None
        self.quality_value = None
        self.goodput_value = None
        dist = 0
        self.frame = None

        self.elapsed_time_old = 0
        self.total_bytes = 0
        self.message_count = 0
        self.start_time = time.time()
        
        # Subscrições para os três tópicos
        self.sub_rssi = self.create_subscription(Int8, '/anafi/link/rssi', self.rssi_callback, 10)
        self.sub_quality = self.create_subscription(UInt8, '/anafi/link/quality', self.quality_callback, qos_profile)
        self.sub_goodput = self.create_subscription(UInt16, '/anafi/link/goodput', self.goodput_callback, 10)
        self.sub_camera_img = self.create_subscription(Image, "/anafi/camera/image", self.image_callback, qos_profile)
        self.timer = self.create_timer(1.0, self.print_bandwidth)  # 1.0 segundo = 1 Hz
        self.subscription = self.create_subscription(Float32MultiArray, '/anafi/drone/position_global', self.position_callback, qos_profile)

        # Inicializa o arquivo CSV
        self.csv_file = open('metrics.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Timestamp', 'Largura de Banda (MB/s)', 'Tamanho Médio das Mensagens (MB)', 'Hz', 'RSSI', 'Quality', 'Goodput (MB/s)'])

    def rssi_callback(self, msg):
        self.rssi_value = msg.data

    def quality_callback(self, msg):
        self.quality_value = msg.data

    def goodput_callback(self, msg):
        self.goodput_value = msg.data

    def image_callback(self, msg):
        self.total_bytes += len(msg.data)
        self.message_count += 1
        try:
            self.frame = self.br.imgmsg_to_cv2(msg)  # Usar `msg` diretamente
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
            self.frame = None

    def position_callback(self, msg):
        self.current_position = msg.data
        x = self.current_position[0]
        y = self.current_position[1]
        self.dist = np.sqrt((x*x) + (y*y))

    def print_bandwidth(self):
        elapsed_time = time.time() - self.start_time
        if elapsed_time > 1 and self.message_count != 0:
            bandwidth = (self.total_bytes / 1e6) / (elapsed_time - self.elapsed_time_old)  # MB/s
            message_size_mean = (self.total_bytes / 1e6) / self.message_count  # MB
            hz = self.message_count
            goodput_mb_s = self.goodput_value * 8 / 1024  # MB/s

            print(f'Largura de Banda: {bandwidth:.2f} MB/s, Tamanho Médio das Mensagens: {message_size_mean:.2f} MB, Hz: {hz}')
            print(f'RSSI: {self.rssi_value}, Quality: {self.quality_value}, Goodput: {goodput_mb_s:.2f} MB/s')
            print(f'Distancia: {self.dist}')
            # Salva as métricas no arquivo CSV
            self.save_to_csv(elapsed_time, bandwidth, message_size_mean, hz, self.rssi_value, self.quality_value, goodput_mb_s, self.dist)

            # Format the text to be overlayed on the image
            text_lines = [
                'Distancia: {:.2f} M'.format(self.dist),
                'Quality: {} (0 - 5)'.format(self.quality_value),
                'RSSI: {} Db'.format(self.rssi_value),
                'Goodput: {:.2f} MB/s'.format(goodput_mb_s),
                'Banda topico camera: {:.2f} MB/s'.format(bandwidth),
                'Tamanho Medio das Mensagens camera: {:.2f} MB'.format(message_size_mean),
                'Frequencia: {} Hz'.format(hz)
                
                
            ]

            if self.frame is not None:
                # Adicionar sobreposição de texto na imagem
                font = cv2.FONT_HERSHEY_SIMPLEX
                color = (155, 0, 155)
                y0, dy = 30, 30  # Posição inicial e distância entre linhas

                for i, line in enumerate(text_lines):
                    y = y0 + i * dy
                    cv2.putText(self.frame, line, (10, y), font, 0.6, color, 2, cv2.LINE_AA)

                cv2.imshow("camera", self.frame)
                cv2.waitKey(1)
            else:
                self.get_logger().info('No image frame available.')

            cv2.imshow("camera", self.frame)
            cv2.waitKey(1)

        self.total_bytes = 0
        self.message_count = 0
        self.elapsed_time_old = elapsed_time

    def save_to_csv(self, timestamp, bandwidth, message_size_mean, hz, rssi, quality, goodput, dist):
        self.csv_writer.writerow([timestamp, bandwidth, message_size_mean, hz, rssi, quality, goodput, dist])
        self.csv_file.flush()

    def __del__(self):
        # Fecha o arquivo CSV quando o nó é destruído
        self.csv_file.close()

def main(args=None):
    rclpy.init(args=args)
    link_subscriber = LinkSubscriber()
    rclpy.spin(link_subscriber)
    link_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''


'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import pandas as pd

class Registro(Node):
    def __init__(self):
        super().__init__('registro')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10)

        self.subscription_pos = self.create_subscription(Float32MultiArray, '/anafi/drone/position_global', self.position_callback, qos_profile)
        self.subscription_traj = self.create_subscription(Float32MultiArray, '/anafi/drone/xyzw_trajetoria', self.trajectory_callback, qos_profile)
        
        self.drone_pos = None
        self.traj_pos = None
        self.drone_path = []
        self.traj_path = []
        
        self.fig, self.ax = plt.subplots(figsize=(6.4, 4.8))
        self.ax.set_title("Drone Position vs. Desired Position")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        plt.ion()
        plt.show()
    
    def position_callback(self, msg):
        self.drone_pos = (msg.data[0], msg.data[1])
        self.drone_path.append(self.drone_pos)
        self.update_plot()
    
    def trajectory_callback(self, msg):
        self.traj_pos = (msg.data[0], msg.data[1])
        self.traj_path.append(self.traj_pos)
        self.update_plot()
    
    def update_plot(self):
        if self.drone_pos is None or self.traj_pos is None:
            return
        
        self.ax.clear()
        self.ax.set_title("Drone Position vs. Desired Position")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        
        r = 5
        
        self.ax.set_xlim(-r, r)
        self.ax.set_ylim(-r, r)
        
        self.ax.plot(*zip(*self.drone_path), linestyle='dashed', color='blue', alpha=0.5, label='Drone Path')
        self.ax.plot(*zip(*self.traj_path), linestyle='dashed', color='red', alpha=0.5, label='Target Path')
        
        self.ax.scatter(*self.drone_pos, color='blue', label='Drone')
        self.ax.scatter(*self.traj_pos, color='red', label='Target')
        self.ax.legend()
        
        plt.draw()
        plt.pause(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = Registro()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

'''

'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8, UInt8, UInt16
import matplotlib.pyplot as plt
import csv

class Registro(Node):
    def __init__(self):
        super().__init__('registro')

        # QoS padrão para todos
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # subscrições de posição e trajetória (mantidas)
        self.subscription_pos = self.create_subscription(
            Float32MultiArray,
            '/anafi/drone/position_global',
            self.position_callback,
            qos_profile
        )
        self.subscription_traj = self.create_subscription(
            Float32MultiArray,
            '/anafi/drone/xyzw_trajetoria',
            self.trajectory_callback,
            qos_profile
        )
        self.subscrition_gps = self.create_subscription(
            Float32MultiArray,
            '/anafi/drone/position_global_gps_only',
            self.gps_callback,
            qos_profile
        )
        # subscrições de métricas adicionais
        self.sub_rssi     = self.create_subscription(Int8,    '/anafi/link/rssi',    self.rssi_callback,    qos_profile)
        self.sub_quality  = self.create_subscription(UInt8,   '/anafi/link/quality', self.quality_callback, qos_profile)
        self.sub_goodput  = self.create_subscription(UInt16,  '/anafi/link/goodput', self.goodput_callback, qos_profile)

        # estados atuais
        self.drone_pos    = (None, None)
        self.traj_pos     = (None, None)
        self.drone_pos_gps = (None, None)
        self.rssi_value   = None
        self.quality_value= None
        self.goodput_value= None

        # caminhos para plot
        self.drone_path = []
        self.traj_path  = []

        # tempo inicial
        self.frist_t = self.get_clock().now().nanoseconds * 1e-9


        # inicializa CSV com novas colunas
        self.csv_file = open('modelagem180hz.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'time',
            'drone_x','drone_y',
            'target_x','target_y',
            'gps_x','gps_y',
            'rssi','quality','goodput'
        ])
        self.csv_file.flush()

        # setup do plot (sem alterações)
        self.fig, self.ax = plt.subplots(figsize=(6.4, 4.8))
        self.ax.set_title("Drone Position vs. Desired Position")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        plt.ion()
        plt.show()

    def position_callback(self, msg):
        x, y = msg.data[0], msg.data[1]
        self.drone_pos = (x, y)
        self.drone_path.append(self.drone_pos)
        self.log_data()
        self.update_plot()

    def gps_callback(self, msg):
        gpsx, gpsy = msg.data[0], msg.data[1]
        self.drone_pos_gps = (gpsx, gpsy)
        self.drone_path.append(self.drone_pos_gps)
        self.log_data()

    def trajectory_callback(self, msg):
        xt, yt = msg.data[0], msg.data[1]
        self.traj_pos = (xt, yt)
        self.traj_path.append(self.traj_pos)
        self.log_data()
        self.update_plot()



    def rssi_callback(self, msg):
        self.rssi_value = msg.data
        self.log_data()

    def quality_callback(self, msg):
        self.quality_value = msg.data
        self.log_data()

    def goodput_callback(self, msg):
        self.goodput_value = msg.data
        self.log_data()

    def log_data(self):
        # timestamp ROS em segundos
        
        t = self.get_clock().now().nanoseconds * 1e-9 - self.frist_t
        dx, dy = self.drone_pos
        tx, ty = self.traj_pos
        gpsx, gpsy = self.drone_pos_gps
        # escreve valores, usa '' quando None
        row = [
            f"{t:.6f}",
            "" if dx is None else f"{dx:.4f}",
            "" if dy is None else f"{dy:.4f}",
            "" if tx is None else f"{tx:.4f}",
            "" if ty is None else f"{ty:.4f}",
            "" if gpsx is None else f"{gpsx:.4f}",
            "" if gpsy is None else f"{gpsy:.4f}",
            "" if self.rssi_value   is None else str(self.rssi_value),
            "" if self.quality_value is None else str(self.quality_value),
            "" if self.goodput_value is None else str(self.goodput_value)
        ]
        self.csv_writer.writerow(row)
        self.csv_file.flush()

    def update_plot(self):
        if None in self.drone_pos or None in self.traj_pos:
            return

        self.ax.clear()
        self.ax.set_title("Drone Position vs. Desired Position")
        self.ax.set_xlabel("X"); self.ax.set_ylabel("Y")
        r=5
        self.ax.set_xlim(-r,r); self.ax.set_ylim(-r,r)

        self.ax.plot(*zip(*self.drone_path),  linestyle='dashed', alpha=0.5, label='Drone Path')
        self.ax.plot(*zip(*self.traj_path),   linestyle='dashed', alpha=0.5, label='Target Path')
        self.ax.scatter(*self.drone_pos, label='Drone')
        self.ax.scatter(*self.traj_pos,  label='Target')
        self.ax.legend()
        plt.draw(); plt.pause(0.1)

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Registro()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8, UInt8, UInt16
from geometry_msgs.msg import Vector3Stamped
import matplotlib.pyplot as plt
import csv

class Registro(Node):
    def __init__(self):
        super().__init__('registro')

        # QoS profile
        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriptions
        self.sub_pos = self.create_subscription(
            Float32MultiArray, '/anafi/drone/position_global', self.position_callback, qos)
        self.sub_traj = self.create_subscription(
            Float32MultiArray, '/anafi/drone/xyzw_trajetoria', self.trajectory_callback, qos)
        self.sub_gps = self.create_subscription(
            Float32MultiArray, '/anafi/drone/position_global_gps_only', self.gps_callback, qos)
        self.sub_rssi = self.create_subscription(
            Int8, '/anafi/link/rssi', self.rssi_callback, qos)
        self.sub_quality = self.create_subscription(
            UInt8, '/anafi/link/quality', self.quality_callback, qos)
        self.sub_goodput = self.create_subscription(
            UInt16, '/anafi/link/goodput', self.goodput_callback, qos)
        self.sub_rpy = self.create_subscription(
            Vector3Stamped, '/anafi/drone/rpy', self.rpy_callback, qos)

        # State variables
        self.drone_pos = (None, None)
        self.traj_pos = (None, None)
        self.gps_pos = (None, None)
        self.rssi = None
        self.quality = None
        self.goodput = None
        self.roll = None
        self.pitch = None
        self.yaw = None

        self.drone_path = []
        self.traj_path = []

        # Timing
        self.start_t = self.get_clock().now().nanoseconds * 1e-9

        # CSV setup with new RPY columns
        self.csv_file = open('modelagem180hz.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'time', 'drone_x', 'drone_y', 'target_x', 'target_y',
            'gps_x', 'gps_y', 'rssi', 'quality', 'goodput', 'roll', 'pitch', 'yaw'
        ])
        self.csv_file.flush()

        # Plot setup
        self.fig, self.ax = plt.subplots(figsize=(6.4, 4.8))
        self.ax.set_title("Drone Position vs. Desired Position")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        plt.ion()
        plt.show()

        # Timers: log at 180 Hz, plot at 5 Hz
        self.create_timer(1.0/180.0, self.log_data)
        self.create_timer(1.0/5.0, self.update_plot)

    # Callbacks only update state
    def position_callback(self, msg):
        self.drone_pos = (msg.data[0], msg.data[1])
        self.drone_path.append(self.drone_pos)

    def gps_callback(self, msg):
        self.gps_pos = (msg.data[0], msg.data[1])

    def trajectory_callback(self, msg):
        self.traj_pos = (msg.data[0], msg.data[1])
        self.traj_path.append(self.traj_pos)

    def rssi_callback(self, msg):
        self.rssi = msg.data

    def quality_callback(self, msg):
        self.quality = msg.data

    def goodput_callback(self, msg):
        self.goodput = msg.data

    def rpy_callback(self, msg):
        self.roll = msg.vector.x
        self.pitch = msg.vector.y
        self.yaw = msg.vector.z

    # Logging at 180 Hz
    def log_data(self):
        t = self.get_clock().now().nanoseconds * 1e-9 - self.start_t
        dx, dy = self.drone_pos
        tx, ty = self.traj_pos
        gx, gy = self.gps_pos
        row = [
            f"{t:.6f}",
            f"{dx:.4f}" if dx is not None else "",
            f"{dy:.4f}" if dy is not None else "",
            f"{tx:.4f}" if tx is not None else "",
            f"{ty:.4f}" if ty is not None else "",
            f"{gx:.4f}" if gx is not None else "",
            f"{gy:.4f}" if gy is not None else "",
            str(self.rssi) if self.rssi is not None else "",
            str(self.quality) if self.quality is not None else "",
            str(self.goodput) if self.goodput is not None else "",
            f"{self.roll:.6f}" if self.roll is not None else "",
            f"{self.pitch:.6f}" if self.pitch is not None else "",
            f"{self.yaw:.6f}" if self.yaw is not None else ""
        ]
        self.csv_writer.writerow(row)
        self.csv_file.flush()

    # Plotting at 5 Hz
    def update_plot(self):
        if None in self.drone_pos or None in self.traj_pos:
            return
        self.ax.clear()
        self.ax.set_title("Drone Position vs. Desired Position")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        r = 5
        self.ax.set_xlim(-r, r)
        self.ax.set_ylim(-r, r)
        self.ax.plot(*zip(*self.drone_path), linestyle='dashed', alpha=0.5, label='Drone Path')
        self.ax.plot(*zip(*self.traj_path), linestyle='dashed', alpha=0.5, label='Target Path')
        self.ax.scatter(*self.drone_pos, label='Drone')
        self.ax.scatter(*self.traj_pos, label='Target')
        self.ax.legend()
        plt.draw()
        plt.pause(0.001)

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Registro()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

