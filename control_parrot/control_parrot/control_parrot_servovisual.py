import rclpy
import numpy as np
import cv2
from cv_bridge import CvBridge
from rclpy.node import Node
from anafi_ros_interfaces.msg import PilotingCommand
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class ParrotControl(Node):
    def __init__(self):
        super().__init__('parrot_control')

        # QoS Profiles
        qos_profile1 = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)
        qos_profile2 = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                  durability=QoSDurabilityPolicy.SYSTEM_DEFAULT, depth=10)

        # Limites de controle
        self.lim_horiontal_vel = 0.3    # m/s
        self.lim_vertical_vel = 1.0     # m/s
        self.lim_horizontal_tilt = 40   # graus
        self.Delta = np.array([0.0, 0.0, 1.5])  # manter 1.5 m acima do ArUco

        # Estados internos
        self.first_time = True
        self.current_speed = (0.0, 0.0, 0.0)
        self.aruco_pose = None              # (x, y, z, yaw)
        self.vel_ref_R = np.zeros((2, 1))    # velocidade desejada XY saturada (2×1)
        self.erro_norm = 0.0                # norma do erro de posição XY
        self.last_frame = None              # frame OpenCV mais recente
        self.last_frame_time = None         # Timestamp do último frame recebido

        # CvBridge e parâmetros ArUco
        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.marker_length = 0.30           # tamanho do marcador em metros

        # Matriz de calibração da câmera (substituir pelos valores reais)
        self.camera_matrix = np.array([[300.0,   0.0, 1280/2],
                                       [  0.0, 300.0, 720/2],
                                       [  0.0,   0.0,   1.0]])
        self.dist_coeffs = np.zeros((5, 1))  # sem distorção

        # Publisher de comando de voo
        self.publisher_ = self.create_publisher(PilotingCommand,
                                                '/anafi/drone/command_autonomo',
                                                qos_profile2)

        # Publisher da imagem anotada com ArUco
        self.image_pub_ = self.create_publisher(Image,
                                                '/anafi/camera/image_aruco',
                                                qos_profile1)

        # Subscriptions
        self.create_subscription(Vector3Stamped,
                                 '/anafi/drone/speed',
                                 self.speed_callback,
                                 qos_profile1)
        self.create_subscription(Image,
                                 '/anafi/camera/image',
                                 self.image_callback,
                                 qos_profile1)

        # Timer para ler e processar a última imagem a 80 Hz (0.0125 s)
        self.timer = self.create_timer(1.0 / 80.0, self.control_loop)

        # Mensagem de comando inicializada
        self.command = PilotingCommand()
        self.command.header = Header()
        self.command.roll = 0.0
        self.command.pitch = 0.0
        self.command.yaw = 0.0
        self.command.gaz = 0.0

        self.get_logger().info("ParrotControl: subscrições criadas e nó pronto.")

    def speed_callback(self, msg: Vector3Stamped):
        # Armazena velocidade atual do drone (x, y, z)
        self.current_speed = (msg.vector.x, msg.vector.y, msg.vector.z)

    def image_callback(self, msg: Image):
        # Converte ROS Image para OpenCV e armazena na variável last_frame e tempo
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.last_frame = frame
            self.last_frame_time = msg.header.stamp
        except Exception as e:
            self.get_logger().error(f"Falha ao converter imagem: {e}")

    def control_loop(self):
        # Chama detecção/annotação de ArUco na última imagem (mesma imagem se não chegou nova)
        if self.last_frame is not None:
            frame = self.last_frame.copy()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(gray,
                                                      self.aruco_dict,
                                                      parameters=self.aruco_params)

            if ids is not None and 0 in ids.flatten():
                index = list(ids.flatten()).index(0)
                marker_corners = [corners[index]]

                # Estima pose do marcador
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(marker_corners,
                                                                       self.marker_length,
                                                                       self.camera_matrix,
                                                                       self.dist_coeffs)
                rvec = rvecs[0][0]  # (x, y, z) em rad
                tvec = tvecs[0][0]  # (x, y, z) em m

                # Desenha contorno e eixos
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                cv2.drawFrameAxes(frame,
                                  self.camera_matrix,
                                  self.dist_coeffs,
                                  rvec,
                                  tvec,
                                  self.marker_length * 0.5)

                # Atualiza pose do ArUco
                self.aruco_pose = (tvec[0], tvec[1], tvec[2], rvec[0])
            else:
                # Se não detectou, seta pose como None
                self.aruco_pose = None

            # Sobreposição de texto: velocidades e erro
            vx, vy, _ = self.current_speed
            vel_atual_str = f"Vel atual: [{vx:.2f}, {vy:.2f}] m/s"

            vdx = float(self.vel_ref_R[0, 0])
            vdy = float(self.vel_ref_R[1, 0])
            vel_desej_str = f"Vel desejada: [{vdx:.2f}, {vdy:.2f}] m/s"

            erro_norm_str = f"Erro norm: {self.erro_norm:.2f} m"

            org1 = (10, 30)
            org2 = (10, 60)
            org3 = (10, 90)

            fonte = cv2.FONT_HERSHEY_SIMPLEX
            escala = 0.6
            cor = (0, 255, 0)
            espessura = 2

            cv2.putText(frame, vel_atual_str, org1, fonte, escala, cor, espessura, cv2.LINE_AA)
            cv2.putText(frame, vel_desej_str, org2, fonte, escala, cor, espessura, cv2.LINE_AA)
            cv2.putText(frame, erro_norm_str, org3, fonte, escala, cor, espessura, cv2.LINE_AA)

            # Converte de volta para ROS Image e publica
            try:
                annotated_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                annotated_msg.header.stamp = self.get_clock().now().to_msg()
                self.image_pub_.publish(annotated_msg)
            except Exception as e:
                self.get_logger().error(f"Falha ao publicar imagem anotada: {e}")
        else:
            # Se nunca leu quadro válido, mantém aruco_pose None
            self.aruco_pose = None
            self.vel_ref_R = np.zeros((2, 1))
            self.erro_norm = 0.0

        # Após detecção, publica comando de voo
        self.publish_message()

    def publish_message(self):
        if self.aruco_pose is None:
            self.vel_ref_R = np.zeros((2, 1))
            self.erro_norm = 0.0
            return

        # Se chegar aqui, significa que self.aruco_pose não é None
        x, y, z, psi = self.aruco_pose
        Pos_VANT = np.array([y, -x, -z])

        dot_x, dot_y, _ = self.current_speed
        drone_vel = np.transpose(np.matrix([[dot_x, dot_y]]))  # 2×1

        # Inicialização dos ganhos na primeira chamada
        if self.first_time:
            self.first_time = False
            self.g = 9.81
            self.tilt_max = 10.0
            u_scale = 1.0 / (self.g * self.tilt_max)
            self.U_inv = np.matrix([[u_scale, 0.0],
                                    [0.0, u_scale]])
            self.zdot_max = 1.0
            self.kp = 1.0
            self.kd = 3.0
            self.kp_psi = 1
            self.psi_aruco = 0.0

        # Posição desejada: 1.5 m acima do marcador
        Pose_aruco = np.array([0.0, 0.0, 0.0])
        self.pos_desejada_VANT = Pose_aruco + self.Delta
        erro_pos = self.pos_desejada_VANT - Pos_VANT

        # Norma do erro XY
        self.erro_norm = float(np.linalg.norm([erro_pos[0], erro_pos[1]]))
        print(erro_pos[0], erro_pos[1])
        erro_pos_xy = np.transpose(np.matrix([[erro_pos[0], erro_pos[1]]]))  # 2×1

        # Controle proporcional XY
        self.vel_ref = self.kp * erro_pos_xy
        self.vel_ref_R = np.maximum(-self.lim_horiontal_vel,
                                    np.minimum(self.lim_horiontal_vel, self.vel_ref))

        # Controle PD aceleração XY
        self.ac_ref = self.kd * self.vel_ref_R  # + drone_vel
        self.U = self.ac_ref  # self.U_inv.dot(self.ac_ref)

        self.U = np.maximum(-self.lim_horizontal_tilt,
                            np.minimum(self.lim_horizontal_tilt, self.U))

        # Controle proporcional Z
        self.Zdot_ref = self.kp * (self.pos_desejada_VANT[2] - Pos_VANT[2])
        self.Zdot_ref = (self.Zdot_ref / self.zdot_max) * 0.0
        self.Zdot_ref = max(-self.lim_vertical_vel,
                            min(self.lim_vertical_vel, self.Zdot_ref))

        # Erro de yaw [-π, π]
        psi_erro = psi - self.psi_aruco
        if abs(psi_erro) > np.pi:
            if psi_erro > np.pi:
                psi_erro -= 2.0 * np.pi
            else:
                psi_erro += 2.0 * np.pi

        # Controle yaw (graus/s)
        Yawdot_ref = self.kp_psi * np.rad2deg(psi_erro)

        # Publica comando de voo normalmente
        self.command.header.stamp = self.get_clock().now().to_msg()
        self.command.roll = float(self.U[1, 0])
        self.command.pitch = float(self.U[0, 0])
        # self.command.gaz = float(self.Zdot_ref)
        # self.command.yaw = float(-Yawdot_ref)
        self.publisher_.publish(self.command)
        #print(f"Comando publicado: roll={self.command.roll:.2f}, "
        #      f"pitch={self.command.pitch:.2f}, "
        #      f"yaw={Yawdot_ref:.2f}, gaz={self.Zdot_ref:.2f}")

def main(args=None):
    rclpy.init(args=args)
    parrot_control = ParrotControl()
    rclpy.spin(parrot_control)
    parrot_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
