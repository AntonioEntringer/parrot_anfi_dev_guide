import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3Stamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import math
import numpy as np

class GPSPositionNode(Node):
    def __init__(self):
        super().__init__('gps_position_node')

        # Variáveis para referência do primeiro GPS (zero de posição)
        self.first_sample = None
        self.first_sample_yaw = None

        # Armazena o yaw atual (obtido do tópico de RPY)
        self.current_yaw = 0.0

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.publisher_ = self.create_publisher(Float32MultiArray, '/anafi/drone/position_global', qos_profile)
        self.publisher_gps_only = self.create_publisher(Float32MultiArray, '/anafi/drone/position_global_gps_only', qos_profile)
        
        self.gps_subscription = self.create_subscription(NavSatFix, '/anafi/drone/gps/location', self.gps_callback, qos_profile)
        self.rpy_subscription = self.create_subscription(Vector3Stamped, '/anafi/drone/rpy', self.rpy_callback, qos_profile)
        self.speed_subscription = self.create_subscription(Vector3Stamped, '/anafi/drone/speed', self.speed_callback, qos_profile)
        #self.altitude_subscription = self.create_subscription(Vector3Stamped, '/anafi/drone/altitude', self.altitude_callback, qos_profile)

        # Inicializa o estado do filtro de Kalman
        # Estado: [x, y, z, vx, vy, vz]^T ( m/s)
        self.state = np.zeros((6, 1))
        # Covariância inicial
        self.P = np.eye(6)
        # Modelo de transição (discreto)
        self.dt = 1.0/30.0
        self.F = np.array([[1, 0, 0, self.dt, 0,      0],
                           [0, 1, 0, 0,      self.dt, 0],
                           [0, 0, 1, 0,      0,      self.dt],
                           [0, 0, 0, 1,      0,      0],
                           [0, 0, 0, 0,      1,      0],
                           [0, 0, 0, 0,      0,      1]])
        
        # Ruído de processo
        self.Q = np.diag([0.01, 0.01, 0.01, 0.1, 0.1, 0.1])

        # Matrizes de observação
        self.H_gps = np.array([[1, 0, 0, 0, 0, 0],
                               [0, 1, 0, 0, 0, 0],
                               [0, 0, 1, 0, 0, 0]])
        
        self.R_gps = np.diag([0.5, 0.5, 0.5])  # metros^2

        self.H_vel = np.array([[0, 0, 0, 1, 0, 0],
                               [0, 0, 0, 0, 1, 0],
                               [0, 0, 0, 0, 0, 1]])
        
        self.R_vel = np.diag([0.1, 0.1, 0.1])  # (m/s)^2

    def predict(self):

        self.state = self.F @ self.state #predicao filtro de kalmam
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z, H, R):

        #z = vetor de medição (coluna)
        #H = matriz de observação
        #R = covariância da medição
  
        y = z - H @ self.state  # inovação
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.state = self.state + K @ y
        self.P = (np.eye(len(self.state)) - K @ H) @ self.P
    
    def speed_callback(self, msg):
        #tempo atual  a partir do clock do ROS2
        current_time = self.get_clock().now().nanoseconds * 1e-9  # converte para segundos
        
        # usar o dt padrão no primeiro ciclo
        if not hasattr(self, 'last_speed_time'):
            self.last_speed_time = current_time
            self.get_logger().warn(f"dt padrao 1/30 sendo utilizado")
            dt = self.dt  # dt padrão (1/30 s)

        else:
            dt = current_time - self.last_speed_time
            self.last_speed_time = current_time
            self.get_logger().warn(f"current dt: {dt:.3f} s")

            if dt > 0.1:
                self.get_logger().warn(f"dt muito alto ({dt:.3f} s), amostra ignorada para evitar surtos.")

                return  # Ignora a amostra

        self.F = np.array([[1, 0, 0, dt, 0,  0],
                           [0, 1, 0, 0,  dt, 0],
                           [0, 0, 1, 0,  0,  dt],
                           [0, 0, 0, 1,  0,  0],
                           [0, 0, 0, 0,  1,  0],
                           [0, 0, 0, 0,  0,  1]])
        
        v_body = np.array([[-msg.vector.x],
                           [-msg.vector.y],
                           [msg.vector.z]])
        
        yaw = self.current_yaw  # em radianos
        R_yaw = np.array([[np.cos(yaw), -np.sin(yaw), 0], #rotacao
                          [np.sin(yaw),  np.cos(yaw), 0],
                          [0, 0, 1]])
        v_world = R_yaw @ v_body  # velocidade no mundo
        
        # Realiza a predição usando o dt atualizado.
        self.predict()
        # Atualiza o filtro com a medição de velocidade (no referencial global)
        self.update(v_world, self.H_vel, self.R_vel)
        
        # Publica a posição estimada
        fused_msg = Float32MultiArray()
        fused_msg.data = [float(-self.state[0]), float(-self.state[1]), float(self.state[2]), self.current_yaw]
        self.publisher_.publish(fused_msg)
        self.get_logger().info(f"[FO] Posição estimada: x={-self.state[0,0]:.2f} m, y={-self.state[1,0]:.2f} m, z={self.state[2,0]:.2f} m, yaw={math.degrees(self.current_yaw):.2f}°")


    def gps_callback(self, msg):

        current_lat = msg.latitude
        current_lon = msg.longitude
        current_alt = msg.altitude

        # Converter a latitude e longitude para metros (relativo ao primeiro dado)
        if self.first_sample is None:
            self.first_sample = (current_lat, current_lon, current_alt)
            return

        delta_lat = current_lat - self.first_sample[0]
        delta_lon = current_lon - self.first_sample[1]
        delta_alt = current_alt - self.first_sample[2]

        lat_to_m = 111320   # mais ou menos isso
        lon_to_m = 40008000 * np.cos(math.radians(self.first_sample[0])) / 360

        y_gps = delta_lat * lat_to_m
        x_gps = delta_lon * lon_to_m
        z_gps = delta_alt

        c, s = np.cos(-self.first_sample_yaw), np.sin(-self.first_sample_yaw)
        R_z = np.array([
            [ c, -s, 0],
            [ s, c, 0],
            [ 0,  0, 1],
        ])

        z_rotated_r = np.array([[-y_gps],
                              [x_gps],
                              [z_gps]])

        # vetor rotacionado para o frame inicial
        z_rotated = R_z @ z_rotated_r               # shape (3,)

        # se precisar em (3,1):
        z_rotated_r = z_rotated.reshape(3,1)


        # Atualiza o filtro com a medição de posição do GPS
        self.predict()
        self.update(z_rotated_r, self.H_gps, self.R_gps)

        fused_msg = Float32MultiArray()
        fused_msg.data = [-float(self.state[0]), -float(self.state[1]), float(self.state[2]), self.current_yaw]
        self.publisher_.publish(fused_msg)
        self.publisher_gps_only.publish(fused_msg)

        self.get_logger().info(f"[GPS] Posição estimada: x={-self.state[0,0]:.2f} m, y={-self.state[1,0]:.2f} m, z={self.state[2,0]:.2f} m, yaw={math.degrees(self.current_yaw):.2f}°")
        
    def rpy_callback(self, msg):
        current_yaw = math.radians(msg.vector.z)
        if self.first_sample_yaw is None:
            self.first_sample_yaw = current_yaw
        self.current_yaw = current_yaw - self.first_sample_yaw


def main(args=None):
    rclpy.init(args=args)
    node = GPSPositionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
