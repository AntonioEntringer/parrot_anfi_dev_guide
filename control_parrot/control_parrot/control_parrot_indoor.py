import rclpy
import math
import numpy as np
from rclpy.node import Node
from anafi_ros_interfaces.msg import PilotingCommand
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Header, Float32MultiArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import time


class ParrotControl(Node):
    def __init__(self):
        super().__init__('parrot_control')

        qos_profile1 = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        qos_profile2 = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.SYSTEM_DEFAULT,
            depth=10
        )

        # Publishers
        self.publisher_ = self.create_publisher(PilotingCommand, '/anafi/drone/command_autonomo', qos_profile1)
        self.xyzw_trajetoria = self.create_publisher(Float32MultiArray, '/anafi/drone/xyzw_trajetoria', qos_profile2)
        self.get_logger().info(f'The topic "xyzw_trajetoria" has been successfully created.')

        # Subscriptions        
        self.subscription = self.create_subscription(Float32MultiArray, '/anafi/drone/position_global', self.position_callback, qos_profile1)
        self.subscription = self.create_subscription(Vector3Stamped, '/anafi/drone/speed', self.speed_callback, qos_profile1)
        self.timer = self.create_timer(0.01, self.publish_message)  # Publish at 100 Hz

        self.command = PilotingCommand()
        self.command.header = Header()
        self.command.roll = 0.0
        self.command.pitch = 0.0
        self.command.yaw = 0.0
        self.command.gaz = 0.0

        self.x = None  # posição atual [x, y, z, psi]
        self.x_dot = (0.0, 0.0, 0.0)  # velocidade atual [vx, vy, vz]
        
        self.first_time = 1

    def position_callback(self, msg):
        self.x = msg.data
        #self.get_logger().info(f'Received position: x={self.x[0]:.6f}m, y={self.x[1]:.6f}m, z={self.x[2]:.6f}m')

    def speed_callback(self, msg):
        self.x_dot = (msg.vector.x, msg.vector.y, msg.vector.z)
        #print(self.x_dot,'vel')

    def publish_message(self):
        if self.x:
            # posição
            x_pos = self.x[0]
            y_pos = self.x[1]
            z_pos = self.x[2]
            psi = self.x[3]
            x = np.array([x_pos, y_pos, z_pos])  # posição do drone
            
            # velocidade
            x_dot = np.transpose(np.array([[self.x_dot[0], self.x_dot[1]]]))

            if self.first_time == 1:
                self.first_time = 0

                # Constantes físicas e parâmetros fixos
                self.g = 9.81                                  # gravidade [m/s²]
                self.tilt_max = 15                             # inclinação máxima [graus]
                self.vel_max = 2                               # velocidade máxima [m/s]
                self.zdot_max = 1                              # velocidade vertical máxima [m/s]
                self.kp = 13.89                              # ganho proporcional (posição)
                self.kd = 3.16                                # ganho derivativo (posição)
                self.kd2 = 5                                 # ganho derivativo (velocidade)
                self.kp_z = 1                                # ganho proporcional (controle em Z)
                self.kp_psi = 1                                # ganho proporcional (yaw)
                self.U_inv = np.array([[(1 / self.g), 0],    # modelo empirico
                                        [0, (1 / self.g)]])

                # Variáveis de estado e trajetória
                self.psi_d = 0                                 # yaw desejado [rad]
                self.xd_old = x                              # posição desejada anterior
                self.r = 1                                     # raio da trajetória [m]
                self.w = (2 * np.pi) / 40                      # frequência angular da trajetória [rad/s]
                self.t_inicial = time.time()                   # tempo inicial


            t = time.time() - self.t_inicial

            # trajetória desejada
            self.xd = np.array([  self.r *               np.sin(self.w * t),                      self.r * np.sin(2 * self.w * t),    1])  # lemniscata
            self.xd_dot = np.array([   self.r * self.w *      np.cos(self.w * t),         2 * self.r * self.w * np.cos(2 * self.w * t),    0])
            self.xd_2dot = np.array([  -self.r * self.w ** 2 * np.sin(self.w * t),   -4 * self.r * self.w ** 2 * np.sin(2 * self.w * t),    0])


            # Publish d_trajectory
            traj_msg = Float32MultiArray()
            traj_msg.data = [float(self.xd[0]), float(self.xd[1]), float(self.xd[2]), float(self.psi_d)]
            self.xyzw_trajetoria.publish(traj_msg)

            R = np.array([[np.cos(psi), np.sin(psi)],
                        [-np.sin(psi),  np.cos(psi)]])


            
            # controle linear XY
            x_dot = np.array([[self.x_dot[0]], [self.x_dot[1]]])
            x_dot = np.dot(R.T, x_dot)  
            x_ = np.array([[x[0]], [x[1]]])
            xd_ = np.array([[self.xd[0]], [self.xd[1]]])
            xd_dot_ = np.array([[self.xd_dot[0]], [self.xd_dot[1]]])
            xd_2dot_ = np.array([[self.xd_2dot[0]], [self.xd_2dot[1]]])



            u_ref = xd_2dot_ + self.kd * (xd_dot_ - x_dot) + self.kp * (xd_ - x_)
            u_ref = self.kd2 * (u_ref - x_dot)
            u_ref = np.dot(R, u_ref) 
            
            # LEI DE CONTROLE EMPIRICO
            #self.U = self.U_inv @ u_ref
            # LEI DE CONTROLE CASO 1
            #self.U = np.dot(np.array([[9.72, 0], [0, 9.24]]),self.xd_2dot_)
            # LEI DE CONTROLE CASO 2
            self.U = np.dot(np.array([[0.1, 0], [0, 0.1]]), u_ref) - np.dot(np.array([[0.005, 0], [0, 0.005]]), x_dot)
            # LEI DE CONTROLE CASO 3
            #self.U = np.dot(np.array([[9.73, 0], [0, 9.93]]), self.xd_2dot_) - np.dot(np.array([[0.24, 0], [0, 0.29]]), vel) - np.dot(np.array([[0.01, 0],[0, 0.08]]),np.multiply(vel,vel))


            self.U = np.maximum(-self.tilt_max, np.minimum(self.tilt_max, self.U))
    
            # controle linear Z
            zdot_ref = self.kp_z * (self.xd[2] - self.x[2]) + self.xd_dot[2] + self.xd_2dot[2]
            zdot_ref = max(-self.zdot_max, min(self.zdot_max, zdot_ref))

            # controlador angular z
            delta_x = self.xd[0] - self.xd_old[0]
            delta_y = self.xd[1] - self.xd_old[1]
            if abs(delta_x) > 1e-6 or abs(delta_y) > 1e-6:
                self.psi_d = np.arctan2(delta_y, delta_x)
                #self.psi_d = -np.pi/2
            # Atualiza a posição desejada anterior
            self.xd_old = self.xd.copy()
            psi_erro = self.psi_d - psi
            if abs(psi_erro) > np.pi:
                if psi_erro > np.pi:
                    psi_erro -= 2*np.pi
                else:
                    psi_erro += 2*np.pi

            # controle Yaw
            yaw_ref = (self.kp_psi * psi_erro) * 180/np.pi

            print('\n\nRoll', round(float(-self.U[1]), 4), 'Pitch', round(float(self.U[0]), 4))
            print('Vel_ref_X', round(float(u_ref[0]), 4), 'Vel_ref_Y', round(float(u_ref[1]), 4),'Vel_ref_z:',zdot_ref)
            print('tempo:', t)

            self.command.header.stamp = self.get_clock().now().to_msg()
            self.command.roll = float(-self.U[1, 0])
            self.command.pitch = float(self.U[0, 0])
            self.command.gaz = float(zdot_ref)
            self.command.yaw = float(yaw_ref)
            self.publisher_.publish(self.command)


def main(args=None):
    rclpy.init(args=args)
    parrot_control = ParrotControl()
    rclpy.spin(parrot_control)
    parrot_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
