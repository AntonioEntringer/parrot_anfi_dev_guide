import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Joy
from anafi_ros_interfaces.msg import PilotingCommand, GimbalCommand
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Header
from std_srvs.srv import Trigger
import numpy as np
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class JoystickControl(Node):
    def __init__(self):
        super().__init__('joystick_control')

        # QoS para comandos de drone e gimbal compatível com o subscriber
        qos_profile_cmd = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # QoS para assinaturas de sensores (BEST_EFFORT)
        qos_profile_sens = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Publishers
        self.publisher_drone = self.create_publisher(PilotingCommand, '/anafi/drone/command', qos_profile_cmd)
        self.publisher_gimbal = self.create_publisher(GimbalCommand,'/anafi/gimbal/command', qos_profile_cmd)
        # Subscriptions
        self.joy_subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.speed_subscription = self.create_subscription(Vector3Stamped, '/anafi/drone/speed', self.speed_callback, qos_profile_sens)
        self.autonomous_subscription = self.create_subscription(PilotingCommand, '/anafi/drone/command_autonomo', self.autonomous_callback, qos_profile_sens)

        # Estado interno
        self.last_joy_msg = None
        self.speed_msg = None
        self.autonomous_msg = None

        # Comandos padrão
        self.drone_cmd = PilotingCommand()
        self.drone_cmd.header = Header()
        self.drone_cmd.roll = 0.0
        self.drone_cmd.pitch = 0.0
        self.drone_cmd.yaw = 0.0
        self.drone_cmd.gaz = 0.0

        self.gimbal_cmd = GimbalCommand()
        self.gimbal_cmd.header = Header()
        self.gimbal_cmd.mode = 1  # modo de controle PTZ
        self.gimbal_cmd.frame = 1
        self.gimbal_cmd.roll = 0.0
        self.gimbal_cmd.pitch = 0.0
        self.gimbal_cmd.yaw = 0.0

        # Controle e parâmetros
        self.k_speed = 1.0
        self.prev_axis_8 = 0
        self.control_mode = "angle"
        self.prev_button_3 = False
        self.pass_through_mode = False
        self.prev_button_4 = False

        # Timer de publicação (60 Hz)
        self.timer = self.create_timer(1/60.0, self.publish_command)

        # Serviços de takeoff/land e parâmetros
        self.takeoff_client = self.create_client(Trigger, '/anafi/drone/takeoff')
        self.land_client = self.create_client(Trigger, '/anafi/drone/land')
        self.param_client = self.create_client(SetParameters, '/anafi/anafi/set_parameters')
        self.wait_for_services()
        self.set_drone_parameters()

    def wait_for_services(self):
        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Aguardando takeoff...')
        while not self.land_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Aguardando land...')
        while not self.param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Aguardando set_parameters...')

    def set_drone_parameters(self):
        req = SetParameters.Request()
        parametros = [
            ("drone/max_altitude", 1000.0),
            ("drone/max_distance", 1000.0),
            ("drone/max_horizontal_speed", 15.0),
            ("drone/max_vertical_speed", 4.0),
            ("drone/max_pitch_roll", 40.0)
        ]
        for nome, valor in parametros:
            param = Parameter()
            param.name = nome
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = float(valor)
            req.parameters.append(param)
        future = self.param_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().info("Parâmetros definidos com sucesso!")
        else:
            self.get_logger().error("Falha ao definir parâmetros.")

    def joy_callback(self, msg: Joy):
        self.last_joy_msg = msg

    def speed_callback(self, msg: Vector3Stamped):
        self.speed_msg = msg

    def autonomous_callback(self, msg: PilotingCommand):
        self.autonomous_msg = msg

    def publish_command(self):
        # Toggle pass-through com botão 4
        if self.last_joy_msg:
            btns = self.last_joy_msg.buttons
            if btns[3] and not self.prev_button_4:
                self.pass_through_mode = not self.pass_through_mode
                modo = "pass-through" if self.pass_through_mode else "manual"
                self.get_logger().info(f"Modo: {modo}")
            self.prev_button_4 = btns[3]

        if self.pass_through_mode:
            if self.autonomous_msg:
                self.autonomous_msg.header.stamp = self.get_clock().now().to_msg()
                self.publisher_drone.publish(self.autonomous_msg)
            self.last_joy_msg = None
            return

        if not self.last_joy_msg:
            return

        axes = self.last_joy_msg.axes
        btns = self.last_joy_msg.buttons

        # Zona morta
        if not any(abs(a) > 0.1 for a in axes):
            self.last_joy_msg = None
            return

        # Ajuste de ganho (eixo 8)
        axis_8 = int(np.sign(axes[7]))
        if axis_8 and axis_8 != self.prev_axis_8:
            self.k_speed = np.clip(self.k_speed + axis_8, 0, 15)
            self.get_logger().info(f'k_speed = {self.k_speed}')
        self.prev_axis_8 = axis_8

        # Modo angle/speed (botão 3)
        if btns[2] and not self.prev_button_3:
            self.control_mode = 'speed' if self.control_mode == 'angle' else 'angle'
            self.get_logger().info(f"Modo de controle: {self.control_mode}")
        self.prev_button_3 = btns[2]

        # Mapeamento yaw
        yaw = np.clip(axes[3], -1, 1) * 100

        # Gimbal pitch via eixo do gaz atual (axes[4])
        gimbal_pitch = float(np.clip(-axes[4], -1, 1))  # escala em graus
        self.gimbal_cmd.header.stamp = self.get_clock().now().to_msg()
        self.gimbal_cmd.pitch = gimbal_pitch
        self.publisher_gimbal.publish(self.gimbal_cmd)

        # Gaz (drone) mapeado para gatilhos: RT (axes[5]) sobe, LT (axes[2]) desce
        rt = (axes[5] + 1.0) / 2.0  # normaliza de [-1,1] para [0,1]
        lt = (axes[2] + 1.0) / 2.0
        gaz_cmd = float((lt - rt) * self.k_speed)

        # Cálculo de roll/pitch drone
        if self.control_mode == 'angle':
            roll_cmd = -np.clip(axes[0], -1, 1) * 40
            pitch_cmd = np.clip(axes[1], -1, 1) * 40
        else:
            if not self.speed_msg:
                self.get_logger().warn("Velocidade indisponível.")
                self.last_joy_msg = None
                return
            desired_x = np.clip(axes[1], -1, 1) * self.k_speed
            desired_y = np.clip(axes[0], -1, 1) * self.k_speed
            vel_x = desired_x + np.clip(10*(desired_x - self.speed_msg.vector.x), -40, 40)
            vel_y = desired_y + np.clip(10*(desired_y - self.speed_msg.vector.y), -40, 40)
            pitch_cmd = np.clip(vel_x, -40, 40)
            roll_cmd = np.clip(-vel_y, -40, 40)

        # Takeoff / Land
        if btns[0]:
            self.get_logger().info("TAKEOFF...")
            self.takeoff_client.call_async(Trigger.Request())
            self.get_clock().sleep_for(Duration(seconds=2))
        elif btns[1]:
            self.get_logger().info("LAND...")
            self.land_client.call_async(Trigger.Request())
            self.get_clock().sleep_for(Duration(seconds=2))

        # Publica comando drone
        self.drone_cmd.header.stamp = self.get_clock().now().to_msg()
        self.drone_cmd.roll = float(roll_cmd)
        self.drone_cmd.pitch = float(pitch_cmd)
        self.drone_cmd.yaw = float(yaw)
        self.drone_cmd.gaz = gaz_cmd
        self.publisher_drone.publish(self.drone_cmd)

        self.last_joy_msg = None


def main(args=None):
    rclpy.init(args=args)
    node = JoystickControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
