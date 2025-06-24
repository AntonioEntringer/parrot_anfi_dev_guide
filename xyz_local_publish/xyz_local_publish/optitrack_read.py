import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import math

class PositionNode(Node):
    def __init__(self):
        super().__init__('vrpn_position_node')
        
        # variáveis de estado
        self.first_pos = None
        self.first_yaw = None
        self.latest = None  # tupla (x_rel, y_rel, z_rel, yaw_rel)
        
        # QoS igual ao original
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # publisher mantém o mesmo tópico e tipo
        self.pub = self.create_publisher(Float32MultiArray,
                                         '/anafi/drone/position_global',
                                         qos)
        # subscription única ao VRPN
        self.sub = self.create_subscription(PoseStamped,
                                            '/vrpn_mocap/B1/pose',
                                            self.cb_vrpn,
                                            qos)
        # timer para forçar 180 Hz
        period = 1.0 / 90.0
        self.create_timer(period, self.publish_loop)

    def cb_vrpn(self, msg: PoseStamped):
        # posição VRPN
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        # quaternion → yaw
        q = msg.pose.orientation
        # formula: yaw = atan2(2*(w*z + x*y), 1 - 2*(y²+z²))
        yaw = math.atan2(
            2.0*(q.w*q.z + q.x*q.y),
            1.0 - 2.0*(q.y*q.y + q.z*q.z)
        )

        # primeira amostra: armazena offset
        if self.first_pos is None:
            self.first_pos = (x, y, z)
            self.first_yaw = yaw
            return

        # calcula posição/yaw relativos
        x_rel = x - self.first_pos[0]
        y_rel = y - self.first_pos[1]
        z_rel = z - self.first_pos[2]
        yaw_rel = yaw - self.first_yaw

        self.latest = (x_rel, y_rel, z_rel, yaw_rel)

    def publish_loop(self):
        if self.latest is None:
            return
        arr = Float32MultiArray()
        arr.data = [ *self.latest ]
        self.pub.publish(arr)
        x, y, z, yaw = self.latest
        self.get_logger().info(
            f'[{self.get_clock().now().to_msg().sec}] '
            f'x={x:.4f}, y={y:.4f}, z={z:.4f}, yaw={yaw:.4f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = PositionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()