
#
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3Stamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import math

class PositionNode(Node):
    def __init__(self):
        super().__init__('gps_position_node')
        
        self.first_sample = None
        self.first_sample_yaw = None
        self.current_yaw = None

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.publisher_ = self.create_publisher(Float32MultiArray, '/anafi/drone/position_global', qos_profile)
        
        self.gps_subscription = self.create_subscription(PoseStamped, '/anafi/drone/pose', self.pose_callback, qos_profile)
        self.rpy_subscription = self.create_subscription(Vector3Stamped, '/anafi/drone/rpy', self.rpy_callback, qos_profile)
        #self.timer = self.create_timer(0.1, self.publish_message)

    def pose_callback(self, msg):
        position = msg.pose.position
        x = -position.x
        y = -position.y
        z = position.z

        if self.first_sample is None:
            self.first_sample = (x, y, z)
            print('zero')
            return

        x -= self.first_sample[0]
        y -= self.first_sample[1]
        z -= self.first_sample[2]
        #print('x:',round(x, 4),'y:', round(y, 4),'z:', round(z, 4))

        array_msg = Float32MultiArray()
        array_msg.data = [x, y, z, self.current_yaw if self.current_yaw is not None else 0.0]
        self.publisher_.publish(array_msg)
        
        if self.current_yaw is not None:
            self.get_logger().info(f'Publishing global position: x={x:.6f}m, y={y:.6f}m, z={z:.6f}m, yaw={self.current_yaw:.6f}')
        else:
            self.get_logger().info(f'Publishing global position: x={x:.6f}m, y={y:.6f}m, z={z:.6f}m, yaw=None')

    def rpy_callback(self, msg):
        self.current_yaw = math.radians(msg.vector.z)

        if self.first_sample_yaw is None:
            self.first_sample_yaw = self.current_yaw
            return

        self.current_yaw -= self.first_sample_yaw

        #self.get_logger().info(f'Updated yaw: {self.current_yaw:.6f}')

def main(args=None):
    rclpy.init(args=args)
    position_node = PositionNode()
    rclpy.spin(position_node)
    position_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
