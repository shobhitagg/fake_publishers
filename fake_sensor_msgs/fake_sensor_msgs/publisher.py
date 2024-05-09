import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import JointState

class FakeSensorPublisher(Node):

    def __init__(self):
        super().__init__('fake_sensor_publisher')
        self.publisher_ = self.create_publisher(JointState, '/yk_tester/joint_states', 10)
        timer_period = 0.025  # seconds
        self.joint_state_timer = self.create_timer(timer_period, self.joint_state_callback)
        self.i = 0

    def joint_state_callback(self):
        msg = JointState()
        
        msg.name = ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t']
        msg.position = [np.random.uniform(-np.pi,np.pi) for _ in range(6)]
        msg.velocity = [0.0 for _ in range(6)]
        msg.effort = [0.0 for _ in range(6)]
        
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing JointState')
        self.i += 1
        
def main(args=None):
    rclpy.init(args=args)

    talker = FakeSensorPublisher()

    rclpy.spin(talker)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    talker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
