import rclpy
import numpy as np
from rclpy.node import Node

from geometry_msgs.msg import WrenchStamped, Pose

class FakeGeometryPublisher(Node):

    def __init__(self):
        super().__init__('fake_geometry_publisher')
        timer_period = 0.025  # seconds
        
        self.wrench_publisher_ = self.create_publisher(WrenchStamped, '/fts', 10)
        self.wrenchtimer = self.create_timer(timer_period, self.wrench_callback)
        self.wrench_i = 0
        
        self.pose_publisher_ = self.create_publisher(Pose, '/yk_tester/eef_pose', 10)
        self.posetimer = self.create_timer(timer_period, self.pose_callback)
        self.pose_i = 0

    def wrench_callback(self):
        msg = WrenchStamped()
        
        msg.wrench.force.x = np.random.uniform(-10,10)
        msg.wrench.force.y = np.random.uniform(-10,10)
        msg.wrench.force.z = np.random.uniform(-20,20)
        msg.wrench.torque.x = np.random.uniform(-10,10)
        msg.wrench.torque.y = np.random.uniform(-10,10)
        msg.wrench.torque.z = np.random.uniform(-10,10)
        
        self.wrench_publisher_.publish(msg)
        self.get_logger().info('Publishing /fts data')
        self.wrench_i += 1

    def pose_callback(self):
        msg = Pose()
        
        msg.position.x = np.random.uniform(-0.5,0.5)
        msg.position.y = np.random.uniform(-0.5,0.5)
        msg.position.z = np.random.uniform(-0.5,0.5)
        msg.orientation.x = np.random.uniform(-0.5,0.5)
        msg.orientation.y = np.random.uniform(-0.5,0.5)
        msg.orientation.z = np.random.uniform(-0.5,0.5)
        msg.orientation.w = np.random.uniform(-0.5,0.5)
        
        self.pose_publisher_.publish(msg)
        self.get_logger().info('Publishing /yk_tester/eef_pose data')
        self.pose_i += 1

def main(args=None):
    rclpy.init(args=args)

    talker = FakeGeometryPublisher()

    rclpy.spin(talker)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    talker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
