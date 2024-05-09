import rclpy
from rclpy.node import Node

from industrial_msgs.msg import RobotStatus, TriState, RobotMode

class FakeIndustrialPublisher(Node):

    def __init__(self):
        super().__init__('fake_industrial_publisher')
        self.publisher_ = self.create_publisher(RobotStatus, '/yk_tester/robot_status', 10)
        timer_period = 0.025  # seconds
        self.timer = self.create_timer(timer_period, self.robot_status_callback)
        self.i = 0

    def robot_status_callback(self):
        msg = RobotStatus()
        
        msg.mode.val = RobotMode.AUTO
        msg.e_stopped.val = TriState.FALSE
        msg.drives_powered.val = TriState.TRUE
        msg.motion_possible.val = TriState.TRUE
        msg.in_error.val = TriState.FALSE
        
        # assign random value to in_motion
        if self.i % 10 == 0:
            msg.in_motion.val = TriState.TRUE
        else:
            msg.in_motion.val = TriState.FALSE
        
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing RobotStatus')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    talker = FakeIndustrialPublisher()

    rclpy.spin(talker)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    talker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
