import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time


class Navigator(Node):

    def __init__(self):
        super().__init__('navigator')

        self.state = 'clear'
        self.turn_start_time = 0.0
        self.turn_duration = 1.2

        self.forward_speed = 0.18
        self.turn_speed = 0.5

        self.create_subscription(String, '/obstacle_event', self.on_event, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_timer(0.05, self.update_motion)

        self.get_logger().info("Navigator running")

    def on_event(self, msg: String):
        event = msg.data
        self.get_logger().info(f"Navigator received: {event}")

        if event == 'turn_left':
            self.state = 'turn_left'
            self.turn_start_time = time.time()
        else:
            self.state = event

    def update_motion(self):
        cmd = Twist()

        if self.state == 'STOP_YELLOW':
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        elif self.state == 'turn_left':
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_speed

            if time.time() - self.turn_start_time > self.turn_duration:
                self.state = 'clear'

        else:
            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0

        self.pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = Navigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
