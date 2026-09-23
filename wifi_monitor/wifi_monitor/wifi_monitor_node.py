#!/usr/bin/python3
import subprocess
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool


class WifiMonitor(Node):

    def __init__(self):
        super().__init__('wifi_monitor')
        self.declare_parameter('~/dev', 'wlx00c0ca91ebc1')
        self.declare_parameter('~/hz', 1.0)
        self.dev=self.get_parameter("~/dev").get_parameter_value().string_value
        self.hz=self.get_parameter("~/hz").get_parameter_value().double_value

        self.publisher = self.create_publisher(Bool, '/has_wifi', 1)
        timer_period = 1.0/self.hz  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.count = 0
        self.previous_error = False
        self.previous_success = False

    def timer_callback(self):
        if self.count > 0:
            self.count -= 1
            return
        try:
            wifi_str = subprocess.check_output(['ifconfig', self.dev], stderr=subprocess.STDOUT);
            b = Bool()
            b.data = "inet" in wifi_str.decode("latin1")
            self.publisher.publish(b)

            if not self.previous_success:
                self.previous_success = True
                self.previous_error = False
                self.get_logger().info("Retrieved status of interface %s. Now updating at %f Hz." % (self.dev, self.hz))

        except subprocess.CalledProcessError:
            if not self.previous_error:
                self.previous_error = True
                self.previous_success = False
                self.get_logger().error("Error checking status of interface %s. Will try again every 10s." % self.dev)
                self.count = max(0,int(10 * self.hz))



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = WifiMonitor()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
