#!/usr/bin/python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from kingfisher_msgs.msg import Drive
from math import fabs, copysign

class SignControl:
    def __init__(self,node,dead_time=0.25):
        self.dead_time = dead_time
        self.state = 0
        self.stamp = node.get_clock().now()

    def set_dead_time(self, dead_time):
        self.dead_time = dead_time

    def update(self,node,value):
        now = node.get_clock().now()
        if self.state == -1:
            if value < -0.01:
                return value
            else:
                self.state = 0
                self.stamp = now
                return 0.0
        if self.state == 0:
            if (value < -0.01) and ((now-self.stamp).nanoseconds/1e9 > self.dead_time):
                self.state = -1
                self.stamp = now
                return value
            if (value >  0.01) and ((now-self.stamp).nanoseconds/1e9 > self.dead_time):
                self.state = +1
                self.stamp = now
                return value
            return 0.0
        if self.state == +1:
            if value >  0.01:
                return value
            else:
                self.state = 0
                self.stamp = now
                return 0.0



class KingfisherTwist(Node):

    def __init__(self):
        super().__init__('kingfisher_twist')
        self.declare_parameter('rotation_scale', 0.2)
        self.declare_parameter('fwd_speed_scale', 1.0)
        self.declare_parameter('rev_speed_scale', 1.0)
        self.declare_parameter('left_max', 1.0)
        self.declare_parameter('right_max', 1.0)
        self.declare_parameter('dead_time', 1.0)

        self.rotation_scale=self.get_parameter("rotation_scale").get_parameter_value().double_value
        self.fwd_speed_scale=self.get_parameter("fwd_speed_scale").get_parameter_value().double_value
        self.rev_speed_scale=self.get_parameter("rev_speed_scale").get_parameter_value().double_value
        self.left_max=self.get_parameter("left_max").get_parameter_value().double_value
        self.right_max=self.get_parameter("right_max").get_parameter_value().double_value
        self.dead_time=self.get_parameter("dead_time").get_parameter_value().double_value
        
        self.state_left = SignControl(self,self.dead_time)
        self.state_right = SignControl(self,self.dead_time)

        self.cmd_pub = self.create_publisher(Drive,'cmd_drive', 1)
        self.vel_sub = self.create_subscription(Twist,"cmd_vel", self.callback, 1) 


    def callback(self, twist):
        """ Receive twist message, formulate and send Chameleon speed msg. """
        cmd = Drive()
        speed_scale = 0.0
        if twist.linear.x > 0.01: speed_scale = self.fwd_speed_scale
        if twist.linear.x < -0.01: speed_scale = self.rev_speed_scale
        cmd.left = (twist.linear.x * speed_scale) - (twist.angular.z * self.rotation_scale)
        cmd.right = (twist.linear.x * speed_scale) + (twist.angular.z * self.rotation_scale)

        # Maintain ratio of left/right in saturation
        if fabs(cmd.left) > 1.0:
            cmd.right = cmd.right * 1.0 / fabs(cmd.left)
            cmd.left = copysign(1.0, cmd.left)
        if fabs(cmd.right) > 1.0:
            cmd.left = cmd.left * 1.0 / fabs(cmd.right)
            cmd.right = copysign(1.0, cmd.right)

        # Apply down-scale of left and right thrusts.
        cmd.left *= self.left_max
        cmd.right *= self.right_max

        left = cmd.left
        right = cmd.right
        cmd.left = self.state_left.update(self,cmd.left)
        cmd.right = self.state_right.update(self,cmd.right)
        # rospy.loginfo("Twist left : state %+d stamp %.3f cmd %.1f -> %.1f" % \
        #        (self.state_left.state,self.state_left.stamp.to_sec(),left,cmd.left))
        # rospy.loginfo("Twist right: state %+d stamp %.3f cmd %.1f -> %.1f" % \
        #        (self.state_right.state,self.state_right.stamp.to_sec(),right,cmd.right))

        self.cmd_pub.publish(cmd)




def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = KingfisherTwist()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

