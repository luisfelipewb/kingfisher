#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from visualization_msgs.msg import Marker,MarkerArray
from kingfisher_msgs.msg import Drive


class KingfisherViz(Node):

    def __init__(self):
        super().__init__('kingfisher_viz')
        self.declare_parameter('base_frame', "base_link")

        self.base_frame=self.get_parameter("base_frame").get_parameter_value().string_value

        self.marker_pub = self.create_publisher(MarkerArray,'~/marker', 1)
        self.drv_sub = self.create_subscription(Drive,"cmd_drive", self.callback,1) 


    def callback(self, msg):
        self.left = msg.left
        self.right = msg.right
        self.publish(self.left,self.right)

    # RViz warns "Scale of 0 in one of x/y/z" on a zero scale component, and an
    # idle boat publishes cmd_drive 0.0 continuously. Keep the arrow length off
    # zero; the sign is passed through untouched.
    MIN_ARROW_LENGTH = 1e-3

    def arrow_length(self, thrust):
        if abs(thrust) < self.MIN_ARROW_LENGTH:
            return self.MIN_ARROW_LENGTH
        return thrust

    def publish(self,left,right):
        lifetime = rclpy.duration.Duration(seconds=3.0).to_msg()
        now = self.get_clock().now()
        ma=MarkerArray()
        ml = Marker()
        mr = Marker()
        ml.header.stamp = now.to_msg()
        ml.header.frame_id = self.base_frame
        ml.ns = "kf_drive"
        ml.id = 0
        ml.type = Marker.ARROW
        ml.action = Marker.ADD
        ml.pose.position.x = -0.5
        ml.pose.position.y = 0.49
        ml.pose.position.z = 0.
        ml.pose.orientation.x = 0.7071067811865475
        ml.pose.orientation.y = 0.0
        ml.pose.orientation.z = 0.0
        ml.pose.orientation.w = 0.7071067811865475
        ml.scale.x = self.arrow_length(left)
        ml.scale.y = 0.1
        ml.scale.z = 0.1
        ml.color.a = 1.0 
        ml.color.r = 1.0
        ml.color.g = 0.0
        ml.color.b = 0.0
        ml.lifetime = lifetime

        mr.header.stamp = ml.header.stamp
        mr.header.frame_id = self.base_frame
        mr.ns = "kf_drive"
        mr.id = 1
        mr.type = Marker.ARROW
        mr.action = Marker.ADD
        mr.pose.position.x = -0.5
        mr.pose.position.y = -0.49
        mr.pose.position.z = 0.
        mr.pose.orientation.x = 0.7071067811865475
        mr.pose.orientation.y = 0.0
        mr.pose.orientation.z = 0.0
        mr.pose.orientation.w = 0.7071067811865475
        mr.scale.x = self.arrow_length(right)
        mr.scale.y = 0.1
        mr.scale.z = 0.1
        mr.color.a = 1.0 
        mr.color.r = 0.0
        mr.color.g = 1.0
        mr.color.b = 0.0
        mr.lifetime = lifetime
        ma.markers.append(ml)
        ma.markers.append(mr)
        self.marker_pub.publish(ma)




def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = KingfisherViz()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

