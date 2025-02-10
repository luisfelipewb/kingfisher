#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker, MarkerArray

import roslib; roslib.load_manifest('kingfisher_node')
from kingfisher_msgs.msg import Drive

class KFDriveViz:
    def __init__(self):
        rospy.init_node('kf_drive_viz')
        self.base_frame = rospy.get_param("~base_frame","base_link")
        self.subDrive = rospy.Subscriber("~cmd_drive",Drive, self.drive_cb, queue_size=1)
        self.pub = rospy.Publisher("~marker",MarkerArray,queue_size=1)
        self.left = 0
        self.right = 0
    
    def publish(self,left,right):
        ma=MarkerArray()
        ml = Marker()
        mr = Marker()
        ml.header.stamp = rospy.Time()
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
        ml.scale.x = left
        ml.scale.y = 0.1
        ml.scale.z = 0.1
        ml.color.a = 1.0 
        ml.color.r = 1.0
        ml.color.g = 0.0
        ml.color.b = 0.0

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
        mr.scale.x = right
        mr.scale.y = 0.1
        mr.scale.z = 0.1
        mr.color.a = 1.0 
        mr.color.r = 0.0
        mr.color.g = 1.0
        mr.color.b = 0.0
        ma.markers.append(ml)
        ma.markers.append(mr)
        self.pub.publish(ma)

    def drive_cb(self, msg):
        self.left = msg.left
        self.right = msg.right
        self.publish(self.left,self.right)

if __name__ == '__main__':
    try:
        kd = KFDriveViz() 
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


