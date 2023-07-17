#!/usr/bin/python

import rospy 

from geometry_msgs.msg import Twist
from kingfisher_msgs.msg import Drive
from dynamic_reconfigure.server import Server
from kingfisher_node.cfg import TwistConfig
from math import fabs, copysign
from std_msgs.msg import Float32

class SignControl:
    def __init__(self,dead_time=0.25):
        self.dead_time = dead_time
        self.state = 0
        self.stamp = rospy.Time.now()

    def set_dead_time(self, dead_time):
        self.dead_time = dead_time

    def update(self,value):
        now = rospy.Time.now()
        if self.state == -1:
            if value < -0.01:
                return value
            else:
                self.state = 0
                self.stamp = now
                return 0.0
        if self.state == 0:
            if (value < -0.01) and ((now-self.stamp).to_sec() > self.dead_time):
                self.state = -1
                self.stamp = now
                return value
            if (value >  0.01) and ((now-self.stamp).to_sec() > self.dead_time):
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

class TwistSubscriber:
    def __init__(self):
        self.rotation_scale = rospy.get_param('~rotation_scale', 0.2)
        self.speed_scale = rospy.get_param('~speed_scale', 1.0)

        self.state_left = SignControl(0.5)
        self.state_right = SignControl(0.5)
        self.cmd_pub = rospy.Publisher('cmd_drive', Drive, queue_size=1)
        rospy.Subscriber("cmd_vel", Twist, self.callback) 

        srv = Server(TwistConfig, self.reconfigure)

    def reconfigure(self, config, level):
        self.rotation_scale = config['rotation_scale']
        self.fwd_speed_scale = config['fwd_speed_scale']
        self.rev_speed_scale = config['rev_speed_scale']
        self.left_max = config['left_max']
        self.right_max = config['right_max']
        self.state_left.set_dead_time(config['dead_time'])
        self.state_right.set_dead_time(config['dead_time'])
        return config


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
        cmd.left = self.state_left.update(cmd.left)
        cmd.right = self.state_right.update(cmd.right)
        # rospy.loginfo("Twist left : state %+d stamp %.3f cmd %.1f -> %.1f" % \
        #        (self.state_left.state,self.state_left.stamp.to_sec(),left,cmd.left))
        # rospy.loginfo("Twist right: state %+d stamp %.3f cmd %.1f -> %.1f" % \
        #        (self.state_right.state,self.state_right.stamp.to_sec(),right,cmd.right))

        self.cmd_pub.publish(cmd)
