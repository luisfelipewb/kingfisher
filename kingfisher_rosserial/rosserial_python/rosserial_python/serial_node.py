#!/usr/bin/python3

import sys
import os

import importlib
import rclpy
from rclpy.node import Node
from rclpy.time import Time


import threading
from serial import *
from io import BytesIO

import std_msgs.msg # Bool
import rosserial_msgs
import rosserial_msgs.msg # Log, TopicInfo
import rosserial_msgs.srv # RequestParam
import kingfisher_msgs
from kingfisher_msgs.msg import Drive, Sense

import pathlib, rosserial_python 
rosserial_path = os.path.dirname(rosserial_python.__file__)
# Ugly hack to load the old modules
sys.path.insert(0,rosserial_path)

import TopicInfo,Log
import Time
import RequestParam
import Drive, Sense
import Bool
import Float32
# Dictionnary: microcontroller message type -> (Python class, ROS2 class, f(msg1,msg2) converter function)
# f will convert msg1 from the ROS1 python class to ROS2 if its second argument is None, and the ROS2 class to ROS1 if the first argument is None.
# If f is None, a default function will do a attribute-wise copy of the structure.
known_types={'kingfisher_msgs/Drive':(Drive.Drive,kingfisher_msgs.msg.Drive,None),
        'kingfisher_msgs/Sense':(Sense.Sense,kingfisher_msgs.msg.Sense,None) ,
        'rosserial_msgs/Log':(Log.Log,rosserial_msgs.msg.Log,None), 
        'rosserial_msgs/TopicInfo':(TopicInfo.TopicInfo,rosserial_msgs.msg.TopicInfo,None), 
        'rosserial_msgs/RequestParam':(RequestParam.RequestParam,rosserial_msgs.srv.RequestParam,None), # Service
        'std_msgs/Bool':(Bool.Bool,std_msgs.msg.Bool,None) ,
        'std_msgs/Float32':(Float32.Float32,std_msgs.msg.Float32,None) 
        }


import time
import struct


def from_ros2(msg,message_type):
    msg1 = known_types[message_type][0]()
    if known_types[message_type][2] is None:
        for d in msg1.__slots__:
            msg1.__setattr__(d,msg.__getattribute__("_"+d))
        return msg1
    else:
        return known_types[message_type][2](None,msg)


def to_ros2(msg,message_type):
    msg2 = known_types[message_type][1]()
    if known_types[message_type][2] is None:
        for d in msg.__slots__:
            msg2.__setattr__("_"+d,msg.__getattribute__(d))
        return msg2
    else:
        return known_types[message_type][2](msg,None)

class Publisher:
    """ 
        Prototype of a forwarding publisher.
    """
    def __init__(self, node, topic, message_type):
        """ Create a new publisher. """ 
        self.topic = topic
        self.message_type = message_type
        self.publisher = node.create_publisher(known_types[self.message_type][1], topic, 1)
    
    def handlePacket(self, data):
        """ """
        m = known_types[self.message_type][0]()
        m.deserialize(data)
        self.publisher.publish(to_ros2(m,self.message_type))


class Subscriber:
    """ 
        Prototype of a forwarding subscriber.
    """

    def __init__(self, node, topic, topic_id, message_type):
        self.topic = topic
        self.message_type = message_type
        self.topic_id = topic_id
        self.node = node
        
        self.subscriber = node.create_subscription(known_types[self.message_type][1],topic,self.callback,1)

    def callback(self, msg):
        """ Forward a message """
        data_buffer = BytesIO()
        msg = from_ros2(msg, self.message_type)
        msg.serialize(data_buffer)
        self.node.send(self.topic_id, data_buffer.getbuffer())


class SerialClient(Node):
    def __init__(self):
        super().__init__("rosserial_client")
        self.declare_parameter('~/port', "/dev/ttyACM0")
        self.declare_parameter('~/baud', 57600)
        self.declare_parameter('~/timeout', 5.0)

        port=self.get_parameter("~/port").get_parameter_value().string_value
        baud=self.get_parameter("~/baud").get_parameter_value().integer_value
        self.timeout=self.get_parameter("~/timeout").get_parameter_value().double_value

        """ Initialize node, connect to bus, attempt to negotiate topics. """
        self.mutex = threading.Lock()

        self.lastsync = self.get_clock().now()

        # open a specific port
        self.port = Serial(port, baud, timeout=self.timeout*0.5)
        
        self.port.timeout = 0.050 #edit the port timeout
        
        self.senders = dict() #Publishers/ServiceServers
        self.receivers = dict() #subscribers/serviceclients

        time.sleep(1.0) 
        self.requestTopics()
        
    def requestTopics(self):
        """ Determine topics to subscribe/publish. """
        self.port.flushInput()
        # request topic sync
        self.port.write(bytes("\xff\xff\x00\x00\x00\x00\xff",encoding='latin1'))

    def run(self):
        """ Forward recieved messages to appropriate publisher. """
        data = ''
        while rclpy.ok():
            rclpy.spin_once(self,timeout_sec=0.005)
            now = self.get_clock().now()
            if (now - self.lastsync).nanoseconds/1e9 > (self.timeout * 3):
                self.get_logger().error("Lost sync with device, restarting...")
                self.requestTopics()
                self.lastsync = now
            
            flag = [0,0]
            flag[0]  = self.port.read(1)
            if len(flag[0])==0:
                continue
            if (flag[0] != b'\xff'):
                self.get_logger().info("Failed Packet Flags 0 (%d bytes)" % len(flag[0]))
                continue
            flag[1] = self.port.read(1)
            if (len(flag[1])==0) or (flag[1] != b'\xff'):
                self.get_logger().info("Failed Packet Flags 1")
                continue
            # topic id (2 bytes)
            header = self.port.read(4)
            if (len(header) != 4):
                #self.port.flushInput()
                self.get_logger().info("Failed Packet Header")
                continue
            
            topic_id, msg_length = struct.unpack("<hh", header)
            msg = self.port.read(msg_length)
            # print("Msg: "+str(["%02X"%x for x in msg]))
            if (len(msg) != msg_length):
                self.get_logger().info("Packet Failed :  Failed to read msg data")
                #self.port.flushInput()
                continue
            chk = self.port.read(1)
            checksum = sum(map(int,header) ) + sum(map(int, msg)) + sum(map(int, chk))

            if checksum%256 == 255:
                if topic_id == TopicInfo.TopicInfo.ID_PUBLISHER:
                    try:
                        m = TopicInfo.TopicInfo()
                        m.deserialize(msg)
                        if m.message_type in known_types:
                            self.senders[m.topic_id] = Publisher(self,m.topic_name,m.message_type)
                            self.get_logger().info("Setup Publisher on %s [%s]" % (m.topic_name, m.message_type) )
                        else:
                            self.get_logger().warn("Cannot create publisher on %s [%s]: type not managed" % (m.topic_name, m.message_type) )
                    except Exception as e:
                        self.get_logger().error("Failed to parse publisher: %s" % str(e))
                elif topic_id == TopicInfo.TopicInfo.ID_SUBSCRIBER:
                    try:
                        m = TopicInfo.TopicInfo()
                        m.deserialize(msg)
                        if m.message_type in known_types:
                            self.receivers[m.topic_id] = Subscriber(self,m.topic_name,m.topic_id,m.message_type)
                        self.get_logger().info("Setup Subscriber on %s [%s]" % (m.topic_name, m.message_type))
                    except Exception as e:
                        self.get_logger().error("Failed to parse subscriber. %s"%str(e))
                elif topic_id == TopicInfo.TopicInfo.ID_SERVICE_SERVER:
                    # try:
                    #     m = TopicInfo.TopicInfo()
                    #     m.deserialize(msg)
                    #     self.senders[m.topic_id]=ServiceServer(self, m.topic_name, m.message_type, self) 
                    #     self.get_logger().info("Setup ServiceServer on %s [%s]"%(m.topic_name, m.message_type) )
                    # except:
                    #    self.get_logger().error("Failed to parse service server")
                    self.get_logger().warn("Not implemented: service server request: %s [%s]" % (m.topic_name, m.message_type))
                elif topic_id == TopicInfo.TopicInfo.ID_SERVICE_CLIENT:
                    self.get_logger().warn("Not implemented: service client request: %s [%s]" % (m.topic_name, m.message_type))

                elif topic_id == TopicInfo.TopicInfo.ID_PARAMETER_REQUEST:
                    self.handleParameterRequest(msg)
                
                elif topic_id == TopicInfo.TopicInfo.ID_LOG:
                    self.handleLogging(msg)
                    
                elif topic_id == TopicInfo.TopicInfo.ID_TIME:
                    t = Time.Time()
                    now = self.get_clock().now()
                    t.data.secs = int(now.nanoseconds//1e9)
                    t.data.nsecs = int(now.nanoseconds - t.data.secs*1e9)
                    data_buffer = BytesIO()
                    t.serialize(data_buffer)
                    self.send( TopicInfo.TopicInfo.ID_TIME, data_buffer.getbuffer() )
                    self.lastsync = now
                    self.get_logger().debug("Got sync message")
                elif topic_id >= 100: # TOPIC
                    try:
                        self.senders[topic_id].handlePacket(msg)
                    except KeyError:
                        self.get_logger().error("Tried to publish before configured, topic id %d" % topic_id)
                else:
                    self.get_logger().error("Unrecognized command topic %d !" % topic_id)
            else:
                self.get_logger().error("Invalid checksum !")


    def handleParameterRequest(self,data):
        """Handlers the request for parameters from the rosserial_client
            This is only serves a limmited selection of parameter types.
            It is meant for simple configuration of your hardware. It 
            will not send dictionaries or multitype lists.
        """
        req = RequestParamRequest()
        req.deserialize(data)
        self.get_logger().warn("Not implemented: parameter request: %s " % (req.name))
        return

        # resp = RequestParamResponse()
        # 
        # param = rospy.get_param(req.name) # TODO
        # if param == None:
        #     self.get_logger().error("Parameter %s does not exist"%req.name)
        #     return
        # if (type(param) == dict):
        #     self.get_logger().error("Cannot send param %s because it is a dictionary"%req.name)
        #     return
        # if (type(param) != list):
        #     param = [param]
        # #check to make sure that all parameters in list are same type
        # t = type(param[0])
        # for p in param:
        #     if t!= type(p):
        #         self.get_logger().error('All Paramers in the list %s must be of the same type'%req.name)
        #         return      
        # if (t == int):
        #     resp.ints= param
        # if (t == float):
        #     resp.floats=param
        # if (t == str):
        #     resp.strings = param
        # print(str(resp))
        # data_buffer = BytesIO()
        # resp.serialize(data_buffer)
        # self.send(TopicInfo.ID_PARAMETER_REQUEST, data_buffer.getbuffer())

    def handleLogging(self, data):
        m= Log()
        m.deserialize(data)
        if (m.level == Log.DEBUG):
            self.get_logger().debug(m.msg)
        elif(m.level== Log.INFO):
            self.get_logger().info(m.msg)
        elif(m.level== Log.WARN):
            self.get_logger().warn(m.msg)
        elif(m.level== Log.ERROR):
            self.get_logger().error(m.msg)
        elif(m.level==Log.FATAL):
            self.get_logger().fatal(m.msg)
        
    def encode(self, topic, msg):
        """ Encode a message on a particular topic. """
        length = len(msg)
        checksum = 255 - ( ((topic&255) + (topic>>8) + (length&255) + (length>>8) + sum(map(int,msg)))%256 )
        buf = BytesIO()
        buf.write(b'\xff\xff')
        buf.write(topic.to_bytes(2,byteorder="little"))
        buf.write(length.to_bytes(2,byteorder="little"))
        buf.write(msg)
        buf.write(checksum.to_bytes(1,byteorder="little"))
        return buf.getbuffer()

    def send(self, topic, msg):
        """ Send a message on a particular topic to the device. """
        with self.mutex:
            data = self.encode(topic,msg)
            self.port.write(data)





def main(args=None):
    rclpy.init(args=args)

    client = SerialClient()

    client.run()

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
