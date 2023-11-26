 #!/usr/bin/env python

# Documentation - https://docs.ros.org/en/jade/api/rospy/html/rospy-module.html

import rospy
from std_msgs.msg import (
    String,
    ColorRGBA,
    Float32,
    Int32,
    Duration
)


class ControlSignalsBase:
    """
    Control Signals Base
    
    Listens to Commands from AI/Manaul Input and passes it onto the channel
    """
    
    def __init__(self, listener_name:str,  
                 channel_name:str,  
                 channel_dtype:str=String,
                 anonymous=False):
        
        self.listener_name = listener_name
        self.sensor_channel = channel_name
        self.annonymous = anonymous
        self.channel_dtype = channel_dtype  
        
    @staticmethod  
    def callback(data):
      rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
      
    
    def listener(self):
        rospy.init_node(self.listener_name, 
                        anonymous=self.annonymous)
        rospy.Subscriber(self.sensor_channel, self.channel_dtype, self.callback)
        
        

class MotorControllerBase:
    """
    Controls Motors
    
    """    
    
    def __init__(self, sensor_name:str, 
                 sensor_channel:str, 
                 sensor_data_type=String, 
                 anonymous=False,
                 queue_size=10, rate_hz=10):
        # creating publisher
        self.publisher = rospy.Publisher(sensor_channel, sensor_data_type, 
                                         queue_size=queue_size)
        # initialising node
        rospy.init_node(sensor_name, anonymous=anonymous)
        # setting comm hertz.
        self.rate = rospy.Rate(rate_hz)
        
    def write(self):
        """ Override this method"""
        while not rospy.is_shutdown():
            # Place holder code
           hello_str = "hello world %s" % rospy.get_time()
           rospy.loginfo(hello_str)
           self.pub.publish(hello_str)
           self.rate.sleep()
        