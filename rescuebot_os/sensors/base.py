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


class SensorWriterBase:
    """
    Sensor Writter
    
    Writes Sensor Data for other down stream tasks to use.
    
    """    
    
    def __init__(self, sensor_name:str, 
                 sensor_channel:str, 
                 sensor_data_type=String, 
                 anonymous=False,
                 queue_size=10, rate_hz=10):
        self.publisher = rospy.Publisher(sensor_channel, sensor_data_type, 
                                         queue_size=queue_size)
        rospy.init_node(sensor_name, anonymous=anonymous)
        
        self.rate = rospy.Rate(rate_hz)
        
    def write(self):
        """ Override this method"""
        while not rospy.is_shutdown():
           hello_str = "hello world %s" % rospy.get_time()
           rospy.loginfo(hello_str)
           self.pub.publish(hello_str)
           self.rate.sleep()
        
        

class SensorListenerBase:
    """
    Sensor Reader
    
    Reads Sensor Data from Sensor Writter
    """
    
    def __init__(self, listener_name:str,  channel_name:str, anonymous=False):
        
        
    def callback(self):
        pass
    
    
    def listener(self):
        pass
        
        

        
        
