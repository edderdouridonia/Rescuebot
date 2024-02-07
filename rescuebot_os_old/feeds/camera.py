
import rospy
from std_msgs.msg import (
    String,
    ColorRGBA,
    Float32,
    Int32,
    Duration
)

import numpy as np

from rescuebot_os_old.feeds.base import SensorBase  


class CameraFeed(SensorBase):
    """
    Sensor Writter
    
    Writes Sensor Data for other down stream tasks to use.
    
    """    
    
    def __init__(self, sensor_name:str='temp', 
                 sensor_channel:str='robot_temp', 
                 sensor_data_type=ColorRGBA, 
                 anonymous=True,
                 queue_size=10, 
                 rate_hz=10):
        """ Camerate feed intialisation"""
        super().__init__(sensor_name=sensor_name, 
                         sensor_channel=sensor_channel,
                        sensor_data_type=sensor_data_type, 
                         anonymous=anonymous,
                         queue_size=queue_size,
                         rate_hz=rate_hz)
        
    def read_from_sensor(self):
        """ Override this method"""
        while not rospy.is_shutdown():
           # TODO: Replace with code to read from sensor and write to topic
           temp_dummy_value = np.random.randn((400,400))
           rospy.loginfo(f"Image: {temp_dummy_value}")
           self.pub.publish(temp_dummy_value)
           self.rate.sleep()
           
           
    