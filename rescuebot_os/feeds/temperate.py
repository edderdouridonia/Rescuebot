
import rospy
from std_msgs.msg import (
    String,
    ColorRGBA,
    Float32,
    Int32,
    Duration
)

import numpy as np

from rescuebot_os.feeds.base import SensorWriterBase  


class TemperatureSensor:
    """
    Sensor Writter
    
    Writes Sensor Data for other down stream tasks to use.
    
    """    
    
    def __init__(self, sensor_name:str='temp', 
                 sensor_channel:str='robot_temp', 
                 sensor_data_type=Float32, 
                 anonymous=True,
                 queue_size=10, rate_hz=10):
        
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
           temp_dummy_value = np.random.rand()
           rospy.loginfo(f"Temperature: {temp_dummy_value}")
           self.pub.publish(temp_dummy_value)
           self.rate.sleep()
           
           
    