
import rospy
from std_msgs.msg import (
    String,
    ColorRGBA,
    Float32,
    Int32,
    Duration
)

import numpy as np

from rescuebot_os.sensors.base import SensorWriterBase  


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
        self.publisher = rospy.Publisher(sensor_channel, sensor_data_type, 
                                         queue_size=queue_size)
        rospy.init_node(sensor_name, anonymous=anonymous)
        
        self.rate = rospy.Rate(rate_hz)
        
    def read_from_sensor(self):
        """ Override this method"""
        while not rospy.is_shutdown():
           # TODO: Replace with code to read from sensor and write to topic
           temp_dummy_value = np.random.rand()
           rospy.loginfo(f"Temperature: {temp_dummy_value}")
           self.pub.publish(temp_dummy_value)
           self.rate.sleep()
           
           
    