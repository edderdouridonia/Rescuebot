
#Base on  
import logging
from datetime import datetime
import board
import busio


from kombu import (
    Exchange, 
    Queue, 
    Producer, 
    Connection
)
from rescuebot.feeds.base import SensorReaderBase
from rescuebot.feeds.sensor_lib.uv import LTR390
from rescuebot.feeds.sensor_lib.luminosity import TSL2591


# Constants for RabbitMQ
SENSORS_CHANNEL = "sensors"
RABBIT_MQ_SERVER_URI = "amqp://guest:guest@localhost:5672//"


class UVSensorReader(SensorReaderBase):
    """
    Specialized writer for the LTR-390-UV-1 Light Sensor.
    """
    def __init__(self, sensor_name, sensor_channel):
        super().__init__(sensor_name, sensor_channel)
        self.sensor = LTR390()
        logging.debug(f"{sensor_name} using LTR-390-UV-1 sensor initialized on channel {sensor_channel}.")

    def write_light_data(self):
        """
        Reads light data from the LTR-390-UV-1 and writes it with a timestamp.
        """
        
        uv_index = self.sensor.UVS()
       
        uv_data = {
            'uv_index': uv_index,
            'timestamp': datetime.now().isoformat()
        }
        self.write(uv_data)
        logging.info(f"UV data written: {uv_data}")
        
        
class LuminositySensorReader(SensorReaderBase):
    """
    Specialized writer for the TSL25911FN Lux Sensor.
    """
    def __init__(self, sensor_name, sensor_channel):
        super().__init__(sensor_name, sensor_channel)
        self.sensor = TSL2591()
        logging.debug(f"{sensor_name} using TSL25911FN sensor initialized on channel {sensor_channel}.")

    def write_lux_data(self):
        """
        Reads lux data from the TSL25911FN and writes it with a timestamp.
        """
        lux = self.sensor.Lux()
        full_spectrum = self.sensor.full_spectrum
        infrared = self.sensor.infrared
        visible = self.sensor.visible
        lux_data = {
            'lux': lux,
            'full_spectrum': full_spectrum,
            'infrared': infrared,
            'visible': visible,
            'timestamp': datetime.now().isoformat()
        }
        self.write(lux_data)
        logging.info(f"Lux data written: {lux_data}")
        