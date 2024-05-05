
#Base on  
import logging
from datetime import datetime
import board
import busio
from adafruit_ltr390 import LTR390
from adafruit_tsl2591 import TSL2591

from kombu import (
    Exchange, 
    Queue, 
    Producer, 
    Connection
)
from rescuebot.feeds.base import SensorBaseWritter


# Constants for RabbitMQ
SENSORS_CHANNEL = "sensors"
RABBIT_MQ_SERVER_URI = "amqp://guest:guest@localhost:5672//"


class UVSensorWriter(SensorBaseWritter):
    """
    Specialized writer for the LTR-390-UV-1 Light Sensor.
    """
    def __init__(self, sensor_name, sensor_channel):
        super().__init__(sensor_name, sensor_channel)
        i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = LTR390(i2c)
        logging.debug(f"{sensor_name} using LTR-390-UV-1 sensor initialized on channel {sensor_channel}.")

    def write_light_data(self):
        """
        Reads light data from the LTR-390-UV-1 and writes it with a timestamp.
        """
        uv_index = self.sensor.uvs
        ambient_light = self.sensor.light
        light_data = {
            'uv_index': uv_index,
            'ambient_light': ambient_light,
            'timestamp': datetime.now().isoformat()
        }
        self.write(light_data)
        logging.info(f"Light data written: {light_data}")
        
        
class LightSensorWriter(SensorBaseWritter):
    """
    Specialized writer for the TSL25911FN Lux Sensor.
    """
    def __init__(self, sensor_name, sensor_channel):
        super().__init__(sensor_name, sensor_channel)
        i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = TSL2591(i2c)
        logging.debug(f"{sensor_name} using TSL25911FN sensor initialized on channel {sensor_channel}.")

    def write_lux_data(self):
        """
        Reads lux data from the TSL25911FN and writes it with a timestamp.
        """
        lux = self.sensor.lux
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
        