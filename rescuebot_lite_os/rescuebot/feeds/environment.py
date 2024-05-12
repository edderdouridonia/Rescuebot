
#Base on  
import logging
from datetime import datetime
import logging
from datetime import datetime
from  import basic as adafruit_bme280
import board
import busio
from kombu import (
    Exchange, 
    Queue, 
    Producer, 
    Connection
)
from rescuebot.feeds.base import SensorBaseWritter
from rescuebot.feeds.sensor_lib.BME280 import BME280

SENSORS_CHANNEL = "sensors"
RABBIT_MQ_SERVER_URI = "amqp://guest:guest@localhost:5672//"
TEMP_SENSOR_PIN = 7 


class EnvironmentSensorWriter(SensorBaseWritter):
    """
    BME280 
    Specialized writer for an Environment Sensor HAT.
    """
    def __init__(self, sensor_name:str, sensor_channel:str, sea_level_pressure:float=1013.25):
        super().__init__(sensor_name, sensor_channel)
        self.sensor = BME280()
        self.sensor.get_calib_param()
        self.sea_level_pressure = sea_level_pressure
        logging.debug(f"{sensor_name} using BME280 sensor initialized on channel {sensor_channel}.")

    def write_environment_data(self):
        """
        Reads environmental data and writes it with a timestamp.
        """
        raw_data = self.sensor.readData()

        env_data = {
            'temperature': raw_data.data[1],
            'humidity': raw_data.data[2],
            'pressure': raw_data.data[0] ,
            'timestamp': datetime.now().isoformat()
        }
        self.write(env_data)
        logging.info(f"Environmental data written: {env_data}")
    