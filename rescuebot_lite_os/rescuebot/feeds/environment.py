
#Base on  
from absl import logging
from datetime import datetime
import logging
from datetime import datetime
from kombu import (
    Exchange, 
    Queue, 
    Producer, 
    Connection
)
from rescuebot.feeds.base import SensorReaderBase
from rescuebot.feeds.sensor_lib.environment import BME280

SENSORS_CHANNEL = "sensors"
RABBIT_MQ_SERVER_URI = "amqp://guest:guest@localhost:5672//"
TEMP_SENSOR_PIN = 7 

class EnvironmentSensorReader(SensorReaderBase):
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
            'temperature': raw_data[1],
            'humidity': raw_data[2],
            'pressure': raw_data[0] ,
            'timestamp': datetime.now().isoformat()
        }
        print(f"Environmental data written: {env_data}")
        logging.debug(f"Environmental data written: {env_data}")
        self.write(env_data)
        
