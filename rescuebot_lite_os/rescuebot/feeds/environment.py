
#Base on  
import logging
from datetime import datetime
import logging
from datetime import datetime
from adafruit_bme280 import basic as adafruit_bme280
import board
import busio
from kombu import (
    Exchange, 
    Queue, 
    Producer, 
    Connection
)
from rescuebot.feeds.base import SensorBaseWritter

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
        i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_bme280.Adafruit_BME280_I2C(i2c)
        self.sensor.sea_level_pressure = sea_level_pressure
        logging.debug(f"{sensor_name} using BME280 sensor initialized on channel {sensor_channel}.")

    def write_environment_data(self):
        """
        Reads environmental data and writes it with a timestamp.
        """
        env_data = {
            'temperature': self.sensor.temperature,
            'humidity': self.sensor.humidity,
            'pressure': self.sensor.pressure,
            'timestamp': datetime.now().isoformat()
        }
        self.write(env_data)
        logging.info(f"Environmental data written: {env_data}")
    