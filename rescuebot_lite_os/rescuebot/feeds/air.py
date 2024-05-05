import logging
from datetime import datetime
import board
import busio
from adafruit_sgp40 import SGP40

from kombu import Exchange, Queue, Producer, Connection
from rescuebot.feeds.base import SensorBaseWritter

# Constants for RabbitMQ
SENSORS_CHANNEL = "sensors"
RABBIT_MQ_SERVER_URI = "amqp://guest:guest@localhost:5672//"


class AirQualitySensorWriter(SensorBaseWritter):
    """
    SGP40 - Compatible
    Specialized writer for the SGP40 VOC Sensor.
    """
    def __init__(self, sensor_name, sensor_channel):
        super().__init__(sensor_name, sensor_channel)
        i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = SGP40(i2c)
        logging.debug(f"{sensor_name} using SGP40 sensor initialized on channel {sensor_channel}.")

    def write_voc_data(self):
        """
        Reads VOC data from the SGP40 and writes it with a timestamp.
        """
        voc_index = self.sensor.raw
        voc_data = {
            'voc_index': voc_index,
            'timestamp': datetime.now().isoformat()
        }
        self.write(voc_data)
        logging.info(f"VOC data written: {voc_data}")
        
        