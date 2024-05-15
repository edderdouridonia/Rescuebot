import logging
from datetime import datetime


from kombu import Exchange, Queue, Producer, Connection
from rescuebot.feeds.base import SensorReaderBase
from rescuebot.feeds.sensor_lib.gas import SGP40

# Constants for RabbitMQ
SENSORS_CHANNEL = "sensors"
RABBIT_MQ_SERVER_URI = "amqp://guest:guest@localhost:5672//"
AVG_TEMP = 25
AVG_HUMIDITY = 50

class AirQualitySensorReader(SensorReaderBase):
    """
    SGP40 - Compatible
    Specialized writer for the SGP40 VOC Sensor.
    """
    def __init__(self, sensor_name, sensor_channel):
        super().__init__(sensor_name, sensor_channel)
        self.sensor = SGP40()
        logging.debug(f"{sensor_name} using SGP40 sensor initialized on channel {sensor_channel}.")

    def write_voc_data(self, temperature=AVG_TEMP, humidity=AVG_HUMIDITY):
        """
        Reads VOC data from the SGP40 and writes it with a timestamp.
        """
        # TODO: Make this use the actual temperature readings 
        gas_spg_reading = self.sensor.measureRaw(temperature=temperature, 
                                                 humidity=humidity)
        voc_data = {
            'gas_spg': gas_spg_reading,
            'timestamp': datetime.now().isoformat()
        }
        print(voc_data)
        self.write(voc_data)
        logging.info(f"VOC data written: {voc_data}")
        
        
