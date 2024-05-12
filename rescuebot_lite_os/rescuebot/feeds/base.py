
import os
from kombu import (
    Connection, 
    Exchange, 
    Producer,
    Consumer,
    Queue,
    eventloop
)
from pprint import pformat
from absl import logging


logging.set_verbosity(logging.DEBUG)

SENSORS_CHANNEL = "sensor_data"
# GET environment variables
RABBIT_MQ_SERVER_URI = os.environ.get("RABBIT_MQ_SERVER_URI", "amqp://guest:guest@localhost:5672//")

class SensorReaderBase:
    """
    Sensor Writter (Template class for sensors)
    
    Writes Sensor Data for other down stream tasks to use.
    
    """    
    
    def __init__(self, sensor_name:str, 
                 sensor_channel:str):
        # creating publisher
        self.sensor_name = sensor_name
        self.sensor_channel = sensor_channel
        self.exchange = Exchange(SENSORS_CHANNEL, type='direct')
        self.queue = Queue(sensor_channel, self.exchange, routing_key=sensor_channel)
        logging.debug(f"Sensor {sensor_channel} is ready to write to {sensor_channel}!")
        
    def write(self, sensor_reading: dict):
        """ 
        Writes sensor readings to the class
        """
        with Connection(RABBIT_MQ_SERVER_URI) as connection:
             producer = Producer(connection)
             producer.publish(
               sensor_reading,
                exchange=self.exchange,
                routing_key=self.sensor_channel,
                serializer='json',
                compression='zlib'
            )
        

class SensorReadingListener:
    """
    Sensor Reader (Listens for sensor readings)
    
    Reads Sensor Data from Sensor Writter
    """
    
    def __init__(self,
                 sensor_channel:str):
        self.exchange = Exchange(SENSORS_CHANNEL, type='direct')
        self.queue = Queue(sensor_channel, self.exchange, routing_key=sensor_channel)
        
    @staticmethod
    def pretty(obj):
        return pformat(obj, indent=4)
    
    def handle(self,body, message):
        logging.debug(f'Received message: {body!r}')
        print(f'Received message: {body!r}')
        message.ack()
    
    def listen(self):
        """
        Listens for sensor readings and handles them
        """
        with Connection(RABBIT_MQ_SERVER_URI) as connection:

            with Consumer(connection, self.queue, callbacks=[self.handle]):
                for _ in eventloop(connection):
                    pass
        