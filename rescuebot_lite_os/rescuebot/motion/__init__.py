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

# Set logging level
logging.set_verbosity(logging.DEBUG)

# Channel for sensor data
SENSORS_CHANNEL = "sensor_data"
# Environment variable for RabbitMQ URI
RABBIT_MQ_SERVER_URI = os.environ.get("RABBIT_MQ_SERVER_URI", "amqp://guest:guest@localhost:5672//")

class MovementSensorListener(SensorBaseListener):
    """
    Movement Sensor Listener
    
    Listens for movement commands and handles them accordingly.
    """
    
    def __init__(self):
        # Initialize the base class with the specific channel for movement commands
        super().__init__("movement_commands")
        
    def handle(self, body, message):
        """
        Override the handle method to process movement commands.
        """
        # Logging the received movement command
        logging.debug(f'Received movement command: {self.pretty(body)}')
        print(f'Received movement command: {self.pretty(body)}')
        # Acknowledge the message
        message.ack()

        # Here, you can add your logic to process the movement command.
        # For example, you might want to translate the command into specific
        # actions for a robot or a software simulation.

if __name__ == "__main__":
    # Create an instance of the MovementSensorListener
    listener = MovementSensorListener()
    # Start listening for movement commands
    listener.listen()
