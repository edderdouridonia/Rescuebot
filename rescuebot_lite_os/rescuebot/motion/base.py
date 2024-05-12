import os
import time
from kombu import (
    Connection, 
    Exchange, 
    Producer,
    Consumer,
    Queue,
    eventloop
)
import RPi.GPIO as GPIO
from pprint import pformat
from absl import logging

# Set logging level
logging.set_verbosity(logging.DEBUG)

# Channel for sensor data
SENSORS_CHANNEL = "motion_data"
# Environment variable for RabbitMQ URI
RABBIT_MQ_SERVER_URI = os.environ.get("RABBIT_MQ_SERVER_URI", "amqp://guest:guest@localhost:5672//")
        

class MotorController:
    def __init__(self, motor_pin):
        """
        Initializes the MotorController with pin assignments.
        :param motor1_pins: A tuple of (IN1, IN2) for motor.
       
        """
        self.motor_pins = motor_pin

        # Setup the GPIO pins
        GPIO.setmode(GPIO.BCM)  # Use Broadcom pin numbering
        GPIO.setup(self.motor_pins, GPIO.OUT)

        # Initialize pins to low
        GPIO.output(self.motor_pins, GPIO.LOW)
        

    def move_forward(self, duration_ms:int):
        """
        Drives both motors to move forward for a specified duration.
        :param duration: Time in seconds to move forward.
        """
        GPIO.output(self.motor_pins, (GPIO.HIGH, GPIO.LOW))
       
        time.sleep(duration_ms)
        self.stop()


    def move_backward(self, duration_ms:int):
        """
        Drives both motors to move backward for a specified duration.
        :param duration: Time in seconds to move backward.
        """
        GPIO.output(self.motor_pins, (GPIO.LOW, GPIO.HIGH))
        
        time.sleep(duration_ms)
        self.stop()

    def stop(self):
        """
        Stops both motors.
        """
        GPIO.output(self.motor_pins, GPIO.LOW)
     

    def cleanup(self):
        """
        Cleans up GPIO resources. It's a good practice to call this method before exiting the program.
        """
        GPIO.cleanup()


class MotionCommandBase:
    """
    Movement Base 
    
    Listens for movement commands and handles them accordingly.
    """
    
    def __init__(self, sensor_channel:str, motion_controller:MotorController):
        # Initialize the base class with the specific channel for movement commands
        self.exchange = Exchange(SENSORS_CHANNEL, type='direct')
        self.queue = Queue(sensor_channel, self.exchange, routing_key=sensor_channel)
        self.motion_controller = motion_controller
        
    @staticmethod
    def pretty(obj):
        return pformat(obj, indent=4)
    
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
    listener = MotionBase()
    # Start listening for movement commands
    listener.listen()
