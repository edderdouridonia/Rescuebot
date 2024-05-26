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
from motoron import MotoronI2C

# Initialize Motoron on I2C bus 2
motoron = MotoronI2C(bus=2)
print("Initialized Motoron on I2C bus 2")

class MotorMotionCommand:
    """
    Motor Movement
    Listens for movement commands and handles them for motors  
    """
    def __init__(self, sensor_channel: str, motion_controller):
        self.sensor_channel = sensor_channel
        self.motion_controller = motion_controller
        print(f"MotorMotionCommand initialized for {sensor_channel}")
            
    def handle(self, body, message):
        """
        Handles movement command
        """
        print(f"Handling command: {body}")
        pass

class TrackMotionCommand(MotorMotionCommand):
    def __init__(self, sensor_channel: str, motor_pin: int):
        self.track_motor = motoron
        self.motor_pin = motor_pin
        super().__init__(sensor_channel=sensor_channel, motion_controller=self.track_motor)
        print(f"TrackMotionCommand initialized for {sensor_channel} with motor pin {motor_pin}")
        
    def handle(self, body, message):
        """
        Override the handle method to process movement commands.
        """
        logging.debug(f'Received movement command: {pformat(body)}')
        print(f'Received movement command: {pformat(body)}')
        
        if body["command"] == "forward":
            self.track_motor.set_speed(self.motor_pin, 100)
        elif body["command"] == "backward":
            self.track_motor.set_speed(self.motor_pin, -100)
        elif body["command"] == "stop":
            self.track_motor.set_speed(self.motor_pin, 0)
        
        message.ack()

class ArmMotionCommand(MotorMotionCommand):
    def __init__(self, sensor_channel: str, left_motor_pin: int, right_motor_pin: int):
        self.left_motor = motoron
        self.right_motor = motoron
        self.left_motor_pin = left_motor_pin
        self.right_motor_pin = right_motor_pin
        super().__init__(sensor_channel=sensor_channel, motion_controller=None)
        print(f"ArmMotionCommand initialized for {sensor_channel} with left motor pin {left_motor_pin} and right motor pin {right_motor_pin}")
        
    def handle(self, body, message):
        """
        Override the handle method to process movement commands.
        """
        logging.debug(f'Received movement command: {pformat(body)}')
        print(f'Received movement command: {pformat(body)}')
        
        if body["command"] == "left_arm_up":
            self.left_motor.set_speed(self.left_motor_pin, 100)
        elif body["command"] == "left_arm_down":
            self.left_motor.set_speed(self.left_motor_pin, -100)
        elif body["command"] == "right_arm_up":
            self.right_motor.set_speed(self.right_motor_pin, 100)
        elif body["command"] == "right_arm_down":
            self.right_motor.set_speed(self.right_motor_pin, -100)
        elif body["command"] == "stop":
            self.left_motor.set_speed(self.left_motor_pin, 0)
            self.right_motor.set_speed(self.right_motor_pin, 0)
        
        message.ack()

# Example usage
print("Starting TrackMotionCommand...")
track_command = TrackMotionCommand(sensor_channel='track_channel', motor_pin=2)
print("Starting ArmMotionCommand...")
arm_command = ArmMotionCommand(sensor_channel='arm_channel', left_motor_pin=3, right_motor_pin=1)

print("Setup complete. Waiting for commands...")

# Set up Kombu messaging
rabbit_url = os.getenv('RABBIT_URL', 'amqp://guest:guest@localhost//')
connection = Connection(rabbit_url)
channel = connection.channel()

exchange = Exchange('rescuebot', type='direct')
queue = Queue(name='motor_queue', exchange=exchange, routing_key='motor')

def handle_message(body, message):
    print(f"Message received: {body}")
    if 'track' in body['sensor_channel']:
        track_command.handle(body, message)
