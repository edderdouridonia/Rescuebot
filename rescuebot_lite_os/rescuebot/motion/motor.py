import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), '../../'))

from kombu import (
    Connection, 
    Exchange, 
    Producer,
    Consumer,
    Queue,
    eventloop
)
from rescuebot.motion.base import MotionCommandBase, MotorController
from pprint import pformat
from absl import logging
from motoron import MotoronI2C, Motoron

# Initialize Motoron on I2C bus 2
motoron = MotoronI2C(bus=2)

class MotorMotionCommand(MotionCommandBase):
    """
    Motor Movement
    Listens for movement commands and handles them for motors  
    """
    def __init__(self, sensor_channel: str, motion_controller: MotorController):
        super().__init__(sensor_channel=sensor_channel, 
                         motion_controller=motion_controller)
            
    def handle(self, body, message):
        """
        Handles movement command
        """
        pass

class TrackMotionCommand(MotorMotionCommand):
    def __init__(self, sensor_channel: str, motor_pin: int):
        self.track_motor = Motoron(controller=motoron, motor_number=motor_pin)
        super().__init__(sensor_channel=sensor_channel, motion_controller=None)
        
    def handle(self, body, message):
        """
        Override the handle method to process movement commands.
        """
        logging.debug(f'Received movement command: {self.pretty(body)}')
        print(f'Received movement command: {self.pretty(body)}')
        
        if body["command"] == "forward":
            self.track_motor.set_speed(100)
        elif body["command"] == "backward":
            self.track_motor.set_speed(-100)
        elif body["command"] == "stop":
            self.track_motor.set_speed(0)
        
        message.ack()

class ArmMotionCommand(MotorMotionCommand):
    def __init__(self, sensor_channel: str, left_motor_pin: int, right_motor_pin: int):
        self.left_motor = Motoron(controller=motoron, motor_number=left_motor_pin)
        self.right_motor = Motoron(controller=motoron, motor_number=right_motor_pin)
        super().__init__(sensor_channel=sensor_channel, motion_controller=None)
        
    def handle(self, body, message):
        """
