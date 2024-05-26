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
from motoron import MotoronI2C, MotoronBase  # Use the correct class

# Initialize Motoron on I2C bus 2
motoron = MotoronI2C(bus=2)

class MotorMotionCommand:
    """
    Motor Movement
    Listens for movement commands and handles them for motors  
    """
    def __init__(self, sensor_channel: str, motion_controller: MotoronBase):
        self.sensor_channel = sensor_channel
        self.motion_controller = motion_controller
            
    def handle(self, body, message):
        """
        Handles movement command
        """
        pass

class TrackMotionCommand(MotorMotionCommand):
    def __init__(self, sensor_channel: str, motor_number: int):
        self.track_motor = motoron  # Assuming motoron is the motor controller
        super().__init__(sensor_channel=sensor_channel, motion_controller=self.track_motor)
        
    def handle(self, body, message):
        """
        Override the handle method to process movement commands.
        """
        logging.debug(f'Received movement command: {body}')
        print(f'Received movement command: {body}')
        
        if body["command"] == "forward":
            self.motion_controller.set_speed(motor_number, 100)
        elif body["command"] == "backward":
            self.motion_controller.set_speed(motor_number, -100)
        elif body["command"] == "stop":
            self.motion_controller.set_speed(motor_number, 0)
        
        message.ack()

class ArmMotionCommand(MotorMotionCommand):
    def __init__(self, sensor_channel: str, left_motor_number: int, right_motor_number: int):
        self.left_motor = motoron  # Assuming motoron is the motor controller
        self.right_motor = motoron  # Assuming motoron is the motor controller
        super().__init__(sensor_channel=sensor_channel, motion_controller=None)
        self.left_motor_number = left_motor_number
        self.right_motor_number = right_motor_number
        
    def handle(self, body, message):
        """
        Override the handle method to process movement commands.
        """
        logging.debug(f'Received movement command: {body}')
        print(f'Received movement command: {body}')
        
        if body["command"] == "left_arm_up":
            self.left_motor.set_speed(self.left_motor_number, 100)
        elif body["command"] == "left_arm_down":
            self.left_motor.set_speed(self.left_motor_number, -100)
        elif body["command"] == "right_arm_up":
            self.right_motor.set_speed(self.right_motor_number, 100)
        elif body["command"] == "right_arm_down":
            self.right_motor.set_speed(self.right_motor_number, -100)
        elif body["command"] == "stop":
            self.left_motor.set_speed(self.left_motor_number, 0)
            self.right_motor.set_speed(self.right_motor_number, 0)
        
        message.ack()

# Example usage
track_command = TrackMotionCommand(sensor_channel='track_channel', motor_number=2)
arm_command = ArmMotionCommand(sensor_channel='arm_channel', left_motor_number=3, right_motor_number=1)
