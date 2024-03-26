import os
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


        
class MotorMotionCommand(MotionCommandBase):
    """
    Motor Movement
    
    Listens for movement commands and handles them for motors  
     
    """
    
    def __init___(self, sensor_channel:str, motion_controller:MotorController):
        super.__init__(sensor_channel=sensor_channel, 
                       motion_controller=motion_controller)
            
    
    def handle(self, body, message):
        """
        Handles movement command
        """        
        #TODO: override this method to use MotionController
        pass
    
    
    
class WheelMotionCommand(MotorMotionCommand):
    
    def __init__(self, sensor_channel:str, left_motor_pin:int, right_motor_pin:int):
        self.left_motion_controller = MotorController(motor_pin=left_motor_pin)
        self.right_motion_controller = MotorController(motor_pin=right_motor_pin)
        super.__init__(sensor_channel=sensor_channel, motion_controller=None)
        
    def handle(self,body, message):
        """
        Override the handle method to process movement commands.
        """
        # Logging the received movement command
        logging.debug(f'Received movement command: {self.pretty(body)}')
        print(f'Received movement command: {self.pretty(body)}')
        
        if body["command"] == "left":
            self.left_motion_controller.move_forward()
            
        if body["command"] == "right":
            self.right_motion_controller.move_forward()
            
        if body["command"] == "forward":
            self.right_motion_controller.move_forward()
            self.left_motion_controller.move_forward()
            
        if body["command"] == "backward":
            self.right_motion_controller.move_backward()
            self.left_motion_controller.move_backward()
        
        # Acknowledge the message
        message.ack()
        
        
    class ArmMotionCommand(MotorMotionCommand):
        
        def __init__(self, sensor_channel:str, left_motor_pin:int, right_motor_pin:int):
            #TODO: next time
            pass
        
        def handle(self,body, message):
            """
            Override the handle method to process movement commands.
            """
            # Logging the received movement command
            logging.debug(f'Received movement command: {self.pretty(body)}')
            print(f'Received movement command: {self.pretty(body)}')
            
            #TODO: next item
            
            pass
        
        
        
        