
from rescuebot.rescuebot_os_old.kinetics.base import ControlSignalsBase
import rospy
from std_msgs.msg import (
    String,
    ColorRGBA,
    Float32,
    Int32,
    Duration
)


class ArmMotorControlSignals(ControlSignalsBase):
    """
    Motor control signals for the arms of the robot
    
    Listens to Commands from AI/Manaul Input and passes it onto the channel
    """
    
    def __init__(self, listener_name:str='arm_motor_control_signals_left',  
                 channel_name:str='left_arm_motor_signals',  
                 channel_dtype:str=Float32,
                 anonymous=False):
        
        self.listener_name = listener_name
        self.sensor_channel = channel_name
        self.annonymous = anonymous
        self.channel_dtype = channel_dtype  
        
    @staticmethod  
    def callback(data):
      rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
      
    def listener(self):
        rospy.init_node(self.listener_name, 
                        anonymous=self.annonymous)
        rospy.Subscriber(self.sensor_channel, self.channel_dtype, self.callback)
        