
#Base on  
import logging
from datetime import datetime
import logging
from datetime import datetime
from adafruit_icm20x import ICM20948, AccelRange, GyroRange
import board
import busio
from kombu import (
    Exchange, 
    Queue, 
    Producer, 
    Connection
)
from rescuebot.feeds.base import SensorReaderBase
from rescuebot.feeds.sensor_lib.gyro import ICM20948


# Constants for RabbitMQ
SENSORS_CHANNEL = "sensors"
RABBIT_MQ_SERVER_URI = "amqp://guest:guest@localhost:5672//"


class MotionSensorReader(SensorReaderBase):
    """
    ICM20948
    Specialized writer for the ICM-20948 Motion Sensor.
    """
    def __init__(self, sensor_name, sensor_channel):
        super().__init__(sensor_name, sensor_channel)
       
        self.sensor = ICM20948()
        logging.debug(f"{sensor_name} using ICM-20948 sensor initialized on channel {sensor_channel}.")

    def write_motion_data(self):
        """
        Reads motion data (acceleration, gyro, magnetic) and writes it with a timestamp.
        """
        
        raw_data = []
        
        raw_data = self.sensor.getdata()

        roll, pitch, yaw = (raw_data[0], raw_data[1], raw_data[2])
        accel_x, accel_y, accel_z = (raw_data[3], raw_data[4], raw_data[5])
        gyro_x, gyro_y, gyro_z =  (raw_data[6], raw_data[7], raw_data[8])
        mag_x, mag_y, mag_z =(raw_data[9], raw_data[10], raw_data[11])

        motion_data = {
            'acceleration': {'x': accel_x, 'y': accel_y, 'z': accel_z},
            'gyro': {'x': gyro_x, 'y': gyro_y, 'z': gyro_z},
            'magnetic': {'x': mag_x, 'y': mag_y, 'z': mag_z},
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw,
            'timestamp': datetime.now().isoformat()
        }
        self.write(motion_data)
        logging.info(f"Motion data written: {motion_data}")