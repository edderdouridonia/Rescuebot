
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
from rescuebot.feeds.base import SensorBaseWritter


# Constants for RabbitMQ
SENSORS_CHANNEL = "sensors"
RABBIT_MQ_SERVER_URI = "amqp://guest:guest@localhost:5672//"


class MotionSensorWriter(SensorBaseWritter):
    """
    ICM20948
    Specialized writer for the ICM-20948 Motion Sensor.
    """
    def __init__(self, sensor_name, sensor_channel):
        super().__init__(sensor_name, sensor_channel)
        i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = ICM20948(i2c)
        logging.debug(f"{sensor_name} using ICM-20948 sensor initialized on channel {sensor_channel}.")

    def write_motion_data(self):
        """
        Reads motion data (acceleration, gyro, magnetic) and writes it with a timestamp.
        """
        accel_x, accel_y, accel_z = self.sensor.acceleration
        gyro_x, gyro_y, gyro_z = self.sensor.gyro
        mag_x, mag_y, mag_z = self.sensor.magnetic

        motion_data = {
            'acceleration': {'x': accel_x, 'y': accel_y, 'z': accel_z},
            'gyro': {'x': gyro_x, 'y': gyro_y, 'z': gyro_z},
            'magnetic': {'x': mag_x, 'y': mag_y, 'z': mag_z},
            'timestamp': datetime.now().isoformat()
        }
        self.write(motion_data)
        logging.info(f"Motion data written: {motion_data}")