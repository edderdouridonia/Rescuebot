from rescuebot.feeds.base import SensorBaseWritter, SensorBaseListener, SENSORS_CHANNEL
from rescuebot.feeds.environment import EnvironmentSensorWriter
from rescuebot.feeds.light import LightSensorWriter
from rescuebot.feeds.gyro import MotionSensorWriter
from rescuebot.feeds.air import AirQualitySensorWriter
from rescuebot.feeds.video import VideoSensorWriter
from rescuebot.feeds.proximity import ProximitySensorWriter
from rescuebot.feeds.ultrasonic import UltrasonicSensorWriter
from multiprocessing import Process
from datetime import datetime
import numpy as np
import time
from absl import logging

logging.set_verbosity(logging.DEBUG)
logging.debug(':::RESCUE BOT::::')


def init_random_temperature_sensor(sample_rate_seconds=1):
    logging.debug('SensorWritter: initialising temperature sensors...')
    # Create Sensors (re)
    random_temp_sensor = SensorBaseWritter(sensor_channel=SENSORS_CHANNEL, 
                                           sensor_name='temperature_sensor')

    """ random temperature sensor """
    # generate random temperatures
    def generate_random_temperate():
        return np.random.randint(30, 50)

    logging.debug('reading temperatures :)')
    while True:
        # Get the current date and time
        now = datetime.now()

        # Format the timestamp
        # Example: 2024-01-21 12:34:56
        timestamp = now.strftime("%Y-%m-%d %H:%M:%S")
        test_reading = {'temperature': generate_random_temperate(), 
                        'timestamp': timestamp }
        random_temp_sensor.write(test_reading)
        time.sleep(sample_rate_seconds)
        
        
def init_random_temperature_sensor_reader():
    logging.debug('SensorReader: initialising temperature sensors reader...')
   
    random_temp_sensor_listener = SensorBaseListener('temperature_sensor')

    random_temp_sensor_listener.listen()
    
    

def init_environment_sensor(sample_rate_seconds=1):
    logging.debug('initialising environment sensors...')
    # Create Sensors (re)
    env_sensor = EnvironmentSensorWriter(sensor_channel=SENSORS_CHANNEL, 
                                           sensor_name='environment_sensor')


    logging.debug('reading environment :)')
    while True:
        # Get the current date and time
        env_sensor.write_environment_data()
        time.sleep(sample_rate_seconds)
        
def init_light_sensor(sample_rate_seconds=1):
    logging.debug('initialising environment sensors...')
    # Create Sensors (re)
    light_sensor = LightSensorWriter(sensor_channel=SENSORS_CHANNEL, 
                                           sensor_name='light_sensor')

    logging.debug('reading environment :)')
    while True:
        # Get the current date and time
        light_sensor.write_lux_data()
        time.sleep(sample_rate_seconds)
        
        
def init_gyro_sensor(sample_rate_seconds=1):
    logging.debug('initialising gyro sensors...')
    # Create Sensors (re)
    motion_sensor = MotionSensorWriter(sensor_channel=SENSORS_CHANNEL, 
                                           sensor_name='motion_sensor')

    logging.debug('reading environment :)')
    while True:
        # Get the current date and time
        motion_sensor.write_motion_data()
        time.sleep(sample_rate_seconds)
        
def init_air_sensor(sample_rate_seconds=1):
    logging.debug('initialising air sensors...')
    # Create Sensors (re)
    air_sensor = AirQualitySensorWriter(sensor_channel=SENSORS_CHANNEL, 
                                           sensor_name='air_sensor')

    logging.debug('reading environment :)')
    while True:
        # Get the current date and time
        air_sensor.write_voc_data()
        time.sleep(sample_rate_seconds)
        

def init_video_feed(sample_rate_seconds=1):
    logging.debug('initialising air sensors...')
    # Create Sensors (re)
    air_sensor = VideoSensorWriter(sensor_channel=SENSORS_CHANNEL, 
                                           sensor_name='video_feed')

    logging.debug('reading environment :)')
    while True:
        # Get the current date and time
        air_sensor.write_frame()


def init_proximity_sensor(sample_rate_seconds=1):
    logging.debug('initialising proximity sensors...')
    # Create Sensors (re)
    proximity_sensor = ProximitySensorWriter(sensor_channel=SENSORS_CHANNEL, 
                                           sensor_name='proximity_sensor')

    logging.debug('reading environment :)')
    while True:
        # Get the current date and time
        proximity_sensor.write_proximity_data()
        
        
def init_ultrasonic_sensor(sample_rate_seconds=1):
    logging.debug('initialising ultrasound sensors...')
    # Create Sensors (re)
    ultrasonic_sensor = UltrasonicSensorWriter(sensor_channel=SENSORS_CHANNEL, 
                                           sensor_name='ultrasonic_sensor')

    logging.debug('reading environment :)')
    while True:
        # Get the current date and time
        ultrasonic_sensor.write_distance()
    
        
    

if __name__ == '__main__':
    logging.debug(':::RESCUE BOT - taking a nap (5 seconds) to await to rabbit mq::::')
    time.sleep(5)
    # initial 
    temp_sensor_process = Process(target=init_random_temperature_sensor, args=(1,), daemon=False)
    # ultrasonic_process = Process(target=init_ultrasonic_sensor, args=(1,), daemon=False)
    temp_sensor_reader_process = Process(target=init_random_temperature_sensor_reader, daemon=False)
    temp_sensor_process.start()
    # ultrasonic_process.start()
    temp_sensor_reader_process.start()
    
    temp_sensor_process.join()
    temp_sensor_reader_process.join()
    logging.debug('Shutting down x_x')