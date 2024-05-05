from rescuebot.feeds.base import SensorBaseWritter, SENSORS_CHANNEL
from rescuebot.feeds.environment import EnvironmentSensorWriter
from rescuebot.feeds.light import LightSensorWriter
from rescuebot.feeds.gyro import MotionSensorWriter
from multiprocessing import Process
from datetime import datetime
import numpy as np
import time
from absl import logging

logging.set_verbosity(logging.DEBUG)
logging.debug(':::RESCUE BOT::::')


def init_random_temperature_sensor(sample_rate_seconds=1):
    logging.debug('initialising temperature sensors...')
    # Create Sensors (re)
    random_temp_sensor = SensorBaseWritter(sensor_channel=SENSORS_CHANNEL, 
                                           sensor_name='random_temperature_sensor')

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
        

        
        
    

if __name__ == '__main__':
    # initial 
    temp_sensor_process = Process(target=init_random_temperature_sensor, args=(1,), daemon=False)
    temp_sensor_process.start()
    
    temp_sensor_process.join()
    logging.debug('Shutting down x_x')