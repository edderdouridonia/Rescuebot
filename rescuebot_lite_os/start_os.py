from rescuebot.feeds.base import SensorReaderBase, SensorReadingListener, SENSORS_CHANNEL
from rescuebot.feeds.environment import EnvironmentSensorReader
from rescuebot.feeds.light import (
    UVSensorReader, LuminositySensorReader
)
from rescuebot.feeds.gyro import MotionSensorReader
from rescuebot.feeds.gas import AirQualitySensorReader
#from rescuebot.feeds.video import VideoFeedReader
#from rescuebot.feeds.proximity import ProximitySensorReader
#from rescuebot.feeds.ultrasonic import UltrasonicSensorReader
from multiprocessing import Process
from datetime import datetime
import time
from absl import logging

logging.set_verbosity(logging.DEBUG)
logging.debug(':::RESCUE BOT::::')



def init_environment_sensor_reader(sample_rate_seconds=1):
    logging.debug('initialising environment sensors...')
    # Create Sensors (re)
    env_sensor = EnvironmentSensorReader(sensor_channel=SENSORS_CHANNEL, 
                                           sensor_name='environment_sensor')

    logging.debug('reading environment :)')
    while True:
        # Get the current date and time
        env_sensor.write_environment_data()
        time.sleep(sample_rate_seconds)
        
        
def init_luminosity_sensor_reader(sample_rate_seconds=1):
    logging.debug('initialising luminosity sensors...')
    # Create Sensors (re)
    light_sensor = LuminositySensorReader(sensor_channel=SENSORS_CHANNEL, 
                                           sensor_name='light_sensor')

    logging.debug('reading environment :)')
    while True:
        # Get the current date and time
        data = light_sensor.write_lux_data()
        time.sleep(sample_rate_seconds)
        

def init_uv_sensor_reader(sample_rate_seconds=1):
    logging.debug('initialising uv sensors...')
    # Create Sensors (re)
    light_sensor = UVSensorReader(sensor_channel=SENSORS_CHANNEL, 
                                           sensor_name='uv_sensor')

    logging.debug('UV reading environment :)')
    while True:
        # Get the current date and time
        light_sensor.write_light_data()
        time.sleep(sample_rate_seconds)
        

def init_airquality_sensor_reader(sample_rate_seconds=1):
    logging.debug('initialising environment sensors...')
    # Create Sensors (re)
    light_sensor = AirQualitySensorReader(sensor_channel=SENSORS_CHANNEL, 
                                           sensor_name='uv_sensor')

    logging.debug('UV reading environment :)')
    while True:
        # Get the current date and time
        light_sensor.write_lux_data()
        time.sleep(sample_rate_seconds)
          
def init_gyro_sensor_reader(sample_rate_seconds=1):
    logging.debug('initialising gyro sensors...')
    # Create Sensors (re)
    motion_sensor = MotionSensorReader(sensor_channel=SENSORS_CHANNEL, 
                                           sensor_name='motion_sensor')

    logging.debug('reading environment :)')
    while True:
        # Get the current date and time
        motion_sensor.write_motion_data()
        time.sleep(sample_rate_seconds)
        
def init_air_sensor_reader(sample_rate_seconds=1):
    logging.debug('initialising air sensors...')
    # Create Sensors (re)
    air_sensor = AirQualitySensorReader(sensor_channel=SENSORS_CHANNEL, 
                                           sensor_name='air_sensor')

    logging.debug('reading environment :)')
    while True:
        # Get the current date and time
        air_sensor.write_voc_data()
        time.sleep(sample_rate_seconds)
        

def init_video_feed_reader(sample_rate_seconds=1):
    logging.debug('initialising air sensors...')
    # Create Sensors (re)
    air_sensor = VideoFeedReader(sensor_channel=SENSORS_CHANNEL, 
                                           sensor_name='video_feed')

    logging.debug('reading environment :)')
    while True:
        # Get the current date and time
        air_sensor.write_frame()



def init_proximity_sensor(sample_rate_seconds=1):
    logging.debug('initialising proximity sensors...')
    # Create Sensors (re)
    proximity_sensor = ProximitySensorReader(sensor_channel=SENSORS_CHANNEL, 
                                           sensor_name='proximity_sensor')

    logging.debug('reading environment :)')
    while True:
        # Get the current date and time
        proximity_sensor.write_proximity_data()
        
        
def init_ultrasonic_sensor(sample_rate_seconds=1):
    logging.debug('initialising ultrasound sensors...')
    # Create Sensors (re)
    ultrasonic_sensor = UltrasonicSensorReader(sensor_channel=SENSORS_CHANNEL, 
                                           sensor_name='ultrasonic_sensor')

    logging.debug('reading environment :)')
    while True:
        # Get the current date and time
        ultrasonic_sensor.write_distance()
    
    

if __name__ == '__main__':
    logging.debug('::: System startup - Pausing for 5 seconds to connect to RabbitMQ :::')
    time.sleep(5)

    # Creating and starting processes
    sensors = {
        'environment': init_environment_sensor_reader,
        'luminosity': init_luminosity_sensor_reader,
        'uv': init_uv_sensor_reader,
         'airquality': init_airquality_sensor_reader,
        #' 'gyro': init_gyro_sensor_reader,
         #''air': init_air_sensor_reader,
        #'video': init_video_feed_reader,
        # 'proximity': init_proximity_sensor_reader,
        # 'ultrasonic': init_ultrasonic_sensor_reader
    }
    
    processes = []
    for sensor_name, init_func in sensors.items():
        process = Process(target=init_func, args=(1,))
        processes.append(process)
        process.start()
        logging.debug(f'{sensor_name} sensor process started.')

    # Wait for all processes to complete (they won't unless terminated)
    for process in processes:
        process.join()

    logging.debug('All sensor processes have been terminated.')
