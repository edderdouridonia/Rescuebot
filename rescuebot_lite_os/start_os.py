from rescuebot.feeds.base import SensorBaseWritter, SensorBaseListener, SENSORS_CHANNEL
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
        
    

if __name__ == '__main__':
    logging.debug(':::RESCUE BOT - taking a nap (5 seconds) to await to rabbit mq::::')
    time.sleep(5)
    # initial 
    temp_sensor_process = Process(target=init_random_temperature_sensor, args=(1,), daemon=False)
    temp_sensor_reader_process = Process(target=init_random_temperature_sensor_reader, daemon=False)
    temp_sensor_process.start()
    temp_sensor_reader_process.start()
    
    temp_sensor_process.join()
    temp_sensor_reader_process.join()
    logging.debug('Shutting down x_x')