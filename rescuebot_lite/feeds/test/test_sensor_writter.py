from rescuebot_lite.feeds.base import SensorBaseWritter, SENSORS_CHANNEL
from datetime import datetime


import numpy as np
import time



def generate_random_temperate():
    return np.random.randint(30, 50)


if __name__ == '__main__':
    
    random_temp_sensor = SensorBaseWritter(SENSORS_CHANNEL, 'temperature_sensor')
    while True:
        # Get the current date and time
        now = datetime.now()

        # Format the timestamp
        # Example: 2024-01-21 12:34:56
        timestamp = now.strftime("%Y-%m-%d %H:%M:%S")
        test_reading = {'temperature': generate_random_temperate(), 
                        'timestamp': timestamp }
        random_temp_sensor.write(test_reading)
        time.sleep(1)
        

