from rescuebot_lite.feeds.base import SensorBaseListener, SENSORS_CHANNEL
import numpy as np
import time
def generate_random_temperate():
    return np.random.randint(30, 50)


if __name__ == '__main__':
    
    random_temp_sensor_listener = SensorBaseListener('temperature_sensor')
    
    random_temp_sensor_listener.listen()
        
        

