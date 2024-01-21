from rescuebot_lite.feeds.base import SensorBaseListener
import numpy as np
import time
def generate_random_temperate():
    return np.random.randint(30, 50)


if __name__ == '__main__':
    
    random_temp_sensor_listener = SensorBaseListener('sensor_data', 'temperature_channel')
    
    random_temp_sensor_listener.listen()
        
        

