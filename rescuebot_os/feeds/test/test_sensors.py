import pytest
from rescuebot_os.feeds.temperate import TemperatureSensor  # Replace 'your_module' with the name of your module


# Test the initialization of SensorWriterBase
def test_sensor_writer_base_init(temp_sensor):
   temp_sensor = TemperatureSensor()
   
   temp_sensor.read_from_sensor()
    
