from machine import Pin, ADC
import utime
import ujson

from kombu import (
    Exchange, 
    Queue, 
    Producer, 
    Connection
)
from rescuebot.feeds.base import SensorReaderBase


class ProximitySensorReader(SensorReaderBase):
    def __init__(self, sensor_name, sensor_channel, pin_number, adc_channel):
        super().__init__(sensor_name, sensor_channel)
        self.din = Pin(pin_number, Pin.IN, Pin.PULL_UP)
        self.adc = ADC(adc_channel)
        self.conversion_factor = 3.3 / 65535

    def write_proximity_data(self):
        data = {}
        if self.din.value() == 1:
            data["status"] = "Far from obstacles"
        else:
            data["status"] = "Near the obstacles"
            ad_value = self.adc.read_u16()  # ADC reading
            voltage = ad_value * self.conversion_factor
            data["gas_ad_value_raw"] = ad_value
            data["voltage_converted"] = f"{voltage:.2f} V"
            self.write(data)
            
