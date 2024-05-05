


import RPi.GPIO as GPIO
import time
import logging
from datetime import datetime
from kombu import Exchange, Queue, Producer, Connection

from rescuebot.feeds.base import SensorBaseWritter

class UltrasonicSensorWriter(SensorBaseWritter):
    """
    Specialized writer for the HC-SR04 Ultrasonic Sensor.
    """
    def __init__(self, sensor_name, sensor_channel, trigger_pin, echo_pin):
        super().__init__(sensor_name, sensor_channel)
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(trigger_pin, GPIO.OUT)
        GPIO.setup(echo_pin, GPIO.IN)
        GPIO.output(trigger_pin, GPIO.LOW)
        logging.debug(f"{sensor_name} using HC-SR04 sensor initialized on channel {sensor_channel}.")

    def measure_distance(self):
        """
        Measures distance using the ultrasonic sensor.
        """
        GPIO.output(self.trigger_pin, GPIO.HIGH)
        time.sleep(0.00001)  # 10 microseconds
        GPIO.output(self.trigger_pin, GPIO.LOW)

        start_time = time.time()
        stop_time = start_time

        while GPIO.input(self.echo_pin) == 0:
            start_time = time.time()

        while GPIO.input(self.echo_pin) == 1:
            stop_time = time.time()

        elapsed_time = stop_time - start_time
        distance = (elapsed_time * 34300) / 2  # Speed of sound at 34300 cm/s

        return distance

    def write_distance(self):
        """
        Reads distance and writes it with a timestamp.
        """
        distance = self.measure_distance()
        data = {
            'distance': distance,
            'timestamp': datetime.now().isoformat()
        }
        self.write(data)
        logging.info(f"Distance data written: {data}")

    def cleanup(self):
        """
        Cleans up GPIO resources.
        """
        GPIO.cleanup()