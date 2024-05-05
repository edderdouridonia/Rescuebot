import RPi.GPIO as GPIO
import time

# Constants for the GPIO pins
encoder0pinA = 2  # A pin -> the interrupt pin
encoder0pinB = 4  # B pin -> the digital pin
duration = 0  # the number of the pulses
Direction = True  # the rotation direction

# Initial state of A pin
encoder0PinALast = 0

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(encoder0pinA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(encoder0pinB, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    encoder_init()

def encoder_init():
    global Direction
    Direction = True  # default -> Forward
    GPIO.add_event_detect(encoder0pinA, GPIO.BOTH, callback=wheel_speed)

def wheel_speed(channel):
    global encoder0PinALast, duration, Direction
    Lstate = GPIO.input(encoder0pinA)
    if encoder0PinALast == GPIO.LOW and Lstate == GPIO.HIGH:
        val = GPIO.input(encoder0pinB)
        if val == GPIO.LOW and Direction:
            Direction = False  # Reverse
        elif val == GPIO.HIGH and not Direction:
            Direction = True   # Forward
    encoder0PinALast = Lstate

    if not Direction:
        duration += 1
    else:
        duration -= 1

def loop():
    while True:
        print("Pulse:", duration)
        global duration
        duration = 0
        time.sleep(0.1)

if __name__ == '__main__':
    setup()
    try:
        loop()
    except KeyboardInterrupt:
        GPIO.cleanup()
