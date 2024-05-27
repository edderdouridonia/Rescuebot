import time
from motoron import MotoronI2C, MotoronBase

# Initialize Motoron on I2C bus 2
motoron = MotoronI2C(bus=2)
print("Initialized Motoron on I2C bus 2")

class MotorMotionCommand:
    def __init__(self, motor_number):
        self.motor = MotoronBase(motoron, motor_number)
        print(f"MotorMotionCommand initialized for motor pin {motor_number}")

    def set_speed(self, speed):
        self.motor.set_speed(speed)
        print(f"Motor {self.motor.motor_number} speed set to {speed}")

    def set_direction(self, direction):
        if direction == "forward":
            self.set_speed(abs(self.motor.get_speed()))
        elif direction == "backward":
            self.set_speed(-abs(self.motor.get_speed()))
        print(f"Motor {self.motor.motor_number} direction set to {direction}")

    def stop(self):
        self.set_speed(0)
        print(f"Motor {self.motor.motor_number} stopped")

class TrackMotionCommand(MotorMotionCommand):
    def __init__(self, motor_number):
        super().__init__(motor_number)
        print(f"TrackMotionCommand initialized for motor pin {motor_number}")

    def forward(self, speed=100):
        self.set_speed(speed)
        print("Track moving forward")

    def backward(self, speed=100):
        self.set_speed(-speed)
        print("Track moving backward")

class ArmMotionCommand:
    def __init__(self, left_motor_number, right_motor_number):
        self.left_motor = MotorMotionCommand(left_motor_number)
        self.right_motor = MotorMotionCommand(right_motor_number)
        print(f"ArmMotionCommand initialized for left motor pin {left_motor_number} and right motor pin {right_motor_number}")

    def left_arm_up(self, speed=100):
        self.left_motor.set_speed(speed)
        print("Left arm moving up")

    def left_arm_down(self, speed=100):
        self.left_motor.set_speed(-speed)
        print("Left arm moving down")

    def right_arm_up(self, speed=100):
        self.right_motor.set_speed(speed)
        print("Right arm moving up")

    def right_arm_down(self, speed=100):
        self.right_motor.set_speed(-speed)
        print("Right arm moving down")

    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()
        print("Arms stopped")

# Example usage
track_command = TrackMotionCommand(motor_number=3)  # Assuming motor is connected to M3A and M3B
arm_command = ArmMotionCommand(left_motor_number=1, right_motor_number=2)  # Assuming motors are connected to M1A/M1B and M2A/M2B

# Simulating motor commands for track
print("Track moving forward")
track_command.forward()
time.sleep(2)

print("Track moving backward")
track_command.backward()
time.sleep(2)

print("Track stopping")
track_command.stop()

# Simulating motor commands for arm
print("Left arm moving up")
arm_command.left_arm_up()
time.sleep(2)

print("Right arm moving up")
arm_command.right_arm_up()
time.sleep(2)

print("Arms stopping")
arm_command.stop()
