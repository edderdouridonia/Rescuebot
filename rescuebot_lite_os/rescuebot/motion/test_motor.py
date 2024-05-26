from kombu import Connection, Exchange, Queue
import json

# Define the connection and exchange
connection = Connection('amqp://guest:guest@localhost//')
exchange = Exchange('motor_exchange', type='direct')

# Define the producer
producer = connection.Producer()

# Example commands
commands = [
    {"command": "forward"},
    {"command": "backward"},
    {"command": "stop"},
    {"command": "left_arm_up"},
    {"command": "left_arm_down"},
    {"command": "right_arm_up"},
    {"command": "right_arm_down"},
]

# Send the commands
for command in commands:
    message = json.dumps(command)
    producer.publish(message, exchange=exchange, routing_key='motor_commands')
    print(f"Sent command: {message}")
