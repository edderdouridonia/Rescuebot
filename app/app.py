import streamlit as st
from kombu import Connection, Exchange, Queue, Consumer
import pandas as pd
import matplotlib.pyplot as plt
from absl import logging


logging.set_verbosity(logging.DEBUG)

# Streamlit app title
st.title("Temperature Sensor Data Visualization")

# Configuration for Redis connection
REDIS_URL = "redis-19550.c268.eu-west-1-2.ec2.cloud.redislabs.com"
USERNAME = "rescuebot"
PASSWORD = "gTtXHpg3hYNa6#Y5"

TEMPERATURE_SENSOR_CHANNEL = "temperature_sensor"

def fetch_temperature_data():
    logging.debug("Fetching temperature data ...")
    with Connection(hostname=REDIS_URL, userid=USERNAME,
                    password=PASSWORD, port=19550) as conn:
        # Define the exchange and queue
        exchange = Exchange("sensor_data", type="direct")
        queue = Queue(name=TEMPERATURE_SENSOR_CHANNEL, exchange=exchange, 
                      routing_key=TEMPERATURE_SENSOR_CHANNEL)

        # Temporary list to store the data
        temp_data = []

        # Callback function to process messages
        def process_message(body, message):
            print(f'Received message: {body}')
            temp_data.append(body)
            message.ack()

        # Consuming messages
        with Consumer(conn, queues=queue,
                      callbacks=[process_message],
                      no_ack=False):
            conn.drain_events(timeout=2)  # Adjust timeout as needed

        return temp_data

# Button to fetch data
if st.button("Fetch Temperature Data"):
    data = fetch_temperature_data()
    if data:
        # Assuming data is a list of dictionaries
        df = pd.DataFrame(data)
        st.write("Data Preview:")
        st.write(df.head())

        # Plotting
        st.write("Temperature Readings:")
        fig, ax = plt.subplots()
        ax.plot(df["timestamp"], df["temperature"], color="red")
        ax.set_xlabel("Timestamp")
        ax.set_ylabel("Temperature (°C)")
        ax.set_title("Temperature Readings Over Time")
        st.pyplot(fig)
    else:
        st.error("No data fetched. Ensure the Redis server is running and sending data to the queue.")
