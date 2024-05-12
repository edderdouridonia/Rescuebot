import streamlit as st
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Streamlit app title and sidebar for controls
st.title("Robot Control and Monitoring Interface")
st.sidebar.title("Robot Controls")

# Sidebar for movement controls arranged in a diamond pattern
st.sidebar.header("Movement Controls")

# Top row for the forward button, shifted right
top_col1, top_col2, top_col3 = st.sidebar.columns([1, 1, 1])
with top_col2:
    if st.button("  ↑  ", key="forward"):  # Up arrow for forward
        pass 
        # st.sidebar.write("Moving forward")

# Middle row for left, stop, and right buttons
mid_col1, mid_col2, mid_col3 = st.sidebar.columns(3)
with mid_col1:
    if st.button(" ← ", key="left"):  # Left arrow for turn left
        pass
        # st.sidebar.write("Turning left")
with mid_col2:
    if st.button("■", key="stop"):  # Stop button
        pass
        # st.sidebar.write("Emergency Stop")
with mid_col3:
    if st.button(" → ", key="right"):  # Right arrow for turn right
        pass
        # st.sidebar.write("Turning right")

# Bottom row for the backward button, shifted right
bottom_col1, bottom_col2, bottom_col3 = st.sidebar.columns([1, 1, 1])
with bottom_col2:
    if st.button("  ↓  ", key="backward"):  # Down arrow for backward
        pass
        # st.sidebar.write("Moving backward")


# Generating mock data for sensors
def generate_mock_data():
    time_range = pd.date_range(start="2023-05-10", periods=100, freq='H')
    data = {
        "timestamp": time_range,
        "temperature": np.random.normal(loc=25, scale=3, size=100),
        "humidity": np.random.uniform(30, 90, size=100),
        "pressure": np.random.uniform(900, 1100, size=100),
        "angular_velocity": np.random.normal(loc=0, scale=1, size=100),
        "uv_index": np.random.uniform(0, 11, size=100),
        "voc_index": np.random.uniform(0, 500, size=100),
        "ambient_light": np.random.uniform(100, 10000, size=100)
    }
    return pd.DataFrame(data)

df_sensors = generate_mock_data()

tab1, tab2 = st.tabs(["Sensor Data", "Live Video Stream"])

with tab1:
    st.write("Data Preview:")
    st.write(df_sensors.head())
    # Plotting sensor data
    fig, axs = plt.subplots(7, 1, figsize=(10, 20))
    sensor_names = ["temperature", "humidity", "pressure", "angular_velocity", "uv_index", "voc_index", "ambient_light"]
    colors = ["red", "blue", "green", "purple", "orange", "brown", "yellow"]
    for i, sensor in enumerate(sensor_names):
        axs[i].plot(df_sensors["timestamp"], df_sensors[sensor], color=colors[i])
        axs[i].set_xlabel("Timestamp")
        axs[i].set_ylabel(f"{sensor}")
        axs[i].set_title(f"{sensor.capitalize()} Readings Over Time")
    plt.tight_layout()
    st.pyplot(fig)

with tab2:
    st.header("Live Video Streams")
    col1, col2, col3 = st.columns(3)
    with col1:
        video_stream_url1 = st.text_input("Enter video stream URL 1", key="video1")
        if video_stream_url1:
            st.video(video_stream_url1)
    with col2:
        video_stream_url2 = st.text_input("Enter video stream URL 2", key="video2")
        if video_stream_url2:
            st.video(video_stream_url2)
    with col3:
        video_stream_url3 = st.text_input("Enter video stream URL 3", key="video3")
        if video_stream_url3:
            st.video(video_stream_url3)
