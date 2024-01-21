# Use an official Python runtime as a parent image
FROM python:3.9-slim

# Set the working directory in the container to /app
WORKDIR /app

# Copy only the requirements file, to cache the installed packages
COPY requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

# Copy the rest of the application
COPY . /app

# Make port 8501 available to the world outside this container
EXPOSE 8501

# Define environment variable
ENV NAME=TemperatureApp

# The REDIS_URL should not be hardcoded here, pass it at runtime
ENV REDIS_URL="redis-19550.c268.eu-west-1-2.ec2.cloud.redislabs.com:19550"

# Run app.py when the container launches
CMD ["streamlit", "run", "app.py"]
