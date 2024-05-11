 FROM python:3.12


RUN apt-get upgrade && apt-get update && apt-get install -y \
    gcc \
    libc-dev \
    python3-dev

# Set the working directory in the container to /app
WORKDIR /app

# Copy only the requirements file, to cache the installed packages
COPY rescuebot/requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

# smbus
RUN apt-get install -y \
    python3-smbus \
    i2c-tools \
    icm20948

COPY . /app

# Define environment variable
ENV NAME=rescuebot_lite_os



# The REDIS_URL should not be hardcoded here, pass it at runtime
ENV REDIS_URL="redis-19550.c268.eu-west-1-2.ec2.cloud.redislabs.com:19550"

# Run app.py when the container launches
CMD ["python", "start_os.py"]
