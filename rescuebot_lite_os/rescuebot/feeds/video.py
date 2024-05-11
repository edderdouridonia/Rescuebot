
import cv2
import logging
from datetime import datetime
from kombu import Exchange, Queue, Producer, Connection
import numpy as np
import pickle

# Assuming these constants are defined
SENSORS_CHANNEL = "sensors"
RABBIT_MQ_SERVER_URI = "amqp://guest:guest@localhost:5672//"

from rescuebot.feeds.base import SensorBaseWritter

class VideoSensorWriter(SensorBaseWritter):
    """
    Class to capture video from a USB camera and publish frames to a message broker.
    """
    def __init__(self, sensor_name, sensor_channel, camera_index=0):
        super().__init__(sensor_name, sensor_channel)
        self.camera = cv2.VideoCapture(camera_index)
        if not self.camera.isOpened():
            logging.error(f"Cannot open camera {camera_index}")
            raise ValueError(f"Cannot open camera {camera_index}")
        logging.info(f"VideoSensorWriter initialized for camera {camera_index} on channel {sensor_channel}")

    def write_frame(self):
        """
        Captures a single frame from the camera and publishes it.
        """
        ret, frame = self.camera.read()
        if not ret:
            logging.error("Can't receive frame (stream end?). Exiting ...")
            return
        # Serialize frame
        _, buffer = cv2.imencode('.jpg', frame)
        serialized_frame = pickle.dumps(buffer)
        cv2.imwrite('test.jpg', frame)

        # Create message
        message = {
            'frame': serialized_frame,
            'timestamp': datetime.now().isoformat()
        }

        self.write(message)
        logging.info("Frame published.")

    def release(self):
        """
        Releases the camera resource.
        """
        self.camera.release()
        cv2.destroyAllWindows()
        logging.info("Camera released.")


if __name__ == '__main__':
	vid = VideoSensorWriter()
	vid.write_frame()
	
