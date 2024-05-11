
import cv2
import logging
from datetime import datetime
import numpy as np
import pickle

# Assuming these constants are defined


def test_camera_feed(camera_index=0, num_frames=5):

	camera = cv2.VideoCapture(camera_index)
	frame_idx=0
	while frame_idx < num_frames:
		ret, frame = camera.read()
		_, buffer = cv2.imencode('.jpg', frame)
		serialized_frame = pickle.dumps(buffer)
		cv2.imwrite(f'frame_{frame_idx}.jpg', frame)
		frame_idx +=1
	camera.release()
	cv2.destroyAllWindows()


if __name__ == '__main__':
	test_camera_feed()
	
