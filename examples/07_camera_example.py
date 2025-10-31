"""Simple example for a camera."""

import cv2
from crisp_py.camera import make_camera

# You could also use the Camera object directly:
camera = make_camera("right_wrist_camera", namespace="right")
camera.wait_until_ready()

cv2.imshow("Camera Image", camera.current_image)
cv2.waitKey(0)
