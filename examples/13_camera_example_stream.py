"""Simple toy example to visualize the camera image."""

import time
import matplotlib.pyplot as plt
from crisp_py.camera import CameraConfig, Camera


camera_config = CameraConfig(
    camera_name="primary",
    camera_frame="primary_link",
    resolution=[256, 256],
    camera_color_image_topic="/image_raw",
    camera_color_info_topic="/image_raw/camera_info",
)

camera = Camera(config=camera_config, namespace="")
camera.wait_until_ready()

# Display camera feed
plt.ion()
fig, ax = plt.subplots()
ax.axis("off")

frame = camera.current_image
im = ax.imshow(frame)
while True:
    im.set_data(camera.current_image)
    plt.pause(1.0 / 30.0)
