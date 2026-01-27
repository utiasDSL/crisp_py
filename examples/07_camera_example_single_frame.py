"""Simple example for a camera. It captures a single frame from a camera and plots it using matplotlib."""

import matplotlib.pyplot as plt
from crisp_py.camera import make_camera

camera = make_camera("right_wrist_camera", namespace="right")

# You could also use the Camera object directly:
# from crisp_py.camera import CameraConfig, Camera
# camera_config = CameraConfig(
#     camera_name="primary",
#     camera_frame="primary_link",
#     resolution=[256, 256],
#     camera_color_image_topic="/image_raw",
#     camera_color_info_topic="/image_raw/camera_info",
# )
#
# camera = Camera(config=camera_config, namespace="")

camera.wait_until_ready()

plt.imshow(camera.current_image)
plt.axis("off")
plt.show()
