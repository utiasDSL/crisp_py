"""Simple example to control the gripper."""

import cv2
from crisp_py.camera import Camera, CameraConfig

camera_config = CameraConfig(
    camera_name="primary",
    camera_frame="primary_link",
    resolution=(256, 256),
    camera_color_image_topic="/camera_namespace/wrist_camera/color/image_rect_raw",
    camera_color_info_topic="/camera_namespace/wrist_camera/color/camera_info",
)

camera = Camera(config=camera_config, namespace="")
camera.wait_until_ready()

cv2.imshow("Camera Image", camera.current_image)
cv2.waitKey(0)
