"""Example using the sensor module to read data from a sensor and display real-time Gaussian intensity image."""

from crisp_py.sensors.sensor import Sensor
from crisp_py.sensors.sensor_config import AnySkinSensorConfig

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import time

def create_multi_gaussian_image(size, sensor_values):
    """Create a 2D image with 5 Gaussians positioned at center, top, bottom, left, right."""
    x = np.linspace(0, size-1, size)
    y = np.linspace(0, size-1, size)
    X, Y = np.meshgrid(x, y)

    positions = [
        (size // 2, size // 2),      # center
        (size // 4, size // 2),      # left
        (3 * size // 4, size // 2),   # right
        (size // 2, size // 4),      # top
        (size // 2, 3 * size // 4),  # bottom
    ]

    combined_image = np.zeros((size, size))

    # Create Gaussian for each sensor
    for i, (pos_x, pos_y) in enumerate(positions):
        if i < len(sensor_values):
            # Map sensor value to standard deviation (1 to 50 pixels)
            sensor_val = sensor_values[i]

            # Scale standard deviation based on sensor value
            min_old, max_old, min_new, max_new = -10.0, 400.0, 1.0, 50.0
            std_dev = (max_new - min_new) / (max_old - min_old) * (np.clip(sensor_val, min_old, max_old) - min_old) + min_new

            # Scale magnitude of Gaussian
            min_old, max_old, min_new, max_new = -10.0, 400.0, 0.0, 1.0
            magnitude = (max_new - min_new) / (max_old - min_old) * (np.clip(sensor_val, min_old, max_old) - min_old) + min_new

            # Create individual Gaussian
            gaussian = magnitude * np.exp(-((X - pos_x)**2 + (Y - pos_y)**2) / (2 * std_dev**2))
            # Scale by absolute sensor value for intensity
            intensity = np.clip(abs(sensor_val) / 30.0, 0, 1)
            combined_image += gaussian * intensity
    return combined_image



sensor = Sensor(namespace="anyskin", sensor_config=AnySkinSensorConfig())
sensor.wait_until_ready()

plt.ion()  # Turn on interactive mode
fig = plt.figure(figsize=(16, 8))

# 2D plot (left side)
ax1 = fig.add_subplot(121)
im = ax1.imshow(np.zeros((256, 256)), cmap='hot', vmin=0, vmax=1)
ax1.set_title('2D Gaussian Intensity (256x256)')
plt.colorbar(im, ax=ax1, shrink=0.6)

# 3D plot (right side)
ax2 = fig.add_subplot(122, projection='3d')
x_3d = np.arange(256)
y_3d = np.arange(256)
X_3d, Y_3d = np.meshgrid(x_3d, y_3d)

print(f"X_3d shape: {X_3d.shape}, Y_3d shape: {Y_3d.shape}")
surf = ax2.plot_surface(X_3d, Y_3d, np.zeros((256, 256)), cmap='hot', alpha=0.8)

ax2.set_title('3D Gaussian Surface')
ax2.set_xlabel('X')
ax2.set_ylabel('Y')
ax2.set_zlim(0, 1)

duration = 50  # Run for 30 seconds
dt = 0.05  # Update every 50ms for smoother animation

sensor.calibrate_to_zero()  # Calibrate the sensor to zero

start_time = time.time()

std_dev = 1.0  # Initial standard deviation for Gaussian image

try:
    while time.time() - start_time < duration:
        sensor_values = sensor.value  # Get individual sensor values (should be 5 values)

        # Update 2D plot
        gaussian_img = create_multi_gaussian_image(256, sensor_values)
        im.set_array(gaussian_img)

        # Update 3D plot
        ax2.clear()
        ax2.plot_surface(X_3d, Y_3d, gaussian_img)
        ax2.set_zlim(0, 1)

        if len(sensor_values) >= 5:
            title = f'5 Gaussians - C:{sensor_values[0]:.1f} L:{sensor_values[1]:.1f} R:{sensor_values[2]:.1f} T:{sensor_values[3]:.1f} B:{sensor_values[4]:.1f}'
        else:
            title = f'Sensor values: {[f"{v:.1f}" for v in sensor_values]}'

        ax1.set_title(f'2D View - {title}')
        plt.pause(dt)
        print(f"Sensor values: {[f'{v:.2f}' for v in sensor_values]}")

except KeyboardInterrupt:
    print("Stopped by user")
finally:
    plt.ioff()  # Turn off interactive mode
    plt.show()
