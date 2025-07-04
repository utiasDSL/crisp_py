"""Example using the sensor module to read data from a sensor."""

from crisp_py.sensors.sensor import Sensor
from crisp_py.sensors.sensor_config import AnySkinSensorConfig
import matplotlib.pyplot as plt
import time

sensor = Sensor(namespace="anyskin", sensor_config=AnySkinSensorConfig())
sensor.wait_until_ready()

#%%

duration = 10
dt = 0.01  # seconds

t = []
data = []
start_time = time.time()

sensor.calibrate_to_zero()  # Calibrate the sensor to zero

while time.time() - start_time < duration:
    t.append(time.time() - start_time)
    data.append(sensor.value)
    time.sleep(dt)

# %%

# Smooth the data using a simple moving average
N = 10
data = [sum(data[max(0, i-N):i+1]) / min(i+1, N+1) for i in range(len(data))]

plt.plot(t, data)
plt.xlabel('Time (s)')
plt.ylabel('Sensor Value')
plt.title('Sensor Data Over Time')
plt.grid()
plt.show()
