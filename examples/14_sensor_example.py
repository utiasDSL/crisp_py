"""Example using the sensor module to read data from a sensor."""

from crisp_py.sensors.sensor import Sensor
from crisp_py.sensors.sensor_config import AnySkinSensor
import matplotlib.pyplot as plt
import time

sensor = Sensor(sensor_config=AnySkinSensor())

# %%

duration = 10
dt = 0.01  # seconds

t = []
data = []
start_time = time.time()

while time.time() - start_time < duration:
    t.append(time.time() - start_time)
    data.append(sensor.value)
    time.sleep(dt)

# %%

plt.plot(t, data)
plt.xlabel('Time (s)')
plt.ylabel('Sensor Value')
plt.title('Sensor Data Over Time')
plt.grid()
plt.show()






