"""Example using the sensor module to read data from a sensor."""
# Launch in terminal:
# python scripts/mock/mock_force_torque_sensor.py

from crisp_py.sensors.sensor import make_sensor
import matplotlib.pyplot as plt
import time

sensor = make_sensor("mock_force_torque_sensor", namespace="sensor")
sensor.wait_until_ready()

#%%


duration = 20
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
