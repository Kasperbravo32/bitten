import matplotlib.pyplot as plt
import math
import numpy as np

def movingaverage (values, window):
    weights = np.repeat(1.0, window)/window
    sma = np.convolve(values, weights, 'valid')
    return sma

can_acc_x = []
can_acc_y = []
can_gyro = []
can_temp = []
can_gyro_vert = []
can_angle = []

can_file = open("DiggingTestOnExcavator.log")
# can_id can be 600, 608, 610, 618, 620 or 628
# Pitch / Roll: 600
# Boom 1 : 608
# Boom 2 : 610
# Stick  : 618
# Bucket : 620
# Tilt   : 628
# Robot Sensor is 628... maybe :>

can_id = "608"
can_id = can_id + "#"

for line in can_file:
    if can_id in line:
        line = line.split(" ")[2]
        line = line.replace(can_id, "")
        payload = line.replace("\n","")
        mux = payload[0:2]
        if mux == "01":
            temp_str = payload[4:6] + payload[2:4]
            acc_x = float.fromhex(temp_str)
            acc_x = np.int16(acc_x) / float(10000)
            can_acc_x.append(acc_x)

            temp_str = payload[8:10] + payload[6:8]
            acc_y = float.fromhex(temp_str)
            acc_y = np.int16(acc_y) / float(10000)
            can_acc_y.append(acc_y)

            temp_str = payload[12:14] + payload[10:12]
            gyro = float.fromhex(temp_str)
            gyro = np.int16(gyro) / float(1000)
            gyro = math.degrees(gyro)
            can_gyro.append(gyro)

            temp_str = payload[14:16]
            temp = float.fromhex(temp_str)
            temp = np.int8(temp)
            can_temp.append(temp)

        if mux == "06":
            temp_str = payload[4:6] + payload[2:4]
            gyro_vert = float.fromhex(temp_str)
            gyro_vert = np.int16(gyro_vert) / float(1000)
            gyro_vert = math.degrees(gyro_vert)
            can_gyro_vert.append(gyro_vert)
            
can_file.close()

avg_acc_x = movingaverage(can_acc_x, 1)
avg_acc_y = movingaverage(can_acc_y, 1)

number_of_samples = len(avg_acc_x)
for i in range(0, number_of_samples):
    angle = math.atan2(avg_acc_y[i], avg_acc_x[i])
    angle = math.degrees(angle) + 90
    if angle > 90:
        angle = angle - 180
    elif angle < -90:
        angle = angle + 180
    can_angle.append(angle)

plt.figure(1)
plt.subplot(311)
plt.grid(True)
plt.plot(avg_acc_x, color = "black")
# plt.title("X-accelerometer [g]")
plt.xlabel("Samples [n]")
plt.ylabel("X-acceleration [g]")
# plt.ylim(-0.095, -0.03)

plt.subplot(312)
plt.grid(True)
plt.plot(avg_acc_y, color = "black")
# plt.title("Y-accelerometer [g]")
plt.xlabel("Samples [n]")
plt.ylabel("Y-acceleration [g]")
# plt.ylim(0.000, 0.045)

plt.subplot(313)
plt.grid(True)
plt.plot(can_angle, color = "black")
# plt.title("Angle [degrees]")
plt.xlabel("Samples [n]")
plt.ylabel("Angle [degrees]")
# plt.ylim(-40, 5)

plt.tight_layout(pad=0, w_pad=0, h_pad=-0.6)

plt.show()
