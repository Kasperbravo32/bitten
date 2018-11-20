import matplotlib.pyplot as plt
import math
import numpy

can_acc_x = []
can_acc_y = []
can_gyro = []
can_temp = []
can_gyro_vert = []
can_angle = []

can_file = open("candump-2018-11-20_111156.log")
can_id = "628"
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
            acc_x = numpy.int16(acc_x) / float(10000)
            can_acc_x.append(acc_x)

            temp_str = payload[8:10] + payload[6:8]
            acc_y = float.fromhex(temp_str)
            acc_y = numpy.int16(acc_y) / float(10000)
            can_acc_y.append(acc_y)

            temp_str = payload[12:14] + payload[10:12]
            gyro = float.fromhex(temp_str)
            gyro = numpy.int16(gyro) / float(1000)
            gyro = math.degrees(gyro)
            can_gyro.append(gyro)

            temp_str = payload[14:16]
            temp = float.fromhex(temp_str)
            temp = numpy.int8(temp)
            can_temp.append(temp)

            angle = math.atan2(acc_y, acc_x)
            angle = math.degrees(angle) + 90
            can_angle.append(angle)

        if mux == "06":
            temp_str = payload[4:6] + payload[2:4]
            gyro_vert = float.fromhex(temp_str)
            gyro_vert = numpy.int16(gyro_vert) / float(1000)
            gyro_vert = math.degrees(gyro_vert)
            can_gyro_vert.append(gyro_vert)
            
can_file.close()

plt.figure(1)
plt.subplot(311)
plt.grid(True)
plt.plot(can_acc_x)
plt.title("X-accelerometer [G]")

plt.subplot(312)
plt.grid(True)
plt.plot(can_acc_y)
plt.title("Y-accelerometer [G]")

plt.subplot(313)
plt.grid(True)
plt.plot(can_angle)
plt.title("Angle [degrees]")

plt.tight_layout(pad=0, w_pad=0, h_pad=-0.6)

plt.show()
