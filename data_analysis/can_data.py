import matplotlib.pyplot as plt
import math
import numpy

can_acc_x = []
can_acc_y = []
can_gyro = []
can_temp = []
can_gyro_vert = []
can_angle = []

can_file = open("candump-gravemaskine.log")
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

            if acc_y != 0:
                angle = acc_x/acc_y
                angle = math.degrees(math.atan(angle))
                can_angle.append(angle)

        if mux == "06":
            temp_str = payload[4:6] + payload[2:4]
            gyro_vert = float.fromhex(temp_str)
            gyro_vert = gyro_vert / float(1000)
            can_gyro_vert.append(gyro_vert)
            
can_file.close()

plt.figure(1)
plt.grid(True)
plt.plot(can_acc_x)

plt.figure(2)
plt.grid(True)
plt.plot(can_acc_y)

plt.figure(3)
plt.grid(True)
plt.plot(can_gyro)

plt.figure(4)
plt.grid(True)
plt.plot(can_temp)

plt.figure(5)
plt.grid(True)
plt.plot(can_angle)

plt.figure(6)
plt.grid(True)
plt.plot(can_gyro_vert)

plt.show()
