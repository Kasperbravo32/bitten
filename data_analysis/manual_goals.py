import matplotlib.pyplot as plt
import math
import numpy as np

actual_pos = [[],[],[],[],[],[]]
goal_pos = [[],[],[],[],[],[]]
goal_time = [[],[],[],[],[],[]]

max_rotation = [3.14, 2.57, 2.53, 4.71, 2.44, 4.71]
for i in range(0,6):
        max_rotation[i] = math.degrees(max_rotation[i]) - 5

min_rotation = [-3.14, -2.27, -2.53, -4.71, -2.01, -4.71]
for i in range(0,6):
        min_rotation[i] = math.degrees(min_rotation[i]) + 5

number_of_points = 0

#read from file
test_file = open("logfile1.txt")

for line in test_file:
    if "GOAL" in line:
        for i in range(0,6):
            goal_pos[i].append(math.degrees(float(line.split(" ")[i+1])))
            goal_time[i].append(number_of_points)
    if "ACTUAL" in line:
        for i in range(0,6):
            actual_pos[i].append(math.degrees(float(line.split(" ")[i+1])))
        number_of_points = number_of_points + 1

test_file.close()

#remove dublicate goals
for i in range(0,6):
        last_goal = 0
        index = 0
        length_of_list = len(goal_pos[i])
        for j in range (0, length_of_list):
                if last_goal == goal_pos[i][index]:
                        last_goal = goal_pos[i][index]
                        goal_pos[i].pop(index)
                        goal_time[i].pop(index)
                else:
                        last_goal = goal_pos[i][index]
                        index = index + 1

#calculate delay
delay_up = [[],[],[],[],[],[]]
delay_down = [[],[],[],[],[],[]]
for i in range(0,6):
        length_of_list = len(goal_pos[i])
        for j in range(0, length_of_list):
                if goal_pos[i][j] >= max_rotation[i]:
                        k = goal_time[i][j]
                        last_pos = actual_pos[i][k]
                        delay_counter = 0
                        while actual_pos[i][k] <= last_pos:
                               delay_counter = delay_counter + 1
                               last_pos = actual_pos[i][k]
                               k = k + 1
                        delay_up[i].append(delay_counter * 20)
                elif goal_pos[i][j] <= min_rotation[i]:
                        k = goal_time[i][j]
                        last_pos = actual_pos[i][k]
                        delay_counter = 0
                        while actual_pos[i][k] >= last_pos:
                               delay_counter = delay_counter + 1
                               last_pos = actual_pos[i][k]
                               k = k + 1
                        delay_down[i].append(delay_counter * 20)
                
# for i in range(0,6):
#     goal_time[i].pop(0)
#     goal_time[i].append(number_of_points)

#plots
plt.figure(1)

plt.subplot(321)
plt.grid(True)
plt.plot(actual_pos[0], color = 'black')
plt.plot(goal_time[0], goal_pos[0], marker = "x", linestyle = 'None', color = 'b')
plt.title("Joint 1")
# plt.xlabel("Points [n]")
# plt.ylabel("Angles [degrees]")

plt.subplot(322)
plt.grid(True)
plt.plot(actual_pos[1], color = 'black')
plt.plot(goal_time[1], goal_pos[1], marker = "x", linestyle = 'None', color = 'b')
plt.title("Joint 2")
# plt.xlabel("Points [n]")
# plt.ylabel("Angles [degrees]")

plt.subplot(323)
plt.grid(True)
plt.plot(actual_pos[2], color = 'black')
plt.plot(goal_time[2], goal_pos[2], marker = "x", linestyle = 'None', color = 'b')
plt.title("Joint 3")
# plt.xlabel("Points [n]")
plt.ylabel("Angles [degrees]")

plt.subplot(324)
plt.grid(True)
plt.plot(actual_pos[3], color = 'black')
plt.plot(goal_time[3], goal_pos[3], marker = "x", linestyle = 'None', color = 'b')
plt.title("Joint 4")
# plt.xlabel("Points [n]")
plt.ylabel("Angles [degrees]")

plt.subplot(325)
plt.grid(True)
plt.plot(actual_pos[4], color = 'black')
plt.plot(goal_time[4], goal_pos[4], marker = "x", linestyle = 'None', color = 'b')
plt.title("Joint 5")
plt.xlabel("Points [n]")
# plt.ylabel("Angles [degrees]")

plt.subplot(326)
plt.grid(True)
plt.plot(actual_pos[5], color = 'black')
plt.plot(goal_time[5], goal_pos[5], marker = "x", linestyle = 'None', color = 'b')
plt.title("Joint 6")
plt.xlabel("Points [n]")
# plt.ylabel("Angles [degrees]")

plt.tight_layout(pad=0.01, w_pad=-1.1, h_pad=-0.6)

fig, ax = plt.subplots()

x = np.arange(6)
width = 0.15
markerline1, stemlines1, baseline1 = ax.stem(x - width, delay_up, linefmt = "-.", markerfmt = "_")
markerline2, stemlines2, baseline2 = ax.stem(x + width, delay_down, linefmt = "-.", markerfmt = "_")
# markerline, stemlines, baseline = plt.stem(x, delay_total, linefmt = "-.", markerfmt = "_")

plt.setp(markerline1, color='b', linewidth=5, marker = "x")
plt.setp(stemlines1, color='black', linewidth=1)
plt.setp(baseline1, color='black', linewidth=1)

plt.setp(markerline2, color='r', linewidth=5, marker = "x")
plt.setp(stemlines2, color='black', linewidth=1)
plt.setp(baseline2, color='black', linewidth=1)

plt.title("Average joint response delay")
plt.xticks(x, ('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6'))
plt.ylabel("Delay [ms]")
ax.legend((markerline1, markerline2), ('Moving up', 'Moving down'))

plt.show()