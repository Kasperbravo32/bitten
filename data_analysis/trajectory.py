import matplotlib.pyplot as plt
import math
import numpy as np

actual_pos = [[],[],[],[],[],[]]
goal_pos = [[],[],[],[],[],[]]
goal_time = [[],[],[],[],[],[]]
deviations = [[],[],[],[],[],[]]
average_deviations = []
min_deviations = []
max_deviations = []
len_deviations = []

number_of_points = 0

test_file = open("logfile.txt")

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

for i in range(0,6):
    goal_time[i].pop(0)
    goal_time[i].append(number_of_points)

for i in range(0,6):
    j = 0
    while j < len(goal_pos[i]):
        number_at_goal = goal_time[i][j]
        actual_at_goal = actual_pos[i][number_at_goal-1]
        goal_value = goal_pos[i][j]
        deviation = abs(abs(goal_value) - abs(actual_at_goal))
        deviations[i].append(deviation)
        j = j + 1

for i in range(0,6):
    average_deviations.append(sum(deviations[i])/len(deviations[i]))
print("average deviations:", average_deviations)

for i in range(0,6):
    min_deviations.append(min(deviations[i]))
print("min deviations:", min_deviations)

for i in range(0,6):
    max_deviations.append(max(deviations[i]))
print("max deviations:", max_deviations)

for i in range(0,6):
    len_deviations.append(len(deviations[i]))
print("len deviations:", len_deviations)


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

plt.figure(2)

x = np.arange(6)
markerline, stemlines, baseline = plt.stem(x, average_deviations, linefmt = "-.", markerfmt = "_")
plt.setp(markerline, color='b', linewidth=5, marker = "x")
plt.setp(stemlines, color='black', linewidth=1)
plt.setp(baseline, color='black', linewidth=1)
plt.title("Average joint deviations")
plt.xticks(x, ('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6'))
plt.ylabel("Angles [degrees]")

plt.show()