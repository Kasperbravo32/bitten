import matplotlib.pyplot as plt

actual_pos1 = []
actual_pos2 = []
actual_pos3 = []
actual_pos4 = []
actual_pos5 = []
actual_pos6 = []

goal_pos1 = []
goal_pos2 = []
goal_pos3 = []
goal_pos4 = []
goal_pos5 = []
goal_pos6 = []

goal_time1 = []
goal_time2 = []
goal_time3 = []
goal_time4 = []
goal_time5 = []
goal_time6 = []

number_of_points = 0

test_file = open("STUPIDF1LE.txt")
for line in test_file:
    if "GOAL" in line:
        goal_pos1.append(float(line.split(" ")[1]))
        goal_time1.append(number_of_points)
        goal_pos2.append(float(line.split(" ")[2]))
        goal_time2.append(number_of_points)
        goal_pos3.append(float(line.split(" ")[3]))
        goal_time3.append(number_of_points)
        goal_pos4.append(float(line.split(" ")[4]))
        goal_time4.append(number_of_points)
        goal_pos5.append(float(line.split(" ")[5]))
        goal_time5.append(number_of_points)
        goal_pos6.append(float(line.split(" ")[6]))
        goal_time6.append(number_of_points)
    if "ACTUAL" in line:
        actual_pos1.append(float(line.split(" ")[1]))
        actual_pos2.append(float(line.split(" ")[2]))
        actual_pos3.append(float(line.split(" ")[3]))
        actual_pos4.append(float(line.split(" ")[4]))
        actual_pos5.append(float(line.split(" ")[5]))
        actual_pos6.append(float(line.split(" ")[6]))
        number_of_points = number_of_points + 1
        
test_file.close()

goal_time1.append(number_of_points)
goal_time2.append(number_of_points)
goal_time3.append(number_of_points)
goal_time4.append(number_of_points)
goal_time5.append(number_of_points)
goal_time6.append(number_of_points)

goal_time1.pop(0)
goal_time2.pop(0)
goal_time3.pop(0)
goal_time4.pop(0)
goal_time5.pop(0)
goal_time6.pop(0)

plt.figure(1)

plt.subplot(321)
plt.grid(True)
plt.plot(actual_pos1)
plt.plot(goal_time1, goal_pos1, marker = "x", linestyle = 'None', color = 'r')
# plt.title("Joint 1")
plt.xlabel("Points (n)")
plt.ylabel("Angles (rad)")

plt.subplot(322)
plt.grid(True)
plt.plot(actual_pos2)
plt.plot(goal_time2, goal_pos2, marker = "x", linestyle = 'None', color = 'r')
# plt.title("Actual positions")
plt.xlabel("Points (n)")
plt.ylabel("Angles (rad)")

plt.subplot(323)
plt.grid(True)
plt.plot(actual_pos3)
plt.plot(goal_time3, goal_pos3, marker = "x", linestyle = 'None', color = 'r')
# plt.title("Actual positions")
plt.xlabel("Points (n)")
plt.ylabel("Angles (rad)")

plt.subplot(324)
plt.grid(True)
plt.plot(actual_pos4)
plt.plot(goal_time4, goal_pos4, marker = "x", linestyle = 'None', color = 'r')
# plt.title("Actual positions")
plt.xlabel("Points (n)")
plt.ylabel("Angles (rad)")

plt.subplot(325)
plt.grid(True)
plt.plot(actual_pos5)
plt.plot(goal_time5, goal_pos5, marker = "x", linestyle = 'None', color = 'r')
# plt.title("Actual positions")
plt.xlabel("Points (n)")
plt.ylabel("Angles (rad)")

plt.subplot(326)
plt.grid(True)
plt.plot(actual_pos6)
plt.plot(goal_time6, goal_pos6, marker = "x", linestyle = 'None', color = 'r')
# plt.title("Actual positions")
plt.xlabel("Points (n)")
plt.ylabel("Angles (rad)")

plt.show()