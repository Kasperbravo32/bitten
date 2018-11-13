import matplotlib.pyplot as plt

actual_pos1 = []
actual_pos2 = []
actual_pos3 = []
actual_pos4 = []
actual_pos5 = []
actual_pos6 = []

goal_pos1 = 0
goal_pos2 = 0
goal_pos3 = 0
goal_pos4 = 0
goal_pos5 = 0
goal_pos6 = 0

goal_pos1_arr = []
goal_pos2_arr = []
goal_pos3_arr = []
goal_pos4_arr = []
goal_pos5_arr = []
goal_pos6_arr = []

goal_was_here = 0

test_file = open("STUPIDF1LE.txt")
for line in test_file:
    if "GOAL" in line:
        goal_pos1 = line.split(" ")[1]
        goal_pos1_arr.append(goal_pos1)
        goal_pos2 = line.split(" ")[2]
        goal_pos2_arr.append(goal_pos2)
        goal_pos3 = line.split(" ")[3]
        goal_pos3_arr.append(goal_pos3)
        goal_pos4 = line.split(" ")[4]
        goal_pos4_arr.append(goal_pos4)
        goal_pos5 = line.split(" ")[5]
        goal_pos5_arr.append(goal_pos5)
        goal_pos6 = line.split(" ")[6]
        goal_pos6_arr.append(goal_pos6)
        goal_was_here = 1
    if "ACTUAL" in line:
        actual_pos1.append(line.split(" ")[1])
        actual_pos2.append(line.split(" ")[2])
        actual_pos3.append(line.split(" ")[3])
        actual_pos4.append(line.split(" ")[4])
        actual_pos5.append(line.split(" ")[5])
        actual_pos6.append(line.split(" ")[6])
        if goal_was_here == 1:
            goal_pos1_arr.append(goal_pos1)
            goal_pos2_arr.append(goal_pos2)
            goal_pos3_arr.append(goal_pos3)
            goal_pos4_arr.append(goal_pos4)
            goal_pos5_arr.append(goal_pos5)
            goal_pos6_arr.append(goal_pos6)
            goal_was_here = 0
    
plt.figure(1)

plt.subplot(321)
plt.grid(True)
plt.plot(actual_pos1)
plt.plot(goal_pos1_arr)
plt.title("Actual positions")
plt.xlabel("Points (n)")
plt.ylabel("Angles (rad)")

plt.subplot(322)
plt.grid(True)
plt.plot(actual_pos2)
plt.plot(goal_pos2_arr)
plt.title("Actual positions")
plt.xlabel("Points (n)")
plt.ylabel("Angles (rad)")

plt.subplot(323)
plt.grid(True)
plt.plot(actual_pos3)
plt.plot(goal_pos3_arr)
plt.title("Actual positions")
plt.xlabel("Points (n)")
plt.ylabel("Angles (rad)")

plt.subplot(324)
plt.grid(True)
plt.plot(actual_pos4)
plt.plot(goal_pos4_arr)
plt.title("Actual positions")
plt.xlabel("Points (n)")
plt.ylabel("Angles (rad)")

plt.subplot(325)
plt.grid(True)
plt.plot(actual_pos5)
plt.plot(goal_pos5_arr)
plt.title("Actual positions")
plt.xlabel("Points (n)")
plt.ylabel("Angles (rad)")

plt.subplot(326)
plt.grid(True)
plt.plot(actual_pos6)
plt.plot(goal_pos6_arr)
plt.title("Actual positions")
plt.xlabel("Points (n)")
plt.ylabel("Angles (rad)")

plt.show()

test_file.close()