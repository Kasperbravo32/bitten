import matplotlib.pyplot as plt
import math

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

deviation1 = []
deviation2 = []
deviation3 = []
deviation4 = []
deviation5 = []
deviation6 = []

number_of_points = 0

test_file = open("logfile.txt.txt")
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

goal_time1.pop(0)
goal_time2.pop(0)
goal_time3.pop(0)
goal_time4.pop(0)
goal_time5.pop(0)
goal_time6.pop(0)

goal_time1.append(number_of_points)
goal_time2.append(number_of_points)
goal_time3.append(number_of_points)
goal_time4.append(number_of_points)
goal_time5.append(number_of_points)
goal_time6.append(number_of_points)

i = 0
while i < len(goal_pos1):
    number_at_goal = goal_time1[i]
    actual_at_goal = actual_pos1[number_at_goal-1]
    goal_value = goal_pos1[i]
    deviation = math.degrees(abs(abs(goal_value) - abs(actual_at_goal)))
    deviation1.append(deviation)
    i = i + 1

i = 0
while i < len(goal_pos2):
    number_at_goal = goal_time2[i]
    actual_at_goal = actual_pos2[number_at_goal-1]
    goal_value = goal_pos2[i]
    deviation = math.degrees(abs(abs(goal_value) - abs(actual_at_goal)))
    deviation2.append(deviation)
    i = i + 1

i = 0
while i < len(goal_pos3):
    number_at_goal = goal_time3[i]
    actual_at_goal = actual_pos3[number_at_goal-1]
    goal_value = goal_pos3[i]
    deviation = math.degrees(abs(abs(goal_value) - abs(actual_at_goal)))
    deviation3.append(deviation)
    i = i + 1

i = 0
while i < len(goal_pos4):
    number_at_goal = goal_time4[i]
    actual_at_goal = actual_pos4[number_at_goal-1]
    goal_value = goal_pos4[i]
    deviation = math.degrees(abs(abs(goal_value) - abs(actual_at_goal)))
    deviation4.append(deviation)
    i = i + 1

i = 0
while i < len(goal_pos5):
    number_at_goal = goal_time5[i]
    actual_at_goal = actual_pos5[number_at_goal-1]
    goal_value = goal_pos5[i]
    deviation = math.degrees(abs(abs(goal_value) - abs(actual_at_goal)))
    deviation5.append(deviation)
    i = i + 1

i = 0
while i < len(goal_pos6):
    number_at_goal = goal_time6[i]
    actual_at_goal = actual_pos6[number_at_goal-1]
    goal_value = goal_pos6[i]
    deviation = math.degrees(abs(abs(goal_value) - abs(actual_at_goal)))
    deviation6.append(deviation)
    i = i + 1

average_deviation1 = sum(deviation1)/len(deviation1)
average_deviation2 = sum(deviation2)/len(deviation2)
average_deviation3 = sum(deviation3)/len(deviation3)
average_deviation4 = sum(deviation4)/len(deviation4)
average_deviation5 = sum(deviation5)/len(deviation5)
average_deviation6 = sum(deviation6)/len(deviation6)
print(average_deviation1)
print(average_deviation2)
print(average_deviation3)
print(average_deviation4)
print(average_deviation5)
print(average_deviation6)