import matplotlib.pyplot as plt

actual_pos = []
goal_pos = []

test_file = open("test1.txt")
for line in test_file:
    if "GOAL" in line:
        for x in range(1,6):
            temp = line.split("\t")[x]
            goal_pos.append(temp)
    if "ACTUAL" in line:
        for x in range(1,6):
            temp = line.split("\t")[x]
            actual_pos.append(temp)
    
plt.figure(1)
plt.grid(True)
plt.plot(actual_pos)
plt.title("Distance measurements")
plt.xlabel("Time (s)")
plt.ylabel("Distance (m)")

plt.show()

test_file.close()