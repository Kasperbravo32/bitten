import matplotlib.pyplot as plt

def hex_to_dec(hex_number):
    mult = float(1)
    every_other = 0
    result = float(0)
    for letter in hex_number:
        if letter == "A":
            letter = 10
        elif letter == "B":
            letter = 11
        elif letter == "C":
            letter = 12
        elif letter == "D":
            letter = 13
        elif letter == "E":
            letter = 14
        elif letter == "F":
            letter = 15
        number = float(letter)
        if every_other == 0:
            result = result + ( number * ( mult * float(16) ) )
            every_other = 1
        else:
            result = result + ( number * ( mult / float(16) ) )
            every_other = 0
        mult = mult * float(16)
    return result

values = []

can_file = open("candump-gravemaskine.log")

for line in can_file:
    if "600#01" in line:
        value = line.split(" ")[2]
        value = value.replace("600#01", "")
        value = value.replace("\n","")
        value = hex_to_dec(value)
        values.append(value)

can_file.close()

plt.figure(1)
plt.grid(True)
plt.plot(values)
# plt.title("Distance measurements")
# plt.xlabel("Time (s)")
# plt.ylabel("Distance (m)")

plt.show()
