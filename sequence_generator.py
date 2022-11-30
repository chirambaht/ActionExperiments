import random
import time
speed_array = []
time_array = []

# get program run time
start_time = time.time()
v = input("Press q at any point to finish, or press \"r x\" for x random values: ( c to begin): ")
user_time = time.time()

# random number seed is the time difference between user input and program start
random.seed(user_time - start_time)

if (v == "q"):
    exit()
elif (v[0] == "r"):
    if len(v) < 3:
        print("Invalid input")
        exit()
    g = int(v.split(" ")[-1])
    for i in range(g):
        speed_array.append(random.randint(1, 100))
        time_array.append(random.randint(1, 10) * 1000)
else:
    while True:
        
        # get input from user for a speed (0-100)
        speed = input("Enter a speed (0-100): ")
        if speed == "q":
            break
        speed = int(speed)
        
        # get input from user for time (in seconds)
        time = input("Enter a time (in seconds): ")
        if time == "q":
            break
        time = float(time) * 1000

        # add speed to speed_array
        speed_array.append(speed)
        # add time to time_array
        time_array.append(time)
        print("")


print("\n======== ==== = = == = = ==== ========\n")
# print out speed in C++ suitable format
print("uint8_t sequence_speed[] = {", end="")
for i in range(len(speed_array)):
    print(speed_array[i]  , end="")
    if i != len(speed_array) - 1:
        print(", ", end="")
print("};")

# print out time in C++ suitable format
print("uint16_t sequence_time[] = {", end="")
for i in range(len(time_array)):
    print(time_array[i], end="")
    if i != len(time_array) - 1:
        print(", ", end="")
print("};\n")

# print out length of sequence
sum = 0
for i in time_array:
    sum += i

print(f"Your program will run for {round(sum/1000.0, 2)} seconds")
