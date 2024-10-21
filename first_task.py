import time
from pymycobot.mycobot import MyCobot

# Initialize the current line variable
current_line = 12

# Open and read file
try:
    with open('first_task.txt', 'r') as f:
        ft_list = f.readlines()[current_line:]
except FileNotFoundError:
    print("Error: The file was not found")
except IOError:
    print("Error: An I/O error occurred while trying to read the file")

# Initialize a MyCobot instance with the appropriate port and baud rate
mc = MyCobot('/dev/ttyTHS1', 1000000)

# Home position
mc.sync_send_angles([0, 0, 0, 0, 0, 45], 50)  # Instead of send_angles() due to asynchronous return

# Set the gripper state and speed
mc.set_gripper_value(90, 70)

time.sleep(3)

for line in ft_list:
    current_line += 1

    if line.find("set_coords:") != -1:
        try:
            coords_str = line.split(": ")[1]
            x, y, z = map(float, coords_str.split(","))
            mc.sync_send_coords([x, y, z, -172.0, -2.5, 134.0], 30, 0)
            # mc.send_coords([x, y, z, -172.0, -2.5, 134.0], 30, 0)
            # time.sleep(5)
        except (ValueError, IndexError) as e:
            print("Line ", current_line, "seems to be invalid: ", e)
            continue
    elif line.find("set_coords_pick_pos:") != -1:
        try:
            coords_str = line.split(": ")[1]
            x, y, z = map(float, coords_str.split(","))
            mc.sync_send_coords([x, y, z, -170.96, -1.73, 133.26], 30, 0)
            # mc.send_coords([x, y, z, -170.96, -1.73, 133.26], 30, 0)
            # time.sleep(5)
        except (ValueError, IndexError) as e:
            print("Line ", current_line, "seems to be invalid: ", e)
            continue
    elif line.find("set_coords_place_pos: ") != -1:
        try:
            coords_str = line.split(":")[1]
            x, y, z = map(float, coords_str.split(","))
            mc.sync_send_coords([x, y, z, -173.22, -2.5, 136.73], 30, 0)
            # mc.send_coords([x, y, z, -173.22, -2.5, 136.73], 30, 0)
            # time.sleep(5)
        except (ValueError, IndexError) as e:
            print("Line ", current_line, "seems to be invalid: ", e)
            continue
    elif line.find("set_gripper_state:") != -1:
        try:
            index = line.find(":")
            state = int(line[index + 1])
            if state == 0:
                mc.set_gripper_value(90, 70)
            elif state == 1:
                mc.set_gripper_value(10, 70)
            else:
                print(f"Error: invalid state({state}). Valid state is 0 or 1 ")
        except (ValueError, IndexError) as e:
            print("Line ", current_line, "seems to be invalid: ", e)
            continue
    else:
        print("Cannot detect a valid command")

# Return to home position
mc.sync_send_angles([0, 0, 0, 0, 0, 45], 50)
