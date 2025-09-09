import time
from pymycobot.mycobot import MyCobot

print("=== MyCobot280 Pick and Place Operation ===")
print("Initializing system...")

# Initialize the current line variable
current_line = 12

# Open and read file
try:
    with open('first_task.txt', 'r') as f:
        ft_list = f.readlines()[current_line:]
    print("✅ Task file loaded successfully")
except FileNotFoundError:
    print("❌ Error: The file 'first_task.txt' was not found")
    exit(1)
except IOError:
    print("❌ Error: An I/O error occurred while trying to read the file")
    exit(1)

# Initialize a MyCobot instance with the appropriate port and baud rate
print("🔗 Connecting to MyCobot280...")
mc = MyCobot('/dev/ttyTHS1', 1000000)

# Home position
print("🏠 Moving to home position...")
mc.sync_send_angles([0, 0, 0, 0, 0, 45], 50)  # Instead of send_angles() due to asynchronous return

# Set the gripper state and speed
print("🔧 Initializing gripper...")
mc.set_gripper_value(90, 70)

print("⏳ Waiting for system to stabilize...")
time.sleep(3)

print("🚀 Starting pick and place sequence...")

for line in ft_list:
    current_line += 1

    if line.find("set_coords:") != -1:
        try:
            coords_str = line.split(": ")[1]
            x, y, z = map(float, coords_str.split(","))
            print(f"📍 Moving to coordinates: [{x}, {y}, {z}]")
            mc.sync_send_coords([x, y, z, -172.0, -2.5, 134.0], 30, 0)
            # mc.send_coords([x, y, z, -172.0, -2.5, 134.0], 30, 0)
            # time.sleep(5)
        except (ValueError, IndexError) as e:
            print(f"❌ Line {current_line} seems to be invalid: {e}")
            continue
    elif line.find("set_coords_pick_pos:") != -1:
        try:
            coords_str = line.split(": ")[1]
            x, y, z = map(float, coords_str.split(","))
            print(f"🎯 Moving to pick position: [{x}, {y}, {z}]")
            mc.sync_send_coords([x, y, z, -170.96, -1.73, 133.26], 30, 0)
            # mc.send_coords([x, y, z, -170.96, -1.73, 133.26], 30, 0)
            # time.sleep(5)
        except (ValueError, IndexError) as e:
            print(f"❌ Line {current_line} seems to be invalid: {e}")
            continue
    elif line.find("set_coords_place_pos: ") != -1:
        try:
            coords_str = line.split(":")[1]
            x, y, z = map(float, coords_str.split(","))
            print(f"📦 Moving to place position: [{x}, {y}, {z}]")
            mc.sync_send_coords([x, y, z, -173.22, -2.5, 136.73], 30, 0)
            # mc.send_coords([x, y, z, -173.22, -2.5, 136.73], 30, 0)
            # time.sleep(5)
        except (ValueError, IndexError) as e:
            print(f"❌ Line {current_line} seems to be invalid: {e}")
            continue
    elif line.find("set_gripper_state:") != -1:
        try:
            index = line.find(":")
            state = int(line[index + 1])
            if state == 0:
                print("🖐️ Opening gripper...")
                mc.set_gripper_value(90, 70)
            elif state == 1:
                print("🔧 Closing gripper...")
                mc.set_gripper_value(10, 70)
            else:
                print(f"❌ Error: invalid state({state}). Valid state is 0 or 1")
        except (ValueError, IndexError) as e:
            print(f"❌ Line {current_line} seems to be invalid: {e}")
            continue
    else:
        print(f"⚠️ Cannot detect a valid command on line {current_line}")

# Return to home position
print("🏠 Returning to home position...")
mc.sync_send_angles([0, 0, 0, 0, 0, 45], 50)
print("✅ Pick and place operation completed successfully!")
