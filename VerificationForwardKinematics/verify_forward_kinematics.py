import time
import numpy as np
from pymycobot.mycobot import MyCobot

# Mycobot280 robot DH parameters
# (theta, d, a, alpha offset # mm and radians)
# pi/2 = 1.5708
mycobot_dh_parameters = [
    [0.0, 131.22, 0.0, 1.5708, 0.0],
    [0.0, 0.0, -110.4, 0.0, -1.5708],
    [0.0, 0.0, -96.0, 0.0, 0.0],
    [0.0, 63.4, 0.0, 1.5708, -1.5708],
    [0.0, 75.05, 0.0, -1.5708, 1.5708],
    [0.0, 45.6, 0.0, 0.0, 0.0]
]

# Define transformation (theta, d, a, alpha)
def transform(theta, d, a, alpha):
    return np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
                     [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
                     [0, np.sin(alpha), np.cos(alpha), d],
                     [0, 0, 0, 1]])

# Define forward kinematics (dh_params, joint_angles)
def forward_kinematics(dh_params, joint_angles):
    # Function to compute overall transformation from the base to the end-effector
    T = np.eye(4)
    for i in range(len(joint_angles)):
        theta, d, a, alpha, offset = dh_params[i]  # Unpack DH parameters
        theta += joint_angles[i]
        T = T @ transform(theta + offset, d, a, alpha)  # Compute transformation
    return T  # Return the end-effector position (x, y, z)

# Log messages (user_angles_data, and end_effector_position_user_data, joints_data_encoder, cartesian_data_encoder, end_effector_position_encoder_data)
def log_messages(user_angles_data, end_effector_position_user_data, joints_data_encoder, cartesian_data_encoder, end_effector_position_encoder_data):
    print(f"User input -> Joints angles from user: J1:{user_angles_data[0]}, J2:{user_angles_data[1]}, J3:{user_angles_data[2]}, J4:{user_angles_data[3]}, J5:{user_angles_data[4]}, J6:{user_angles_data[5]}\n")
    print(f"End effector position calculation using user input -> X:{end_effector_position_user_data[0][3]}, Y:{end_effector_position_user_data[1][3]}, Z:{end_effector_position_user_data[2][3]}\n")
    print(f"Joints angles from encoders -> J1:{joints_data_encoder[0]}, J2:{joints_data_encoder[1]}, J3:{joints_data_encoder[2]}, J4:{joints_data_encoder[3]}, J5:{joints_data_encoder[4]}, J6:{joints_data_encoder[5]}\n")
    print(f"Cartesian coordinates from encoders -> X:{cartesian_data_encoder[0]}, Y:{cartesian_data_encoder[1]}, Z:{cartesian_data_encoder[2]}\n")
    print(f"End effector position calculation using encoders input -> X:{end_effector_position_encoder_data[0][3]}, Y:{end_effector_position_encoder_data[1][3]}, Z:{end_effector_position_encoder_data[2][3]}\n")
    print("\n")
    time.sleep(1)

# Initialize the current line variable
current_line = 5

# Read file
try:
    with open('second_task.txt', 'r') as f:
        ft_list = f.readlines()[current_line:]
except FileNotFoundError:
    print("Error: The file was not found")
except IOError:
    print("Error: An I/O error occurred while trying to read the file")

# Initialize myCobot instance with the appropriate port and baud rate
mc = MyCobot('/dev/ttyTHS1', 1000000)

# Send angles (joint1, joint2, joint3, joint4, joint5, joint6 : speed:range 0-100)
mc.send_angles([0, 0, 0, 45, 30], 30) # Instead of send_angles() due to asynchronous return
mc.set_gripper_state(0, 70)  # Set the gripper state and speed

time.sleep(2)

for line in ft_list:
    current_line += 1
    j_angles = []
    
    if line.find("set_angles:") == -1:
        continue
    if line.find("set_angles:") != -1:
        try:
            coords = line.split(":")[-1].split(",")
            j1, j2, j3 = map(float, coords.strip().split(", "))
            j_angles = [j1, j2, j3, 0.0, 0.0, 45.0]
            mc.sync_send_angles(j_angles, 30)
        except (ValueError, IndexError) as e:
            print(f"Line {current_line} seems to be invalid: {e}")
            continue

    time.sleep(2)

    end_effector_position_user_data = forward_kinematics(mycobot_dh_parameters, np.radians(j_angles))

    current_angles_encoder = mc.get_angles()
    current_coords_encoder = mc.get_coords()

    end_effector_position_encoder_data = forward_kinematics(mycobot_dh_parameters, np.radians(current_angles_encoder))

log_messages(j_angles, end_effector_position_user_data, current_angles_encoder, current_coords_encoder, end_effector_position_encoder_data)

# Return to home position
mc.sync_send_angles([0, 0, 0, 0, 0, 45], 30)
