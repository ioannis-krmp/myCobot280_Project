import time
import numpy as np
from pymycobot.mycobot import MyCobot

print("=== MyCobot280 Forward Kinematics Verification ===")
print("This script verifies forward kinematics using Denavit-Hartenberg parameters\n")

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

print("📊 DH Parameters loaded successfully")

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
    print("=" * 80)
    print("📐 FORWARD KINEMATICS VERIFICATION RESULTS")
    print("=" * 80)
    print(f"🎯 User Input -> Joint Angles: J1:{user_angles_data[0]:.2f}°, J2:{user_angles_data[1]:.2f}°, J3:{user_angles_data[2]:.2f}°, J4:{user_angles_data[3]:.2f}°, J5:{user_angles_data[4]:.2f}°, J6:{user_angles_data[5]:.2f}°")
    print(f"🧮 Calculated Position from User Input -> X:{end_effector_position_user_data[0][3]:.2f}mm, Y:{end_effector_position_user_data[1][3]:.2f}mm, Z:{end_effector_position_user_data[2][3]:.2f}mm")
    print(f"🔍 Encoder Feedback -> Joint Angles: J1:{joints_data_encoder[0]:.2f}°, J2:{joints_data_encoder[1]:.2f}°, J3:{joints_data_encoder[2]:.2f}°, J4:{joints_data_encoder[3]:.2f}°, J5:{joints_data_encoder[4]:.2f}°, J6:{joints_data_encoder[5]:.2f}°")
    print(f"📍 Encoder Feedback -> Cartesian: X:{cartesian_data_encoder[0]:.2f}mm, Y:{cartesian_data_encoder[1]:.2f}mm, Z:{cartesian_data_encoder[2]:.2f}mm")
    print(f"✅ Calculated Position from Encoder -> X:{end_effector_position_encoder_data[0][3]:.2f}mm, Y:{end_effector_position_encoder_data[1][3]:.2f}mm, Z:{end_effector_position_encoder_data[2][3]:.2f}mm")
    
    # Calculate differences for verification
    x_diff = abs(end_effector_position_user_data[0][3] - end_effector_position_encoder_data[0][3])
    y_diff = abs(end_effector_position_user_data[1][3] - end_effector_position_encoder_data[1][3])
    z_diff = abs(end_effector_position_user_data[2][3] - end_effector_position_encoder_data[2][3])
    
    print(f"📊 Position Differences -> ΔX:{x_diff:.2f}mm, ΔY:{y_diff:.2f}mm, ΔZ:{z_diff:.2f}mm")
    
    if x_diff < 5 and y_diff < 5 and z_diff < 5:
        print("✅ Forward kinematics verification: PASSED (differences < 5mm)")
    else:
        print("⚠️ Forward kinematics verification: ATTENTION (differences > 5mm)")
    
    print("=" * 80)
    time.sleep(2)

# Initialize the current line variable
current_line = 5

# Read file
try:
    with open('second_task.txt', 'r') as f:
        ft_list = f.readlines()[current_line:]
    print("✅ Task file loaded successfully")
except FileNotFoundError:
    print("❌ Error: The file 'second_task.txt' was not found")
    exit(1)
except IOError:
    print("❌ Error: An I/O error occurred while trying to read the file")
    exit(1)

# Initialize myCobot instance with the appropriate port and baud rate
print("🔗 Connecting to MyCobot280...")
mc = MyCobot('/dev/ttyTHS1', 1000000)

# Send angles (joint1, joint2, joint3, joint4, joint5, joint6 : speed:range 0-100)
print("🏠 Moving to initial position...")
mc.send_angles([0, 0, 0, 45, 30], 30) # Instead of send_angles() due to asynchronous return
mc.set_gripper_state(0, 70)  # Set the gripper state and speed

print("⏳ Waiting for system to stabilize...")
time.sleep(2)

print("🚀 Starting forward kinematics verification sequence...")

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
            print(f"🎯 Moving to joint angles: [{j1:.1f}°, {j2:.1f}°, {j3:.1f}°, 0.0°, 0.0°, 45.0°]")
            mc.sync_send_angles(j_angles, 30)
        except (ValueError, IndexError) as e:
            print(f"❌ Line {current_line} seems to be invalid: {e}")
            continue

    print("⏳ Waiting for movement to complete...")
    time.sleep(2)

    print("🔍 Reading encoder data...")
    end_effector_position_user_data = forward_kinematics(mycobot_dh_parameters, np.radians(j_angles))

    current_angles_encoder = mc.get_angles()
    current_coords_encoder = mc.get_coords()

    end_effector_position_encoder_data = forward_kinematics(mycobot_dh_parameters, np.radians(current_angles_encoder))

    log_messages(j_angles, end_effector_position_user_data, current_angles_encoder, current_coords_encoder, end_effector_position_encoder_data)

# Return to home position
print("🏠 Returning to home position...")
mc.sync_send_angles([0, 0, 0, 0, 0, 45], 30)
print("✅ Forward kinematics verification completed successfully!")
