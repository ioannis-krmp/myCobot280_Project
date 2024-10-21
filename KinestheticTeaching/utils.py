import time
from pymycobot.mycobot import MyCobot

PORT = "/dev/ttyTHS1"
BAUD_RATE = 1000000
DEFAULT_ROBOT_SPEED = 50
DEFAULT_GRIPPER_SPEED = 70
DEFAULT_GRIPPER_TYPE = 1  # adaptive
DEFAULT_GRIPPER_VALUE_OPEN = 90
DEFAULT_GRIPPER_VALUE_CLOSE = 10
OPEN_GRIPPER = 1
CLOSE_GRIPPER = 0
DEFAULT_MOVEMENT_MODE = 0
HOME_POSITION_JOINT_ANGLES = [0, 0, 0, 0, 0, 45]
INVALID_VALUE = -280  # mycobot280
DEFAULT_TIMEOUT = 7

def is_angle_close(desired_angle, current_angle, threshold):
    return abs(desired_angle - current_angle) < threshold

def is_coordinate_close(desired_value, encoder_value, threshold):
    return abs(desired_value - encoder_value) < threshold

def are_all_angles_close(desired_angles, current_angles, threshold):
    return all([is_angle_close(desired_angles[i], current_angles[i], threshold) for i in range(len(desired_angles))])

def are_all_coordinates_close(desired_coordinates, current_coordinates, threshold):
    return all([is_coordinate_close(desired_coordinates[i], current_coordinates[i], threshold) for i in range(len(desired_coordinates))])

def send_coordinates_synchronous(mycobot, coords, speed, mode, timeout):
    mycobot.send_coords(coords, speed, mode)
    if mycobot.is_moving() == -1:
        print("error, the robot seems that it cannot execute the command")
        return False
    if mycobot.is_moving() == 0:
        current_coordinates = mycobot.get_coords()
        if are_all_angles_close(current_coordinates[3:6], coords[3:6], 3) and are_all_coordinates_close(current_coordinates[:3], coords[:3], 6):
            print("The robot is not moving, probably already in the desired position")
            return True
        else:
            print("Command failed to be executed. It will be sent again")
            time.sleep(3) #Give some time to recover from a possible fault situation
            mycobot.send_coords(coords, speed, mode)
    
    if mycobot.is_moving() == 1:
        start_time = time.time()
        while (time.time() - start_time < timeout):
            current_coordinates = mycobot.get_coords()
            if are_all_angles_close(current_coordinates[3:6], coords[3:6], 3) and are_all_coordinates_close(current_coordinates[:3], coords[:3], 6):
                print(f"Successfully reached the position, desired position: {coords}, current position: {current_coordinates}")
                return True
            time.sleep(0.5)  # I could use pass. Small sleep to prevent excessive CPU usage
        print(f"Failed to reach the position, desired position: {coords}, current position: {current_coordinates}")
        return False

def send_angles_synchronous(mycobot, angles, speed, mode, timeout):
    mycobot.send_angles(angles, speed)
    if mycobot.is_moving() == 1:
        print("error, the robot seems that it cannot execute the command")
        return False
    if mycobot.is_moving() == 0:
        current_joint_angles = mycobot.get_angles()
        if are_all_angles_close(angles, current_joint_angles, 3):
            print("The robot is not moving, probably already in the desired position")
            return True
        else:
            print("Command failed to be executed. It will be sent again")
            time.sleep(3) #Give some time to recover from a possible fault situation
            mycobot.send_angles(angles, speed)   

    if mycobot.is_moving() == 1:
        start_time = time.time()
        while (time.time() - start_time < timeout):
            current_joint_angles = mycobot.get_angles()
            if are_all_angles_close(angles, current_joint_angles, 2):
                print(f"Successfully reached the position, desired position: {angles}, current position: {current_joint_angles}")
                return True
            time.sleep(0.5)
        print(f"Failed to reach the position, desired position: {angles}, current position: {current_joint_angles}")
        return False

def get_current_cartesian_pose(mycobot):
    current_cartesian_pose = mycobot.get_coords()
    print("Cartesian pose is: ", current_cartesian_pose)
    return current_cartesian_pose

def get_current_joints_pose(mycobot):
    current_joints_pose = mycobot.get_angles()
    print("Joints pose is: ", current_joints_pose)
    return current_joints_pose

def move_to_home_position(mycobot):
    mycobot.sync_send_angles(HOME_POSITION_JOINT_ANGLES, DEFAULT_ROBOT_SPEED)

def close_gripper(mycobot):
    #mycobot.set_gripper_state(CLOSE_GRIPPER, DEFAULT_GRIPPER_SPEED)
    mycobot.set_gripper_value(DEFAULT_GRIPPER_VALUE_CLOSE, DEFAULT_GRIPPER_SPEED)
    data = mycobot.is_gripper_moving()
    if data == -1:
        print("Error of closing the gripper")
    elif data == 0:
        print("Gripper is not moving, probably is already close")
    elif data == 1:
        print("Gripper is moving to the desired position")
        while(mycobot.is_gripper_moving()):
            #waiting to finish the command execution
            time.sleep(0.2) #Small sleep to prevent excessive CPU usage
    else:
        print("Unknown error in gripper status")
    time.sleep(1.5)  #Safety factor. In maual they use 3 seconds but we constantly check the gripper if is moving

def open_gripper(mycobot):
    mycobot.set_gripper_value(DEFAULT_GRIPPER_VALUE_OPEN, DEFAULT_GRIPPER_SPEED)
    data = mycobot.is_gripper_moving()
    if data == -1:
        print("Error of opening the gripper")
    elif data == 0:
        print("Gripper is not moving, probably is already in the desired position")
    elif data == 1:
        print("Gripper is moving to the desired position")
        while(mycobot.is_gripper_moving()):
            #waiting to finish the command execution
            time.sleep(0.2) #Small sleep to prevent excessive CPU usage
    else:
        print("Unknown error in gripper status")
    time.sleep(1.5) #Safety factor(overkill). In maual they use 3 seconds but we constantly check the gripper if is moving

def execute_robot_path(mycobot, poses, gripper_states, robot_speed, movement_mode, timeout, joints_angles_control):
    if len(poses) != len(gripper_states):
        print("Number of poses does not match number of gripper states")
        move_to_home_position(mycobot)
        return False
    for i in range(len(gripper_states)):
        if gripper_states[i] == CLOSE_GRIPPER:
            close_gripper(mycobot)
        elif gripper_states[i] == OPEN_GRIPPER:
            open_gripper(mycobot)
        elif gripper_states[i] == INVALID_VALUE:
            if len(poses[i]) == 6:
                if joints_angles_control == 1:
                    result = send_angles_synchronous(mycobot, poses[i], robot_speed, movement_mode, timeout)
                else:
                    result = send_coordinates_synchronous(mycobot, poses[i], robot_speed, movement_mode, timeout)
            else:
                print("Invalid pose")
        else:
            print("Invalid gripper state")
        time.sleep(1.5) #Let some time pass due to oscillations of braking
    
    move_to_home_position(mycobot)
