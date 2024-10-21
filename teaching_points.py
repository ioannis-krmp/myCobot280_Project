import tkinter as tk
from tkinter import simpledialog
import utils
from utils import *

class RobotArmApp:
    def __init__(self, master, port, baudrate):
        self.master = master
        self.master.title("MyCobot280 Control")
        self.master.geometry("400x300")
        self.master.minsize(400, 300)

        self.port = port
        self.baudrate = baudrate
        self.my_cobot = MyCobot(self.port, self.baudrate)

        self.poses = []
        self.gripper_states = []
        self.joints_angles_control = True
        self.recording_window = None  # Track the recording window

        # Create buttons for GUI control
        self.start_button = tk.Button(master, text = "Teaching points to robot", command = self.teach_robot_path)
        self.start_button.pack(pady = 10)

        self.run_button = tk.Button(master, text = "Run robot program", command = self.run_path)
        self.run_button.pack(pady = 10)

        self.reset_button = tk.Button(master, text = "Reset commands", command = self.reset_commands)
        self.reset_button.pack(pady = 10)

        self.close_button = tk.Button(master, text = "Close", command = self.close_program)
        self.close_button.pack(pady = 10)

    def teach_robot_path(self):
        control_type_str = tk.simpledialog.askstring("Choose Control Type", "Enter 'Y' for Joint Control or 'N' for Cartesian Control")
        if control_type_str == 'N' or control_type_str == 'n':
            self.joints_angles_control = False
        else:
            self.joints_angles_control = True

        response = tk.simpledialog.askstring("WARNING", "Get ready, servos will be released. Please, hold the robot carefully. Should we continue? 'Y/N' ")
        if response == "Y" or response == "y":
            self.recording_window = tk.Toplevel(self.master)
            self.recording_window.title("Recording commands")
            self.recording_window.geometry("400x300")
            self.recording_window.minsize(400, 300)
            self.my_cobot.release_all_servos()

            # Create buttons to control the procedure of teaching points in robot
            save_pose_button = tk.Button(self.recording_window, text = "Save pose", command = self.save_pose)
            save_pose_button.pack(pady = 10)

            close_gripper_button = tk.Button(self.recording_window, text = "Catch object", command = self.close_gripper_command)
            close_gripper_button.pack(pady = 10)

            open_gripper_button = tk.Button(self.recording_window, text = "Release object", command = self.open_gripper_command)
            open_gripper_button.pack(pady = 10)

            stop_button = tk.Button(self.recording_window, text = "Stop recording", command = self.stop_recording)
            stop_button.pack(pady = 10)

    def save_pose(self):
        joints_pose = get_current_joints_pose(self.my_cobot)
        cartesian_pose = get_current_cartesian_pose(self.my_cobot)
        if self.joints_angles_control:
            pose = joints_pose
        else:
            pose = cartesian_pose

        if pose and all(isinstance(value, (int, float)) for value in pose):
            self.poses.append(pose)
            self.gripper_states.append(INVALID_VALUE)  # In this way, I will check if I should execute command, poses and should have same length

    def open_gripper_command(self):
        self.gripper_states.append(OPEN_GRIPPER)
        self.poses.append(INVALID_VALUE)

    def close_gripper_command(self):
        self.gripper_states.append(CLOSE_GRIPPER)
        self.poses.append(INVALID_VALUE)

    def reset_commands(self):
        self.poses = []
        self.gripper_states = []

    def run_path(self):
        move_to_home_position(self.my_cobot)
        open_gripper(self.my_cobot)
        execute_robot_path(self.my_cobot, self.poses, self.gripper_states, DEFAULT_ROBOT_SPEED, DEFAULT_MOVEMENT_MODE, DEFAULT_TIMEOUT, self.joints_angles_control)

    def stop_recording(self):
        move_to_home_position(self.my_cobot)
        self.recording_window.destroy()

    def close_program(self):
        try:
            move_to_home_position(self.my_cobot)
            self.my_cobot = None
        except Exception as e:
            print(f"Error while shutting down: {e}")
        finally:
            self.master.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotArmApp(root, PORT, BAUD_RATE)
    root.mainloop()