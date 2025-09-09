import tkinter as tk
from tkinter import simpledialog, ttk, messagebox
import utils
from utils import *

class RobotArmApp:
    def __init__(self, master, port, baudrate):
        self.master = master
        self.master.title("MyCobot280 Control System")
        self.master.geometry("800x600")
        self.master.minsize(600, 500)
        self.master.configure(bg='#f0f0f0')

        self.port = port
        self.baudrate = baudrate
        self.my_cobot = MyCobot(self.port, self.baudrate)

        self.poses = []
        self.gripper_states = []
        self.joints_angles_control = True
        self.recording_window = None  # Track the recording window
        self.connection_status = "Connected"  # Track connection status

        # Create and setup the GUI
        self.setup_gui()

    def setup_gui(self):
        # Configure style
        style = ttk.Style()
        style.theme_use('clam')
        
        # Create main frame with padding
        main_frame = ttk.Frame(self.master, padding="20")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configure grid weights for responsive design
        self.master.columnconfigure(0, weight=1)
        self.master.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        
        # Title Label
        title_label = ttk.Label(main_frame, text="MyCobot280 Kinesthetic Teaching", 
                               font=('Arial', 16, 'bold'))
        title_label.grid(row=0, column=0, columnspan=2, pady=(0, 20))
        
        # Status Frame
        status_frame = ttk.LabelFrame(main_frame, text="System Status", padding="10")
        status_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 15))
        status_frame.columnconfigure(1, weight=1)
        
        # Connection Status
        ttk.Label(status_frame, text="Connection:").grid(row=0, column=0, sticky=tk.W, padx=(0, 10))
        self.status_label = ttk.Label(status_frame, text=self.connection_status, 
                                     foreground="green", font=('Arial', 9, 'bold'))
        self.status_label.grid(row=0, column=1, sticky=tk.W)
        
        # Control Mode Status
        ttk.Label(status_frame, text="Control Mode:").grid(row=1, column=0, sticky=tk.W, padx=(0, 10))
        self.mode_label = ttk.Label(status_frame, text="Joint Angles", 
                                   foreground="blue", font=('Arial', 9, 'bold'))
        self.mode_label.grid(row=1, column=1, sticky=tk.W)
        
        # Recorded Commands Status
        ttk.Label(status_frame, text="Recorded Commands:").grid(row=2, column=0, sticky=tk.W, padx=(0, 10))
        self.commands_label = ttk.Label(status_frame, text="0 poses, 0 gripper actions", 
                                       foreground="orange", font=('Arial', 9, 'bold'))
        self.commands_label.grid(row=2, column=1, sticky=tk.W)
        
        # Control Buttons Frame
        control_frame = ttk.LabelFrame(main_frame, text="Robot Control", padding="15")
        control_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 15))
        control_frame.columnconfigure(0, weight=1)
        
        # Create styled buttons with better spacing and sizing
        button_style = {'width': 25, 'padding': (10, 8)}
        
        self.start_button = ttk.Button(control_frame, text="🎯 Start Teaching Session", 
                                      command=self.teach_robot_path, **button_style)
        self.start_button.grid(row=0, column=0, pady=8, padx=10, sticky=(tk.W, tk.E))
        
        self.run_button = ttk.Button(control_frame, text="▶️ Execute Recorded Path", 
                                    command=self.run_path, **button_style)
        self.run_button.grid(row=1, column=0, pady=8, padx=10, sticky=(tk.W, tk.E))
        
        self.reset_button = ttk.Button(control_frame, text="🔄 Reset All Commands", 
                                      command=self.reset_commands, **button_style)
        self.reset_button.grid(row=2, column=0, pady=8, padx=10, sticky=(tk.W, tk.E))
        
        # Information Frame
        info_frame = ttk.LabelFrame(main_frame, text="Information & Logs", padding="10")
        info_frame.grid(row=3, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(0, 15))
        info_frame.columnconfigure(0, weight=1)
        info_frame.rowconfigure(1, weight=1)
        
        # Info text area with scrollbar
        self.info_text = tk.Text(info_frame, height=8, width=60, wrap=tk.WORD, 
                                font=('Consolas', 9), bg='#f8f8f8', fg='#333333')
        scrollbar = ttk.Scrollbar(info_frame, orient=tk.VERTICAL, command=self.info_text.yview)
        self.info_text.configure(yscrollcommand=scrollbar.set)
        
        self.info_text.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 5))
        scrollbar.grid(row=1, column=1, sticky=(tk.N, tk.S))
        
        # Add initial information
        self.log_message("System initialized successfully")
        self.log_message("Ready for kinesthetic teaching")
        self.log_message("Click 'Start Teaching Session' to begin recording robot movements")
        
        # Bottom frame for exit button
        bottom_frame = ttk.Frame(main_frame)
        bottom_frame.grid(row=4, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(10, 0))
        bottom_frame.columnconfigure(0, weight=1)
        
        self.close_button = ttk.Button(bottom_frame, text="❌ Exit Application", 
                                      command=self.close_program, style='Accent.TButton')
        self.close_button.grid(row=0, column=0, pady=10)

    def log_message(self, message):
        """Add a message to the info log with timestamp"""
        import time
        timestamp = time.strftime("%H:%M:%S")
        self.info_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.info_text.see(tk.END)
        self.master.update_idletasks()

    def update_commands_status(self):
        """Update the recorded commands status"""
        pose_count = len([p for p in self.poses if p != INVALID_VALUE])
        gripper_count = len([g for g in self.gripper_states if g != INVALID_VALUE])
        self.commands_label.config(text=f"{pose_count} poses, {gripper_count} gripper actions")

    def teach_robot_path(self):
        # Create a custom dialog for control type selection
        dialog = tk.Toplevel(self.master)
        dialog.title("Select Control Mode")
        dialog.geometry("400x250")
        dialog.resizable(False, False)
        dialog.transient(self.master)
        dialog.grab_set()
        
        # Center the dialog
        dialog.geometry("+%d+%d" % (self.master.winfo_rootx() + 200, self.master.winfo_rooty() + 150))
        
        frame = ttk.Frame(dialog, padding="20")
        frame.pack(fill=tk.BOTH, expand=True)
        
        ttk.Label(frame, text="Choose Control Type", font=('Arial', 12, 'bold')).pack(pady=(0, 15))
        
        control_var = tk.StringVar(value="joint")
        
        ttk.Radiobutton(frame, text="Joint Angles Control (Recommended)", 
                       variable=control_var, value="joint").pack(anchor=tk.W, pady=5)
        ttk.Radiobutton(frame, text="Cartesian Coordinates Control", 
                       variable=control_var, value="cartesian").pack(anchor=tk.W, pady=5)
        
        ttk.Label(frame, text="\n⚠️ Warning: Servos will be released during teaching.\nPlease hold the robot carefully!", 
                 foreground="red", font=('Arial', 10)).pack(pady=15)
        
        button_frame = ttk.Frame(frame)
        button_frame.pack(pady=10)
        
        result = [None]
        
        def ok_pressed():
            result[0] = control_var.get()
            dialog.destroy()
            
        def cancel_pressed():
            result[0] = None
            dialog.destroy()
        
        ttk.Button(button_frame, text="Start Teaching", command=ok_pressed).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Cancel", command=cancel_pressed).pack(side=tk.LEFT, padx=5)
        
        dialog.wait_window()
        
        if result[0] is None:
            self.log_message("Teaching session cancelled by user")
            return
            
        if result[0] == "cartesian":
            self.joints_angles_control = False
            self.mode_label.config(text="Cartesian Coordinates")
            self.log_message("Control mode set to: Cartesian Coordinates")
        else:
            self.joints_angles_control = True
            self.mode_label.config(text="Joint Angles")
            self.log_message("Control mode set to: Joint Angles")

        # Confirm start
        if messagebox.askyesno("Confirm Start", 
                              "Are you ready to start the teaching session?\n\nThe robot servos will be released and you'll need to manually guide the robot."):
            self.start_teaching_session()
        else:
            self.log_message("Teaching session cancelled")

    def start_teaching_session(self):
        """Start the actual teaching session"""
        try:
            self.recording_window = tk.Toplevel(self.master)
            self.recording_window.title("Recording Session - Teaching Mode Active")
            self.recording_window.geometry("500x400")
            self.recording_window.resizable(True, True)
            self.recording_window.configure(bg='#f0f0f0')
            
            # Center the window
            self.recording_window.geometry("+%d+%d" % (self.master.winfo_rootx() + 50, self.master.winfo_rooty() + 50))
            
            main_frame = ttk.Frame(self.recording_window, padding="20")
            main_frame.pack(fill=tk.BOTH, expand=True)
            
            # Title
            ttk.Label(main_frame, text="🎯 Teaching Session Active", 
                     font=('Arial', 14, 'bold'), foreground="green").pack(pady=(0, 15))
            
            # Instructions
            instructions = """
Instructions:
1. Manually move the robot to desired positions
2. Click 'Save Position' to record current pose
3. Use gripper controls to open/close gripper
4. Click 'Finish Teaching' when done
            """
            ttk.Label(main_frame, text=instructions, font=('Arial', 10), 
                     justify=tk.LEFT, background='#ffffcc', relief=tk.SOLID, padding=10).pack(pady=(0, 15))
            
            # Control buttons
            button_frame = ttk.Frame(main_frame)
            button_frame.pack(fill=tk.X, pady=10)
            
            ttk.Button(button_frame, text="💾 Save Current Position", 
                      command=self.save_pose, width=25).pack(pady=5)
            ttk.Button(button_frame, text="🔧 Close Gripper (Catch)", 
                      command=self.close_gripper_command, width=25).pack(pady=5)
            ttk.Button(button_frame, text="🖐️ Open Gripper (Release)", 
                      command=self.open_gripper_command, width=25).pack(pady=5)
            ttk.Button(button_frame, text="✅ Finish Teaching Session", 
                      command=self.stop_recording, width=25, style='Accent.TButton').pack(pady=15)
            
            # Status in recording window
            self.recording_status = ttk.Label(main_frame, text="Ready to record positions...", 
                                            font=('Arial', 10), foreground="blue")
            self.recording_status.pack(pady=10)
            
            # Release servos
            self.my_cobot.release_all_servos()
            self.log_message("Servos released - Robot is now in teaching mode")
            self.log_message("You can now manually move the robot")
            
        except Exception as e:
            self.log_message(f"Error starting teaching session: {str(e)}")
            messagebox.showerror("Error", f"Failed to start teaching session: {str(e)}")

    def save_pose(self):
        try:
            joints_pose = get_current_joints_pose(self.my_cobot)
            cartesian_pose = get_current_cartesian_pose(self.my_cobot)
            
            if self.joints_angles_control:
                pose = joints_pose
                pose_type = "Joint Angles"
            else:
                pose = cartesian_pose
                pose_type = "Cartesian Position"

            if pose and all(isinstance(value, (int, float)) for value in pose):
                self.poses.append(pose)
                self.gripper_states.append(INVALID_VALUE)
                pose_count = len([p for p in self.poses if p != INVALID_VALUE])
                
                self.log_message(f"Position {pose_count} saved ({pose_type}): {[round(x, 2) for x in pose]}")
                self.update_commands_status()
                
                if hasattr(self, 'recording_status'):
                    self.recording_status.config(text=f"✅ Position {pose_count} saved successfully!")
                    
            else:
                self.log_message("Failed to save position - invalid pose data received")
                if hasattr(self, 'recording_status'):
                    self.recording_status.config(text="❌ Failed to save position - try again")
                    
        except Exception as e:
            self.log_message(f"Error saving pose: {str(e)}")
            if hasattr(self, 'recording_status'):
                self.recording_status.config(text="❌ Error saving position")

    def open_gripper_command(self):
        try:
            self.gripper_states.append(OPEN_GRIPPER)
            self.poses.append(INVALID_VALUE)
            gripper_count = len([g for g in self.gripper_states if g != INVALID_VALUE])
            
            self.log_message(f"Gripper OPEN command {gripper_count} recorded")
            self.update_commands_status()
            
            if hasattr(self, 'recording_status'):
                self.recording_status.config(text=f"✅ Gripper OPEN command recorded!")
                
        except Exception as e:
            self.log_message(f"Error recording gripper open command: {str(e)}")

    def close_gripper_command(self):
        try:
            self.gripper_states.append(CLOSE_GRIPPER)
            self.poses.append(INVALID_VALUE)
            gripper_count = len([g for g in self.gripper_states if g != INVALID_VALUE])
            
            self.log_message(f"Gripper CLOSE command {gripper_count} recorded")
            self.update_commands_status()
            
            if hasattr(self, 'recording_status'):
                self.recording_status.config(text=f"✅ Gripper CLOSE command recorded!")
                
        except Exception as e:
            self.log_message(f"Error recording gripper close command: {str(e)}")

    def reset_commands(self):
        if len(self.poses) > 0 or len(self.gripper_states) > 0:
            if messagebox.askyesno("Confirm Reset", 
                                 "Are you sure you want to reset all recorded commands?\nThis action cannot be undone."):
                self.poses = []
                self.gripper_states = []
                self.update_commands_status()
                self.log_message("All recorded commands have been reset")
            else:
                self.log_message("Reset cancelled by user")
        else:
            self.log_message("No commands to reset")

    def run_path(self):
        if len(self.poses) == 0 and len(self.gripper_states) == 0:
            messagebox.showwarning("No Commands", "No recorded commands to execute.\nPlease record some positions first.")
            self.log_message("Cannot execute - no recorded commands")
            return
            
        if messagebox.askyesno("Confirm Execution", 
                              "Are you sure you want to execute the recorded path?\n\nThe robot will move automatically."):
            try:
                self.log_message("Starting path execution...")
                self.log_message("Moving to home position...")
                move_to_home_position(self.my_cobot)
                
                self.log_message("Opening gripper...")
                open_gripper(self.my_cobot)
                
                self.log_message("Executing recorded path...")
                execute_robot_path(self.my_cobot, self.poses, self.gripper_states, 
                                 DEFAULT_ROBOT_SPEED, DEFAULT_MOVEMENT_MODE, 
                                 DEFAULT_TIMEOUT, self.joints_angles_control)
                
                self.log_message("✅ Path execution completed successfully!")
                messagebox.showinfo("Success", "Path execution completed successfully!")
                
            except Exception as e:
                error_msg = f"Error during path execution: {str(e)}"
                self.log_message(f"❌ {error_msg}")
                messagebox.showerror("Execution Error", error_msg)
        else:
            self.log_message("Path execution cancelled by user")

    def stop_recording(self):
        try:
            self.log_message("Ending teaching session...")
            move_to_home_position(self.my_cobot)
            self.log_message("Robot returned to home position")
            
            if self.recording_window:
                self.recording_window.destroy()
                self.recording_window = None
                
            total_poses = len([p for p in self.poses if p != INVALID_VALUE])
            total_gripper = len([g for g in self.gripper_states if g != INVALID_VALUE])
            
            self.log_message(f"Teaching session completed: {total_poses} positions, {total_gripper} gripper commands recorded")
            
        except Exception as e:
            self.log_message(f"Error ending teaching session: {str(e)}")

    def close_program(self):
        if messagebox.askyesno("Exit Application", "Are you sure you want to exit the application?"):
            try:
                self.log_message("Shutting down system...")
                move_to_home_position(self.my_cobot)
                self.my_cobot = None
                self.log_message("System shutdown complete")
            except Exception as e:
                self.log_message(f"Error during shutdown: {e}")
            finally:
                self.master.quit()
                self.master.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotArmApp(root, PORT, BAUD_RATE)
    root.mainloop()
