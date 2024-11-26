import serial
import tkinter as tk
from tkinter import ttk, messagebox

# Configure the serial port
ser = serial.Serial('COM7', 115200, timeout=1)

# Variable to track gripper state (1: open, 0: close)
gripper_state = 0
recording = False
movements = []

def send_command(command):
    try:
        ser.write(command.encode())
        print(f"Sent: {command.strip()}")  # Debug: print sent command
        log_message(f"Command Sent: {command.strip()}")
    except Exception as e:
        log_message(f"Failed to send command: {e}", is_error=True)

def move_robot(x, y, z):
    x_rounded = round(x)
    y_rounded = round(y)
    z_rounded = round(z)
    command = f"move {x_rounded} {y_rounded} {z_rounded}\n"
    send_command(command)

    # Record the movement if recording is active
    if recording:
        movements.append(("move", x_rounded, y_rounded, z_rounded))

def set_gripper_z(gripperz):
    gripperz_rounded = round(float(gripperz))
    command = f"gripperz {gripperz_rounded}\n"
    send_command(command)

    # Record the gripper Z movement if recording is active
    if recording:
        movements.append(("gripperz", gripperz_rounded))

def toggle_gripper():
    global gripper_state
    if gripper_state == 0:
        command = "open_gripper\n"
        gripper_state = 1
        gripper_button.config(text="Gripper: Open")
        log_message("Gripper opened.")
    else:
        command = "close_gripper\n"
        gripper_state = 0
        gripper_button.config(text="Gripper: Closed")
        log_message("Gripper closed.")
    send_command(command)

    # Record the gripper open/close state if recording is active
    if recording:
        movements.append(("gripper_state", gripper_state))

def set_speed():
    speed = int(speed_entry.get())
    if 100 <= speed <= 5000:
        command = f"set_speed {speed}\n"
        send_command(command)
        log_message(f"Speed set to: {speed}")
    else:
        log_message("Speed value must be between 100 and 5000.", is_error=True)

def set_accel():
    accel = int(accel_entry.get())
    if 100 <= accel <= 5000:
        command = f"set_accel {accel}\n"
        send_command(command)
        log_message(f"Acceleration set to: {accel}")
    else:
        log_message("Acceleration value must be between 100 and 5000.", is_error=True)

def stop_robot():
    command = "stop\n"
    send_command(command)
    log_message("Robot stopped.")

# Function to update label text based on slider value
def update_label(slider, label):
    label.config(text=f"{label.cget('text').split(':')[0]}: {round(slider.get())}")

# Function to log messages in the feedback area
def log_message(message, is_error=False):
    if is_error:
        feedback_area.config(fg="red")
    else:
        feedback_area.config(fg="black")
    feedback_area.insert(tk.END, message + "\n")
    feedback_area.see(tk.END)  # Scroll to the end

# Modern UI design with ttk widgets
root = tk.Tk()
root.title("SCARA Robot Control")
root.geometry("800x500")  # Landscape window size

# Create a main frame
main_frame = ttk.Frame(root)
main_frame.pack(fill=tk.BOTH, expand=True)

# Robot Movement Section
robot_frame = ttk.LabelFrame(main_frame, text="Robot Movement", padding=(10, 10))
robot_frame.pack(side=tk.LEFT, padx=10, pady=10, fill='both', expand=True)

def update_position(event=None):
    move_robot(slider_x.get(), slider_y.get(), slider_z.get())

# X Position Control
x_frame = ttk.Frame(robot_frame)
x_frame.pack(pady=5)
slider_x = ttk.Scale(x_frame, from_=-150, to=150, orient=tk.HORIZONTAL, length=250)
slider_x.grid(row=0, column=0)
x_label = ttk.Label(x_frame, text="X Position: 0")
x_label.grid(row=1, column=0)
slider_x.bind("<Motion>", lambda event: update_label(slider_x, x_label))
slider_x.bind("<ButtonRelease-1>", update_position)

# Y Position Control
y_frame = ttk.Frame(robot_frame)
y_frame.pack(pady=5)
slider_y = ttk.Scale(y_frame, from_=-700, to=700, orient=tk.HORIZONTAL, length=250)
slider_y.grid(row=0, column=0)
y_label = ttk.Label(y_frame, text="Y Position: 0")
y_label.grid(row=1, column=0)
slider_y.bind("<Motion>", lambda event: update_label(slider_y, y_label))
slider_y.bind("<ButtonRelease-1>", update_position)

# Z Position Control
z_frame = ttk.Frame(robot_frame)
z_frame.pack(pady=5)
slider_z = ttk.Scale(z_frame, from_=0, to=80, orient=tk.HORIZONTAL, length=250)
slider_z.grid(row=0, column=0)
z_label = ttk.Label(z_frame, text="Z Position: 0")
z_label.grid(row=1, column=0)
slider_z.bind("<Motion>", lambda event: update_label(slider_z, z_label))
slider_z.bind("<ButtonRelease-1>", update_position)

# Gripper Z Control
gripperz_frame = ttk.Frame(robot_frame)
gripperz_frame.pack(pady=5)
slider_gripper_z = ttk.Scale(gripperz_frame, from_=0, to=180, orient=tk.HORIZONTAL, length=250)
slider_gripper_z.grid(row=0, column=0)
gripperz_label = ttk.Label(gripperz_frame, text="Gripper Z Position: 0")
gripperz_label.grid(row=1, column=0)
slider_gripper_z.bind("<Motion>", lambda event: update_label(slider_gripper_z, gripperz_label))
slider_gripper_z.bind("<ButtonRelease-1>", lambda event: set_gripper_z(slider_gripper_z.get()))

# Manual Input Section
manual_frame = ttk.LabelFrame(main_frame, text="Manual Input", padding=(10, 10))
manual_frame.pack(side=tk.LEFT, padx=10, pady=10, fill='both', expand=True)

# Position control textboxes and buttons for each axis
def set_position(axis):
    try:
        position = int(position_entries[axis].get())
        if axis == 'x':
            slider_x.set(position)
        elif axis == 'y':
            slider_y.set(position)
        elif axis == 'z':
            slider_z.set(position)
        move_robot(slider_x.get(), slider_y.get(), slider_z.get())
    except ValueError:
        log_message("Please enter a valid integer.", is_error=True)

position_entries = {}
for axis in ['x', 'y', 'z']:
    frame = ttk.Frame(manual_frame)
    frame.pack(pady=5)
    label = ttk.Label(frame, text=f"{axis.upper()} Position:")
    label.grid(row=0, column=0)
    entry = ttk.Entry(frame, width=10)
    entry.grid(row=0, column=1)
    position_entries[axis] = entry
    button = ttk.Button(frame, text="Set", command=lambda a=axis: set_position(a))
    button.grid(row=0, column=2)

# Speed Control
speed_frame = ttk.Frame(manual_frame)
speed_frame.pack(pady=5)
ttk.Label(speed_frame, text="Speed:").grid(row=0, column=0)
speed_entry = ttk.Entry(speed_frame, width=10)
speed_entry.insert(0, "500")  # Default value
speed_entry.grid(row=0, column=1)
ttk.Button(speed_frame, text="Set Speed", command=set_speed).grid(row=0, column=2)

# Acceleration Control
accel_frame = ttk.Frame(manual_frame)
accel_frame.pack(pady=5)
ttk.Label(accel_frame, text="Acceleration:").grid(row=0, column=0)
accel_entry = ttk.Entry(accel_frame, width=10)
accel_entry.insert(0, "500")  # Default value
accel_entry.grid(row=0, column=1)
ttk.Button(accel_frame, text="Set Acceleration", command=set_accel).grid(row=0, column=2)

# Control Buttons Section
control_frame = ttk.LabelFrame(main_frame, text="Controls", padding=(10, 10))
control_frame.pack(side=tk.RIGHT, padx=10, pady=10, fill='y')

# Gripper Control Button
gripper_button = ttk.Button(control_frame, text="Gripper: opened", command=toggle_gripper)
gripper_button.pack(pady=10)

# Stop Button for all movements
ttk.Button(control_frame, text="STOP", command=stop_robot).pack(pady=10)

# Record/Replay functionality
def record_movements():
    global recording, movements
    if recording:
        recording = False
        record_button.config(text="Start Recording")
        log_message("Recording stopped.")
    else:
        recording = True
        movements = []
        record_button.config(text="Stop Recording")
        log_message("Recording started.")

def replay_movements():
    if not movements:
        log_message("No movements recorded.", is_error=True)
        return
    log_message("Replaying recorded movements.")
    for movement in movements:
        if movement[0] == "move":
            move_robot(movement[1], movement[2], movement[3])
        elif movement[0] == "gripperz":
            set_gripper_z(movement[1])
        elif movement[0] == "gripper_state":
            global gripper_state
            if gripper_state != movement[1]:
                toggle_gripper()

record_button = ttk.Button(control_frame, text="Start Recording", command=record_movements)
record_button.pack(pady=10)

ttk.Button(control_frame, text="Replay Movements", command=replay_movements).pack(pady=10)

# Feedback Area at the bottom of the window
feedback_area = tk.Text(root, height=5, width=80, wrap=tk.WORD)
feedback_area.pack(side=tk.BOTTOM, padx=10, pady=10, fill=tk.X)
feedback_area.config(state=tk.NORMAL)  # Make it editable

root.mainloop()
