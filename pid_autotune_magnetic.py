import tkinter as tk
from tkinter import messagebox
import serial
import json
import threading
import time

SERIAL_PORT = "COM3"  # Update for your system
BAUDRATE = 115200
PID_FILE = "pid_config.json"

def read_serial_data(ser):
    try:
        line = ser.readline().decode().strip()
        return line
    except:
        return ""

class GripperGUI:
    def __init__(self, root):
        self.root = root
        self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)

        self.root.title("Adaptive Gripper GUI + PID Autotune")
        self.root.geometry("400x300")

        self.angle_label = tk.Label(root, text="Angle: ---")
        self.angle_label.pack()
        self.zfield_label = tk.Label(root, text="Z Field: ---")
        self.zfield_label.pack()
        self.class_label = tk.Label(root, text="Classification: ---")
        self.class_label.pack()

        self.btn_grip = tk.Button(root, text="Grip & Classify", command=self.send_grip)
        self.btn_grip.pack(pady=5)

        self.btn_release = tk.Button(root, text="Release", command=self.send_release)
        self.btn_release.pack(pady=5)

        self.btn_autotune = tk.Button(root, text="Start PID Autotune", command=self.start_autotune)
        self.btn_autotune.pack(pady=5)

        self.btn_apply_pid = tk.Button(root, text="Apply Saved PID", command=self.apply_pid)
        self.btn_apply_pid.pack(pady=5)

        self.running = True
        self.update_thread = threading.Thread(target=self.update_labels)
        self.update_thread.start()

        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def send_grip(self):
        self.ser.write(b"G\n")

    def send_release(self):
        self.ser.write(b"O\n")

    def apply_pid(self):
        try:
            with open(PID_FILE, "r") as f:
                pid = json.load(f)
            cmd = f"P {pid['P']} {pid['I']} {pid['D']}\n"
            self.ser.write(cmd.encode())
            messagebox.showinfo("PID", f"Applied PID: {pid}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to apply PID: {e}")

    def start_autotune(self):
        threading.Thread(target=self.run_autotune).start()

    def run_autotune(self):
        print("⚙️ Starting autotune (5s or press Enter in terminal to stop)...")
        t0 = time.time()
        data = []
        self.ser.write(b"T-2.0\n")

        try:
            while time.time() - t0 < 5:
                line = read_serial_data(self.ser)
                if line.startswith("A:"):
                    parts = line.split()
                    angle = float(parts[1])
                    velocity = float(parts[5])
                    data.append((angle, velocity))
                time.sleep(0.05)
        finally:
            self.ser.write(b"T0\n")

        if not data:
            messagebox.showerror("Autotune", "No data collected")
            return

        angle_changes = [abs(data[i+1][0] - data[i][0]) for i in range(len(data)-1)]
        velocities = [v for _, v in data]
        p_est = max(angle_changes) * 0.5
        i_est = sum(velocities) * 0.1
        d_est = (max(velocities) - min(velocities)) * 0.05

        pid = {"P": round(p_est, 3), "I": round(i_est, 3), "D": round(d_est, 3)}
        with open(PID_FILE, "w") as f:
            json.dump(pid, f, indent=2)
        messagebox.showinfo("Autotune Done", f"PID saved: {pid}")

    def update_labels(self):
        while self.running:
            line = read_serial_data(self.ser)
            if line.startswith("A:"):
                parts = line.split()
                self.angle_label.config(text=f"Angle: {parts[1]}")
                self.zfield_label.config(text=f"Z Field: {parts[3]}")
            elif line.startswith("Class:"):
                self.class_label.config(text=f"Classification: {line.split(':')[1].strip()}")
            time.sleep(0.1)

    def close(self):
        self.running = False
        time.sleep(0.2)
        self.ser.close()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = GripperGUI(root)
    root.mainloop()
