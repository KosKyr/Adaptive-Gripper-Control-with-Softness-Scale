import tkinter as tk
from tkinter import messagebox
import serial
import json
import threading
import time

SERIAL_PORT = "COM4"  # Update this as needed
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
        self.root.title("Adaptive Gripper GUI + PID Autotune")
        self.root.attributes('-fullscreen', True)
        self.root.configure(bg="#1e1e2f")

        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
        except serial.SerialException as e:
            messagebox.showerror("Serial Port Error", f"Failed to open {SERIAL_PORT}: {e}")
            self.root.destroy()
            return

        tk.Label(root, text="Adaptive Gripper Control Interface", font=("Helvetica", 24, "bold"), fg="#ffffff", bg="#1e1e2f").pack(pady=20)

        self.angle_label = tk.Label(root, text="Angle: ---", font=("Helvetica", 16), fg="#90ee90", bg="#1e1e2f")
        self.angle_label.pack()
        self.zfield_label = tk.Label(root, text="Z Field: ---", font=("Helvetica", 16), fg="#add8e6", bg="#1e1e2f")
        self.zfield_label.pack()
        self.velocity_label = tk.Label(root, text="Velocity: ---", font=("Helvetica", 16), fg="#f0e68c", bg="#1e1e2f")
        self.velocity_label.pack()
        self.class_label = tk.Label(root, text="Classification: ---", font=("Helvetica", 16), fg="#ffcccb", bg="#1e1e2f")
        self.class_label.pack(pady=10)

        button_frame = tk.Frame(root, bg="#1e1e2f")
        button_frame.pack(pady=20)

        btn_style = {"font": ("Helvetica", 14, "bold"), "width": 20, "height": 2}

        tk.Button(button_frame, text="Grip & Classify", bg="#28a745", fg="white", command=self.send_grip, **btn_style).grid(row=0, column=0, padx=10, pady=10)
        tk.Button(button_frame, text="Release", bg="#dc3545", fg="white", command=self.send_release, **btn_style).grid(row=0, column=1, padx=10, pady=10)
        tk.Button(button_frame, text="Start PID Autotune", bg="#ffc107", fg="black", command=self.start_autotune, **btn_style).grid(row=1, column=0, padx=10, pady=10)
        tk.Button(button_frame, text="Apply Saved PID", bg="#007bff", fg="white", command=self.apply_pid, **btn_style).grid(row=1, column=1, padx=10, pady=10)

        tk.Button(root, text="Exit Fullscreen / Quit", bg="#6c757d", fg="white", font=("Helvetica", 12), command=self.close).pack(pady=10)

        self.running = True
        self.update_thread = threading.Thread(target=self.update_labels)
        self.update_thread.start()

        self.root.bind("<Escape>", lambda event: self.close())

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
        print("⚙️ Starting autotune (5s)...")
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
                try:
                    parts = line.split()
                    self.angle_label.config(text=f"Angle: {parts[1]}")
                    self.zfield_label.config(text=f"Z Field: {parts[3]}")
                    self.velocity_label.config(text=f"Velocity: {parts[5]}")
                except:
                    pass
            elif line.startswith("Class:"):
                classification = line.split(":")[1].strip()
                self.class_label.config(text=f"Classification: {classification}")
            time.sleep(0.1)

    def close(self):
        self.running = False
        time.sleep(0.2)
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = GripperGUI(root)
    root.mainloop()
