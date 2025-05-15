import tkinter as tk
from tkinter import ttk, messagebox
import serial
import json
import threading
import time
from PIL import Image, ImageTk

SERIAL_PORT = "COM4"  # Update this to match your system
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
        self.root.title("Adaptive Gripper Control with Softness Scale")
        self.root.attributes('-fullscreen', True)
        self.root.configure(bg="#2c3e50")

        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
        except serial.SerialException as e:
            messagebox.showerror("Serial Port Error", f"Failed to open {SERIAL_PORT}: {e}")
            self.root.destroy()
            return

        title = tk.Label(root, text="🦾 Gripper Interface + Softness Visualizer", font=("Helvetica", 26, "bold"),
                         fg="#ecf0f1", bg="#2c3e50")
        title.pack(pady=20)

        self.angle_label = tk.Label(root, text="Angle: ---", font=("Helvetica", 18), fg="#1abc9c", bg="#2c3e50")
        self.angle_label.pack()
        self.zfield_label = tk.Label(root, text="Z Field: ---", font=("Helvetica", 18), fg="#3498db", bg="#2c3e50")
        self.zfield_label.pack()
        self.velocity_label = tk.Label(root, text="Velocity: ---", font=("Helvetica", 18), fg="#f39c12", bg="#2c3e50")
        self.velocity_label.pack()
        self.class_label = tk.Label(root, text="Classification: ---", font=("Helvetica", 18), fg="#e74c3c", bg="#2c3e50")
        self.class_label.pack(pady=10)

        # --- Softness scale ---
        self.softness_label = tk.Label(root, text="Softness Indicator", font=("Helvetica", 18), fg="#ffffff", bg="#2c3e50")
        self.softness_label.pack()

        self.score_value_label = tk.Label(root, text="---", font=("Helvetica", 16, "bold"), fg="#9b59b6", bg="#2c3e50")
        self.score_value_label.pack()

        self.softness_bar = ttk.Progressbar(root, orient="horizontal", length=400, mode="determinate")
        self.softness_bar.pack(pady=10)
        self.softness_bar["value"] = 0

        style = ttk.Style()
        style.theme_use("default")
        style.configure("TProgressbar", foreground="#9b59b6", background="#9b59b6", thickness=30)

        button_frame = tk.Frame(root, bg="#2c3e50")
        button_frame.pack(pady=20)

        btn_style = {"font": ("Helvetica", 14, "bold"), "width": 20, "height": 2}

        tk.Button(button_frame, text="Grip & Classify", bg="#27ae60", fg="white", command=self.send_grip,
                  **btn_style).grid(row=0, column=0, padx=10, pady=10)
        tk.Button(button_frame, text="Release", bg="#c0392b", fg="white", command=self.send_release,
                  **btn_style).grid(row=0, column=1, padx=10, pady=10)
        tk.Button(button_frame, text="Start PID Autotune", bg="#f1c40f", fg="black", command=self.start_autotune,
                  **btn_style).grid(row=1, column=0, padx=10, pady=10)
        tk.Button(button_frame, text="Apply Saved PID", bg="#2980b9", fg="white", command=self.apply_pid,
                  **btn_style).grid(row=1, column=1, padx=10, pady=10)

        tk.Button(root, text="Exit Fullscreen / Quit", bg="#7f8c8d", fg="white", font=("Helvetica", 12),
                  command=self.close).pack(pady=10)

        # --- Load image ---
        try:
            image = Image.open(r"C:\Users\vasla\Downloads\test.png")  # Replace with your image path
            image = image.resize((300, 200))
            self.img = ImageTk.PhotoImage(image)
            self.img_label = tk.Label(root, image=self.img, bg="#2c3e50")
            self.img_label.pack(pady=20)
        except Exception as e:
            print(f"🖼️ Image load failed: {e}")

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
                    if len(parts) >= 6:
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
                    if len(parts) >= 6:
                        self.angle_label.config(text=f"Angle: {parts[1]}")
                        self.zfield_label.config(text=f"Z Field: {parts[3]}")
                        self.velocity_label.config(text=f"Velocity: {parts[5]}")
                except:
                    pass
            elif line.startswith("Class:"):
                classification = line.split(":")[1].strip()
                self.class_label.config(text=f"Classification: {classification}")
                if classification == "SOFT":
                    self.softness_bar["value"] = 100
                    self.score_value_label.config(text="Soft")
                elif classification == "HARD":
                    self.softness_bar["value"] = 20
                    self.score_value_label.config(text="Hard")
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
