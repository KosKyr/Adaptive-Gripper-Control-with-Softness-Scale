import tkinter as tk
from tkinter import ttk, messagebox
import serial
import json
import threading
import time
import cv2
import numpy as np
from PIL import Image, ImageTk
from pipeline import Pipeline

SERIAL_PORT = "COM5"
BAUDRATE = 115200
PID_FILE = "pid_config.json"

class GripperGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Adaptive Gripper Control with Softness Scale")
        self.root.attributes('-fullscreen', True)
        self.root.configure(bg="#2c3e50")

        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
        except serial.SerialException as e:
            self.ser = None
            messagebox.showwarning("Serial Port Warning", f"Serial not connected: {e}")

        self.setup_labels()
        self.setup_buttons()
        self.running = True
        self.update_thread = threading.Thread(target=self.update_labels)
        self.update_thread.start()

        self.root.bind("<Escape>", lambda event: self.close())

    def setup_labels(self):
        title = tk.Label(self.root, text="🧫 Gripper Interface + Softness Visualizer", font=("Helvetica", 26, "bold"),
                         fg="#ecf0f1", bg="#2c3e50")
        title.pack(pady=20)

        self.angle_label = tk.Label(self.root, text="Angle: ---", font=("Helvetica", 18), fg="#1abc9c", bg="#2c3e50")
        self.angle_label.pack()
        self.zfield_label = tk.Label(self.root, text="Z Field: ---", font=("Helvetica", 18), fg="#3498db", bg="#2c3e50")
        self.zfield_label.pack()
        self.velocity_label = tk.Label(self.root, text="Velocity: ---", font=("Helvetica", 18), fg="#f39c12", bg="#2c3e50")
        self.velocity_label.pack()
        self.class_label = tk.Label(self.root, text="Classification: ---", font=("Helvetica", 18), fg="#e74c3c", bg="#2c3e50")
        self.class_label.pack(pady=10)

        self.softness_label = tk.Label(self.root, text="Softness Indicator", font=("Helvetica", 18), fg="#ffffff", bg="#2c3e50")
        self.softness_label.pack()
        self.score_value_label = tk.Label(self.root, text="---", font=("Helvetica", 16, "bold"), fg="#9b59b6", bg="#2c3e50")
        self.score_value_label.pack()

        self.softness_bar = ttk.Progressbar(self.root, orient="horizontal", length=400, mode="determinate")
        self.softness_bar.pack(pady=10)
        self.softness_bar["value"] = 0

        style = ttk.Style()
        style.theme_use("default")
        style.configure("TProgressbar", foreground="#9b59b6", background="#9b59b6", thickness=30)

        self.timer_label = tk.Label(self.root, text="", font=("Helvetica", 14), fg="#ecf0f1", bg="#2c3e50")
        self.timer_label.pack()
        self.vision_msg_label = tk.Label(self.root, text="", font=("Helvetica", 16), fg="#ecf0f1", bg="#2c3e50")
        self.vision_msg_label.pack(pady=10)

    def setup_buttons(self):
        button_frame = tk.Frame(self.root, bg="#2c3e50")
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

        self.stop_button = tk.Button(self.root, text="Stop Autotune Early", bg="#e67e22", fg="white",
                                     font=("Helvetica", 12), command=self.stop_autotune, state="disabled")
        self.stop_button.pack(pady=5)

        tk.Button(self.root, text="Open Vision Mode (Press Q to Quit)", bg="#16a085", fg="white",
                  font=("Helvetica", 14), command=self.start_yolo_vision_mode).pack(pady=5)

        tk.Button(self.root, text="Exit Fullscreen / Quit", bg="#7f8c8d", fg="white", font=("Helvetica", 12),
                  command=self.close).pack(pady=10)

    def update_labels(self):
        while self.running:
            line = read_serial_data(self.ser) if self.ser else ""
            if not line:
                time.sleep(0.05)
                continue

            print("Serial:", line)  # Debug print, can be commented out

            if line.startswith("A:"):
                try:
                    # Expected format: A: <angle> Z: <z_field> V: <velocity>
                    tokens = line.replace(":", "").split()
                    angle = tokens[1]
                    zfield = tokens[3]
                    velocity = tokens[5]

                    self.angle_label.config(text=f"Angle: {angle}")
                    self.zfield_label.config(text=f"Z Field: {zfield}")
                    self.velocity_label.config(text=f"Velocity: {velocity}")
                except Exception as e:
                    print(f"[ERROR] Failed to parse sensor data: {e} - Line: {line}")

            elif "class" in line.lower():
                try:
                    classification = line.split(":")[1].strip().upper()
                    self.class_label.config(text=f"Classification: {classification}")

                    if classification == "SOFT":
                        self.softness_bar["value"] = 100
                        self.score_value_label.config(text="🟢 Soft Object")
                    elif classification == "HARD":
                        self.softness_bar["value"] = 20
                        self.score_value_label.config(text="🔴 Hard Object")
                    else:
                        self.softness_bar["value"] = 50
                        self.score_value_label.config(text=f"⚪ Unknown ({classification})")
                except Exception as e:
                    print(f"[ERROR] Failed to parse classification: {e} - Line: {line}")

            time.sleep(0.1)


    def start_yolo_vision_mode(self):
        threading.Thread(target=self.run_yolo_vision_mode, daemon=True).start()

    def run_yolo_vision_mode(self):
        def gui_callback(msg):
            self.vision_msg_label.config(text=msg)

        pipe = Pipeline(
            label_color=(0, 255, 0),
            box_color=(0, 0, 255),
            mask_color=(255, 0, 0),
            alpha=0.5,
            pad=0,
            confidence_threeshold=0.25,
            yolo_weights_path="yolo-weights/yolov8n.pt",
            dataset_path="dataset/",
            use_data_set=False,
            filter_classes=["cell phone"],
            callback=gui_callback
        )
        pipe.main()

    def close(self):
        self.running = False
        self.autotune_running = False
        time.sleep(0.2)
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.root.destroy()

def read_serial_data(ser):
    try:
        line = ser.readline().decode().strip()
        return line
    except:
        return ""

if __name__ == "__main__":
    root = tk.Tk()
    app = GripperGUI(root)
    root.mainloop()
