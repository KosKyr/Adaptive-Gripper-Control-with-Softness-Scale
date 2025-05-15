import sys
import serial
import json
import threading
import time
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QKeySequence

SERIAL_PORT = "/dev/ttyUSB0"  # Change to your port
BAUDRATE = 115200
pid_file = "pid_config.json"

class GripperGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.serial = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
        self.init_ui()
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_data)
        self.timer.start(200)

    def init_ui(self):
        self.setWindowTitle("Gripper Control + PID Autotune")
        self.resize(400, 300)

        # Labels
        self.label_angle = QLabel("Angle: ---")
        self.label_field = QLabel("Z Field: ---")
        self.label_result = QLabel("Classification: ---")

        # Buttons
        self.btn_grip = QPushButton("Grip & Classify")
        self.btn_release = QPushButton("Release")
        self.btn_autotune = QPushButton("PID Autotune")
        self.btn_apply_pid = QPushButton("Apply Saved PID")

        self.btn_grip.clicked.connect(self.send_grip)
        self.btn_release.clicked.connect(self.send_release)
        self.btn_autotune.clicked.connect(self.start_autotune)
        self.btn_apply_pid.clicked.connect(self.apply_saved_pid)

        layout = QVBoxLayout()
        layout.addWidget(self.label_angle)
        layout.addWidget(self.label_field)
        layout.addWidget(self.label_result)
        layout.addWidget(self.btn_grip)
        layout.addWidget(self.btn_release)
        layout.addWidget(self.btn_autotune)
        layout.addWidget(self.btn_apply_pid)

        self.setLayout(layout)

    def send_grip(self):
        self.serial.write(b"G\n")  # Define 'G' in firmware to trigger classification

    def send_release(self):
        self.serial.write(b"O\n")  # Define 'O' in firmware to trigger open

    def update_data(self):
        if self.serial.in_waiting:
            line = self.serial.readline().decode().strip()
            if line.startswith("A:"):
                parts = line.split()
                angle = parts[1]
                z = parts[3]
                self.label_angle.setText(f"Angle: {angle}")
                self.label_field.setText(f"Z Field: {z}")
            elif line.startswith("Class:"):
                self.label_result.setText(f"Classification: {line.split(':')[1].strip()}")

    def start_autotune(self):
        thread = threading.Thread(target=self.run_autotune)
        thread.start()

    def run_autotune(self):
        print("⚙️ PID Autotune running for 5s (press Ctrl+C to interrupt)...")
        t0 = time.time()
        data = []
        try:
            self.serial.write(b"T-2.0\n")  # Start torque
            while time.time() - t0 < 5:
                if self.serial.in_waiting:
                    line = self.serial.readline().decode().strip()
                    if line.startswith("A:"):
                        parts = line.split()
                        angle = float(parts[1])
                        velocity = float(parts[5])
                        data.append((angle, velocity))
                time.sleep(0.05)
        except KeyboardInterrupt:
            print("⛔ Interrupted by user.")
        finally:
            self.serial.write(b"T0\n")  # Stop torque

        # Naive PID estimator
        angle_changes = [abs(data[i+1][0] - data[i][0]) for i in range(len(data)-1)]
        velocities = [v for _, v in data]
        p_est = max(angle_changes) * 0.5
        i_est = sum(velocities) * 0.1
        d_est = (max(velocities) - min(velocities)) * 0.05

        pid = {"P": round(p_est, 3), "I": round(i_est, 3), "D": round(d_est, 3)}
        with open(pid_file, "w") as f:
            json.dump(pid, f, indent=2)
        print(f"✅ PID values saved: {pid}")

    def apply_saved_pid(self):
        try:
            with open(pid_file, "r") as f:
                pid = json.load(f)
            self.serial.write(f"PID {pid['P']} {pid['I']} {pid['D']}\n".encode())
            print(f"📦 Sent PID: {pid}")
        except:
            print("❌ Failed to read PID config file.")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = GripperGUI()
    win.show()
    sys.exit(app.exec_())
