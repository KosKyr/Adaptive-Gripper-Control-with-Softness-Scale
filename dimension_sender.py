import serial
import time

class DimensionSender:
    def __init__(self, port='COM3', baudrate=115200):
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            print(f"Connected to microcontroller on {port}")
            time.sleep(2)
        except Exception as e:
            print(f"Serial connection failed: {e}")
            self.ser = None

    def send(self, length_cm, width_cm):
        if self.ser:
            msg = f"{length_cm:.1f},{width_cm:.1f}\n"
            self.ser.write(msg.encode('utf-8'))
            print(f"Sent: {msg.strip()}")

    def close(self):
        if self.ser:
            self.ser.close()
