# Autotune script for gripper torque PID based on magnetic sensor feedback

import serial
import time
import numpy as np

# Serial port settings
PORT = "COM3"  
BAUDRATE = 115200

# Tuning settings
initial_pid = [1.0, 0.0, 0.0]  # P, I, D starting values
torque_step = -1.0             # Applied torque step
samples_per_test = 100         # Samples per test round
sample_interval = 0.05         # Seconds between samples

def send_command(ser, cmd):
    ser.write((cmd + "\n").encode('utf-8'))
    time.sleep(0.1)

def read_sensor_data(ser):
    try:
        line = ser.readline().decode().strip()
        if line.startswith("Mag["):
            parts = line.strip("Mag[] ").split(",")
            if len(parts) >= 4:
                z = float(parts[2])
                b_mag = float(parts[3])
                return z, b_mag
    except:
        pass
    return None, None

def run_test(ser, pid):
    P, I, D = pid
    send_command(ser, f"L {P} {I} {D}")
    send_command(ser, f"T {torque_step}")
    time.sleep(0.5)

    b_values = []
    for _ in range(samples_per_test):
        z, b = read_sensor_data(ser)
        if b is not None:
            b_values.append(b)
        time.sleep(sample_interval)

    send_command(ser, "T 0")
    return np.array(b_values)

def score_response(b_values):
    if len(b_values) == 0:
        return 0
    rise = b_values[-1] - b_values[0]
    overshoot = max(b_values) - b_values[-1]
    smoothness = np.std(b_values[-10:])
    return rise - overshoot - smoothness

def autotune(ser, attempts=10):
    best_score = -float('inf')
    best_pid = initial_pid

    for i in range(attempts):
        P = round(np.random.uniform(0.5, 4.0), 2)
        I = round(np.random.uniform(0.0, 0.5), 2)
        D = round(np.random.uniform(0.0, 0.3), 2)
        print(f"Testing PID: P={P}, I={I}, D={D}")

        b_vals = run_test(ser, [P, I, D])
        score = score_response(b_vals)
        print(f"Score: {score:.3f}")

        if score > best_score:
            best_score = score
            best_pid = [P, I, D]
            print("--> New best PID")

    print("\nBest PID:", best_pid)
    send_command(ser, f"L {best_pid[0]} {best_pid[1]} {best_pid[2]}")
    return best_pid

def main():
    ser = serial.Serial(PORT, BAUDRATE, timeout=1.0)
    time.sleep(2)  # wait for board reset

    print("Starting autotune...")
    best = autotune(ser)

    print("Done. Best PID:", best)
    ser.close()

if __name__ == "__main__":
    main()
