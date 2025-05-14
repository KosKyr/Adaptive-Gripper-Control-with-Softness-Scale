void loop() {
  motor.loopFOC();

  float current_angle = motor.sensor->getAngle();
  unsigned long current_time = millis();
  float dt = (current_time - last_time) / 1000.0f;
  last_time = current_time;
  float velocity = (dt > 0) ? abs(current_angle - last_angle) / dt : 0.0;
  last_angle = current_angle;

#if ENABLE_MAGNETIC_SENSOR
  // Save initial angle once
  if (!angle_saved) {
    initial_open_angle = current_angle;
    angle_saved = true;
  }

  static bool returning_to_open = false;

  if (digitalRead(BUTTON1) == LOW && !object_gripped && !returning_to_open) {
    target_voltage = -3;  // close
  } 
  else if (digitalRead(BUTTON2) == LOW && !returning_to_open) {
    Serial.println("🔁 Returning to initial open position...");
    returning_to_open = true;
    object_gripped = false;
    stable_count = 0;

    // Switch to angle control temporarily
    motor.controller = MotionControlType::angle;
    motor.move(initial_open_angle);
  }

  // Handle returning to position
  if (returning_to_open) {
    motor.controller = MotionControlType::angle;
    motor.move(initial_open_angle);
    if (abs(current_angle - initial_open_angle) < 0.05) {  // within 0.05 rad (~3 deg)
      Serial.println("✅ Reached initial open position.");
      returning_to_open = false;

      // Restore to torque mode
      motor.controller = MotionControlType::torque;
      target_voltage = 0;
    }
    return;  // skip rest of loop during return
  }

  if (!object_gripped) {
    target_voltage = 0;  // hold (unless gripping)
  }
#endif

  // --- Object grip detection ---
  if (abs(target_voltage) > 0.2 && velocity < velocity_threshold) {
    stable_count++;
    if (stable_count > stable_threshold && !object_gripped) {
      object_gripped = true;
      target_voltage = -0.3;  // firm hold
      Serial.println("🟢 Object gripped. Holding firmly.");

      motor.PID_velocity.P = 0.15;
      motor.PID_velocity.I = 3.0;
      motor.PID_velocity.D = 0.01;
    }
  } else {
    stable_count = 0;
    if (object_gripped && abs(target_voltage) > 0.2) {
      object_gripped = false;
      Serial.println("🔄 Grip released. Resuming movement.");

      motor.PID_velocity.P = 0.3;
      motor.PID_velocity.I = 5.0;
      motor.PID_velocity.D = 0.001;
    }
  }

  if (!object_gripped) {
    motor.move(target_voltage);
  }

  command.run();
}
