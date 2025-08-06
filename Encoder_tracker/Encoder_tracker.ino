#include "Arduino.h"
#include "Encoder.h"
#include <Stepper.h>

// === Pins ===
#define ENCODER_CLK   2
#define ENCODER_DT    3

// === Stepper Motor Setup ===
#define STEPS_PER_REV 2048
Stepper stepper(STEPS_PER_REV, 8, 10, 9, 11);  // IN1, IN3, IN2, IN4

// === Encoder Setup ===
Encoder encoder(ENCODER_DT, ENCODER_CLK);

// === Amplification Factor ===
#define ENCODER_STEP_MULTIPLIER 10  // Encoder ticks * this = stepper steps

// === PID Constants ===
float Kp = 2.0;
float Ki = 0.0;
float Kd = 0.1;

// === Control State ===
long targetPosition = 0;
long currentStepperPosition = 0;

float error = 0;
float lastError = 0;
float integral = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  encoder.write(0);  // Reset encoder
  stepper.setSpeed(15);  // Required for Stepper class (used internally)

  // Test step
  Serial.println("Stepper test...");
  stepper.step(STEPS_PER_REV);
  delay(1000);
  stepper.step(-STEPS_PER_REV);
  Serial.println("Ready: PID-controlled stepper following encoder.");
}

void loop() {
  // === Read and Scale Encoder Position ===
  long encoderTicks = encoder.read() / 4;  // /4 if your encoder gives 4 pulses per detent
  targetPosition = encoderTicks * ENCODER_STEP_MULTIPLIER;

  // === PID Calculation ===
  error = targetPosition - currentStepperPosition;
  integral += error;
  float derivative = error - lastError;
  float pidOutput = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;

  // === Constrain Output ===
  pidOutput = constrain(pidOutput, -20, 20);  // Max speed limit

  // === Step motor at speed based on PID output ===
  static unsigned long lastStepTime = 0;
  unsigned long now = millis();

  if (abs(pidOutput) > 0.5 && now - lastStepTime > 10) {
    int direction = (pidOutput > 0) ? 1 : -1;
    stepper.step(direction);
    currentStepperPosition += direction;
    lastStepTime = now;
  }

  // === Debugging ===
  static unsigned long lastDebug = 0;
  if (now - lastDebug > 500) {
    Serial.print("Encoder Ticks: ");
    Serial.print(encoderTicks);
    Serial.print("\tTarget: ");
    Serial.print(targetPosition);
    Serial.print("\tCurrent: ");
    Serial.print(currentStepperPosition);
    Serial.print("\tPID Output: ");
    Serial.println(pidOutput);
    lastDebug = now;
  }
}
