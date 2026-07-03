
// to detect the data from sensor and handle the motor
////
// Measure theta (angle) and omega (angular velocity)
// Send measurements to MATLAB via Serial
// Receive control input from MATLAB
// Apply control to motor
// Dorchi Bhote 2026

#include <Wire.h>
#include <MPU6050.h>

// to define the constants

#define MOTOR_PWM_PIN   9
#define MOTOR_DIR_PIN   8
#define ENCODER_A_PIN   2
#define ENCODER_B_PIN   3

// objects

MPU6050 mpu;

// to define variables
// State variables (what MATLAB needs)
float theta = 0.0;          // Angle (radians)
float omega = 0.0;          // Angular velocity (rad/s)
float theta_prev = 0.0;

// Control variable (from MATLAB)
float control_input = 0.0;  // PWM value (0-255)

// Encoder
volatile long encoder_count = 0;
float encoder_theta = 0.0;

// Timing
unsigned long last_time = 0;
float dt = 0.01;  // 10ms sampling

// inturrupt for encoder

void encoderISR() {
    int a = digitalRead(ENCODER_A_PIN);
    int b = digitalRead(ENCODER_B_PIN);
    if (a == b) {
        encoder_count++;
    } else {
        encoder_count--;
    }
}

// to start setup

void setup() {
    Serial.begin(115200);
    Serial.println("=== Project 4: Digital Twin Hardware ===");
    
    // Initialize pins
    pinMode(MOTOR_PWM_PIN, OUTPUT);
    pinMode(MOTOR_DIR_PIN, OUTPUT);
    pinMode(ENCODER_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_B_PIN, INPUT_PULLUP);
    
    // Encoder interrupt
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderISR, CHANGE);
    
    // MPU6050
    Wire.begin();
    mpu.initialize();
    
    if (!mpu.testConnection()) {
        Serial.println("ERROR: MPU6050 not found!");
        while(1);
    }
    
    Serial.println("Ready for MATLAB communication!");
    last_time = micros();
}

// to start main loop

void loop() {
    // --- 1. READ SENSORS ---
    readIMU();
    readEncoder();
    
    // --- 2. SEND DATA TO MATLAB ---
    // Format: theta,omega,encoder_theta\n
    Serial.print(theta, 4);
    Serial.print(",");
    Serial.print(omega, 4);
    Serial.print(",");
    Serial.println(encoder_theta, 4);
    
    // --- 3. RECEIVE CONTROL FROM MATLAB ---
    if (Serial.available() > 0) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd.length() > 0) {
            control_input = cmd.toFloat();
            // Clamp to valid PWM range
            control_input = constrain(control_input, -255, 255);
        }
    }
    
    // --- 4. APPLY CONTROL ---
    applyMotorControl();
    
    // --- 5. TIMING (100Hz) ---
    delay(10);
}

// to start functions: readIMU()
// Read MPU6050 and compute theta, omega

void readIMU() {
    // Timing
    unsigned long current_time = micros();
    dt = (current_time - last_time) / 1000000.0;
    last_time = current_time;
    
    // Read MPU6050
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    // Convert gyro to rad/s (for ±250 deg/s range)
    float gyro_z = (gz / 131.0) * (PI / 180.0);
    
    // Integrate to get angle (simple integration)
    theta = theta + gyro_z * dt;
    
    // Compute angular velocity
    omega = (theta - theta_prev) / dt;
    theta_prev = theta;
}

// to start function: readEncoder()
// Read encoder for motor position

void readEncoder() {
    // Convert counts to radians
    float counts_per_rev = 48.0;  // 12 CPR * 4x decoding
    encoder_theta = (encoder_count / counts_per_rev) * 2.0 * PI;
}

// to start function: applyMotorControl()
// Apply PWM control to motor

void applyMotorControl() {
    if (control_input >= 0) {
        digitalWrite(MOTOR_DIR_PIN, HIGH);
        analogWrite(MOTOR_PWM_PIN, abs(control_input));
    } else {
        digitalWrite(MOTOR_DIR_PIN, LOW);
        analogWrite(MOTOR_PWM_PIN, abs(control_input));
    }
}