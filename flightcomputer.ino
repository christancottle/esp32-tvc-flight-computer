#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <ESP32Servo.h>
#include <LittleFS.h>

// --- HARDWARE CONFIGURATION ---
const int SERVO_PIN = 18;
const int SERVO_CENTER = 90;
const int SERVO_MAX_DEFLECTION = 30;

// --- SENSOR OBJECTS & LOG FILE ---
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;
Servo tvcServo;
File logFile;

// --- FLIGHT STATES ---
enum FlightState {
  PAD_IDLE,
  POWERED_FLIGHT,
  COAST_TO_APOGEE,
  RECOVERY_LANDED
};
FlightState currentState = PAD_IDLE;

// --- PID GAINS & VARIABLES ---
float kp = 2.50, ki = 0.02, kd = 0.02;
float error, lastError, integralError, derivativeError;
float servoCommand = 90.0;
unsigned long lastTime;

// --- TELEMETRY & BASELINES ---
float basePressure = 0.0;
float maxAltitude = 0.0;
unsigned long flightStartTime = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("\n==========================================");
  Serial.println("🚀 ESP32 AUTONOMOUS FLIGHT COMPUTER BOOT");
  Serial.println("==========================================");

  // 1. Initialize LittleFS Internal Storage
  if (!LittleFS.begin(true)) {
    Serial.println("❌ LittleFS Mount Failed!");
    while (1) delay(10);
  }
  Serial.println("✅ LittleFS Storage Mounted!");

  // Create/Overwrite Telemetry CSV File
  logFile = LittleFS.open("/flight_log.csv", FILE_WRITE);
  if (logFile) {
    logFile.println("Time_ms,State,Tilt_deg,Servo_deg,Alt_m,AccZ_ms2");
    logFile.flush();
    Serial.println("✅ Flash Memory File Created: /flight_log.csv");
  }

  // 2. Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("❌ MPU6050 Init Failed!");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.println("✅ MPU6050 Online!");

  // 3. Initialize BMP280
  if (!bmp.begin(0x76) && !bmp.begin(0x77)) {
    Serial.println("❌ BMP280 Init Failed!");
    while (1) delay(10);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X4,
                  Adafruit_BMP280::STANDBY_MS_63);
  Serial.println("✅ BMP280 Online!");

  // Zero Ground Baseline
  Serial.println("⏳ Sampling ground pressure baseline...");
  float totalP = 0;
  for (int i = 0; i < 10; i++) {
    totalP += bmp.readPressure() / 100.0F;
    delay(20);
  }
  basePressure = totalP / 10.0F;

  // 4. Initialize TVC Servo
  tvcServo.attach(SERVO_PIN);
  tvcServo.write(SERVO_CENTER);

  lastTime = millis();
  Serial.println("\nSTATUS: [PAD_IDLE] - Waiting for launch flick (>22 m/s² Z-accel)");
  Serial.println("--------------------------------------------------");
}

void loop() {
  // Time delta
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  if (dt <= 0) dt = 0.001;

  // Read Sensors
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // 1. Raw Angle calculation & Low-Pass Filter
  float rawAngle = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  static float smoothedAngle = 0.0;
  smoothedAngle = (0.80 * smoothedAngle) + (0.20 * rawAngle);
  float currentAngle = smoothedAngle;

  // 2. Read Altitude with a Glitch Filter (Hot-Reload)
  float rawAltitude = bmp.readAltitude(basePressure); 
  if (rawAltitude > 2000.0 || isnan(rawAltitude)) {
    Wire.begin();
    bmp.begin(0x76);
    delay(5);
    rawAltitude = bmp.readAltitude(basePressure);
  }

  // 3. Rolling Altimeter Smoothing (Prevents instant state-skipping spikes)
  static float currentAltitude = 0.0;
  currentAltitude = (0.85 * currentAltitude) + (0.15 * rawAltitude);

  // --- STATE MACHINE LOGIC ---
  switch (currentState) {

    case PAD_IDLE:
      tvcServo.write(SERVO_CENTER);
      if (a.acceleration.z > 14.0) {
        currentState = POWERED_FLIGHT;
        flightStartTime = millis();
        integralError = 0.0;
        lastError = 0.0;
        maxAltitude = currentAltitude; // Set starting baseline
        Serial.println("\n🚀🚀 LIFTOFF DETECTED! ENTERING [POWERED_FLIGHT] 🚀🚀");
      }
      break;

    case POWERED_FLIGHT:
      // Run Active PID Gimbal Control
      error = currentAngle - 0.0;
      integralError += error * dt;
      derivativeError = (error - lastError) / dt;
      lastError = error;

      servoCommand = SERVO_CENTER - ((kp * error) + (ki * integralError) + (kd * derivativeError));
      servoCommand = constrain(servoCommand, SERVO_CENTER - SERVO_MAX_DEFLECTION, SERVO_CENTER + SERVO_MAX_DEFLECTION);
      tvcServo.write(servoCommand);

      // Keep track of true peak altitude
      if (currentAltitude > maxAltitude) {
        maxAltitude = currentAltitude;
      }

      // APOGEE CHECK (With Debugging Print if it triggers prematurely)
      if ((millis() - flightStartTime) > 2000) {
        if (currentAltitude < (maxAltitude - 0.4)) { 
          Serial.print("\n🪂 APOGEE TRIGGERED! Peak was: "); Serial.print(maxAltitude);
          Serial.print("m, Current is: "); Serial.println(currentAltitude);
          currentState = COAST_TO_APOGEE;
        }
      }
      break;

    case COAST_TO_APOGEE:
      tvcServo.write(SERVO_CENTER);
      
      static float lastCheckAltitude = 0.0;
      static unsigned long stabilityStartTime = 0;
      static unsigned long lastCheckTime = 0;
      
      if (millis() - lastCheckTime > 250) {
        float altitudeChange = abs(currentAltitude - lastCheckAltitude);
        lastCheckAltitude = currentAltitude;
        lastCheckTime = millis();
        
        if (altitudeChange < 0.10) { // Under 10cm variation
          if (stabilityStartTime == 0) stabilityStartTime = millis();
        } else {
          stabilityStartTime = 0; // Reset timer if actively moving
        }
      }

      // LANDING CHECK
      if ((millis() - flightStartTime) > 4000 && stabilityStartTime != 0) { 
        if ((millis() - stabilityStartTime) > 1500) {
          currentState = RECOVERY_LANDED;
          if (logFile) logFile.close();
          Serial.println("\n🛬 TOUCHDOWN DETECTED! TELEMETRY SAVED TO FLASH.");
        }
      }
      break;

    case RECOVERY_LANDED:
      tvcServo.write(SERVO_CENTER);
      break;
  }

  // --- LOGGING & TELEMETRY STREAM ---
  if (currentState == POWERED_FLIGHT || currentState == COAST_TO_APOGEE) {
    if (logFile) {
      logFile.printf("%lu,%d,%.2f,%.2f,%.2f,%.2f\n", 
                      millis() - flightStartTime, currentState, currentAngle, servoCommand, currentAltitude, a.acceleration.z);
    }
  }

  // Live Serial Monitor Output (Watch Alt_m closely during the flick!)
  Serial.print("State:"); Serial.print(currentState); Serial.print(" ");
  Serial.print("Tilt:"); Serial.print(currentAngle, 1); Serial.print(" ");
  Serial.print("Servo:"); Serial.print(servoCommand, 1); Serial.print(" ");
  Serial.print("Alt_m:"); Serial.print(currentAltitude, 2); Serial.print(" ");
  Serial.print("MaxAlt:"); Serial.print(maxAltitude, 2); Serial.print(" ");
  Serial.print("AccZ:"); Serial.println(a.acceleration.z, 1);

  // Keyboard Commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "dump") dumpFlashData();
    if (cmd == "reset") {
      Serial.println("\n🔄 RESETTING COMPLIANT STATE MACHINE...");
      logFile = LittleFS.open("/flight_log.csv", FILE_WRITE);
      if (logFile) logFile.println("Time_ms,State,Tilt_deg,Servo_deg,Alt_m,AccZ_ms2");
      currentState = PAD_IDLE;
      maxAltitude = 0.0;
      currentAltitude = 0.0;
      integralError = 0.0;
      lastError = 0.0;
      flightStartTime = 0;
      tvcServo.write(SERVO_CENTER);
    }
  }

  delay(20); // 50Hz loop
}

// Function to read back saved CSV file from ESP32 Flash Memory
void dumpFlashData() {
  Serial.println("\n--- READING FLIGHT DATA FROM FLASH MEMORY ---");
  File file = LittleFS.open("/flight_log.csv", FILE_READ);
  if (!file) {
    Serial.println("❌ Failed to open log file.");
    return;
  }
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
  Serial.println("--- END OF FLIGHT DATA DUMP ---\n");
}
