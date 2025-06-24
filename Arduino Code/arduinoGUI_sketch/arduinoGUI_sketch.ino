#include <WiFiS3.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <MPU6050.h>

// WiFi and connectivity settings
const char* ssid = "ExploRover";
const char* password = "explorover";
WiFiServer server(8888);

// definitions
#define ARDUINO_LED 13
#define DC_IN1 8
#define DC_IN2 9
#define DC_EN  0 // enable EnA and EnB
#define DC_IN3 2
#define DC_IN4 1
#define STEPPER_STEP 10
#define STEPPER_DIR 11
#define STEPPER_EN 12
#define BMP388_ADDRESS 0x77

// stepper definitions for arms
#define STEPPER1_STEP 4
#define STEPPER1_DIR 3
#define STEPPER2_STEP 6
#define STEPPER2_DIR 5
#define STEPPER_EN_ARMS 7

// Stepper Motor states
#define STEPPER_MOTOR_IDLE 0
#define STEPPER_MOTOR_RUNNING 1

// connection to Arduino 2
// HardwareSerial& stepperSerial = Serial1; // Or use SoftwareSerial if Serial1 isn't available

// sensors for data acquisition
Adafruit_BMP3XX bmp; // temp pressure sensor
MPU6050 mpu(0x68); // accelerometer

struct SensorSample {
  unsigned long timestamp;
  float temperature, pressure;
  float accelX, accelY, accelZ;
};

const int MAX_SAMPLES = 100;
SensorSample logBuffer[MAX_SAMPLES];
int currentIndex = 0;

bool dataLoggingActive = false;
unsigned long lastSampleTime = 0;
const unsigned long sampleInterval = 1000; // sample rate - set here

// variables for DC motor control
enum MotorState {
  MOTOR_IDLE,
  MOTOR_FORWARD,
  MOTOR_BACKWARD,
  MOTOR_PAUSE,
  MOTOR_OPEN_LOOP
};

// DC motor variables
MotorState motorState = MOTOR_IDLE;
unsigned long motorStartTime = 0;
bool wasLastForward = true;
bool isMotorRunning = false;

// motor durations
const unsigned long driveDuration = 2000; 
const unsigned long pauseDuration = 1000;

// stepper function declaration
void stepperStep(bool direction, int steps, int delayMicros = 800);

// Global state variables for arms steppers
int stepperMotorState = STEPPER_MOTOR_IDLE;
bool isStepperMotorRunning = false;

// Movement control variables for arms steppers
bool dirStepper1 = false;
int stepsLeft = 0;
unsigned int stepDelayMicros = 800;
unsigned long lastStepTime = 0;
bool stepPinHigh = false;

// ---------------------------------------------------------------------------

void setup() {
  pins_setup();
  connection_setup();
  delay(1000);
  sensors_setup();
}

void pins_setup() {
  // Setup LED pins
  pinMode(ARDUINO_LED, OUTPUT);
  digitalWrite(ARDUINO_LED, LOW);

  // Setup DC motor pins
  pinMode(DC_IN1, OUTPUT);
  pinMode(DC_IN2, OUTPUT);
  pinMode(DC_EN, OUTPUT); 
  pinMode(DC_IN3, OUTPUT);
  pinMode(DC_IN4, OUTPUT);
  
  // Stepper activation for turning
  pinMode(STEPPER_DIR, OUTPUT);
  pinMode(STEPPER_STEP, OUTPUT);
  pinMode(STEPPER_EN, OUTPUT);
  digitalWrite(STEPPER_DIR, LOW);
  digitalWrite(STEPPER_STEP, LOW);
  digitalWrite(STEPPER_EN, HIGH);

  // Stepper activation for arms
  pinMode(STEPPER1_DIR, OUTPUT);
  pinMode(STEPPER1_STEP, OUTPUT);
  digitalWrite(STEPPER1_DIR, LOW);
  digitalWrite(STEPPER1_STEP, LOW);

  pinMode(STEPPER2_DIR, OUTPUT);
  pinMode(STEPPER2_STEP, OUTPUT);
  digitalWrite(STEPPER2_DIR, LOW);
  digitalWrite(STEPPER2_STEP, LOW);

  pinMode(STEPPER_EN_ARMS, OUTPUT);
  digitalWrite(STEPPER_EN_ARMS, HIGH); // disable motors initially
  
  // Disable DC motors initially
  digitalWrite(DC_EN, LOW); 
}

void connection_setup() { 
  // Start Serial
  Serial.begin(9600); // Main serial
  // stepperSerial.begin(9600); // Link to Arduino 2

  delay(2000); // Allow Serial time to initialize
  blink_led(1, ARDUINO_LED);

  int status = WiFi.beginAP(ssid, password);
  if (status != WL_AP_LISTENING) {
    Serial.println("Failed to create AP");
    while (true);
  }

  server.begin();
  blink_led(2, ARDUINO_LED);

  Serial.print("WiFi ready. Access Point IP: ");
  Serial.println(WiFi.localIP());
}

void check_i2c_addresses() {
  Wire.begin();
  Serial.begin(9600);
  delay(1000);
  Serial.println("🔍 Scanning I2C bus...");

  byte count = 0;
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found I2C device at 0x");
      Serial.println(address, HEX);
      count++;
      delay(1);
    }
  }

  if (count == 0) {
    Serial.println("No I2C devices found. Check wiring.");
  } else {
    Serial.println("Done scanning.");
  }
}

void sensors_setup() {
  Wire.begin();
  delay(500);

  Serial.println("Trying BMP388 at 0x77...");
  if (!bmp.begin_I2C(BMP388_ADDRESS)) { 
    Serial.println("❌ BMP388 not found");
  } else {
    Serial.println("✅ BMP388 initialized.");
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  }

  Serial.println("Initializing MPU6050 at 0x68...");
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("❌ MPU6050 connection failed");
  } else {
    Serial.println("✅ MPU6050 connected.");
  }

  blink_led(3, ARDUINO_LED);
}


// ---------------------------------------------------------------------------

void loop() {
  get_data(); // runs only if dataLoggingActive is true

  checkSerial();
  checkWiFi();

  controlMotor();
  performStep();
}

// ---------------------------------------------------------------------------

void performStep() {
  if (!isStepperMotorRunning || stepperMotorState == STEPPER_MOTOR_IDLE) {
    return; // nothing to do
  }

  unsigned long now = micros();

  if (stepsLeft <= 0) {
    // Movement done
    isStepperMotorRunning = false;
    stepperMotorState = STEPPER_MOTOR_IDLE;
    digitalWrite(STEPPER_EN_ARMS, HIGH); // disable motors
    digitalWrite(STEPPER1_STEP, LOW);
    digitalWrite(STEPPER2_STEP, LOW);
    return;
  }

  if (!stepPinHigh && (now - lastStepTime >= stepDelayMicros)) {
    // Step pins go HIGH
    digitalWrite(STEPPER1_STEP, HIGH);
    digitalWrite(STEPPER2_STEP, HIGH);
    lastStepTime = now;
    stepPinHigh = true;
  } else if (stepPinHigh && (now - lastStepTime >= stepDelayMicros)) {
    // Step pins go LOW
    digitalWrite(STEPPER1_STEP, LOW);
    digitalWrite(STEPPER2_STEP, LOW);
    lastStepTime = now;
    stepsLeft--;
    stepPinHigh = false;
  }
}

void startMovement(bool direction, int steps, int delayMicros) {
  stepperMotorState = STEPPER_MOTOR_RUNNING;
  isStepperMotorRunning = true;
  dirStepper1 = direction;
  stepsLeft = steps;
  stepDelayMicros = delayMicros;
  lastStepTime = 0;
  stepPinHigh = false;

  digitalWrite(STEPPER_EN_ARMS, LOW);  // Enable motor drivers
  digitalWrite(STEPPER1_DIR, dirStepper1 ? HIGH : LOW);
  digitalWrite(STEPPER2_DIR, dirStepper1 ? LOW : HIGH);
}

// void sendToStepper(const String& command) {
  // stepperSerial.println(command); // Send command to Arduino 2
// }

void get_data() {
  if (dataLoggingActive && millis() - lastSampleTime >= sampleInterval) {
    lastSampleTime = millis();

    SensorSample s;
    s.timestamp = millis();
    if (bmp.performReading()) {
      s.temperature = bmp.temperature;
      s.pressure = bmp.pressure / 100.0F;
    } else {
      Serial.println("⚠️ BMP388 read failed");
      s.temperature = s.pressure = 0;
    }

    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);
    s.accelX = ax / 16384.0;
    s.accelY = ay / 16384.0;
    s.accelZ = az / 16384.0;

    logBuffer[currentIndex] = s;
    currentIndex = (currentIndex + 1) % MAX_SAMPLES;
  } 
}

void blink_led(int counter, int ledPin) {
  int led_delay = 500;
  // exception
  if(counter < 1 || counter > 20) return;

  for(int i = 1; i <= counter; i++) {
    digitalWrite(ledPin, HIGH);
    delay(led_delay);
    digitalWrite(ledPin, LOW);
    delay(led_delay);
  }
}

void checkSerial() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toLowerCase();

    Serial.print("Serial received: ");
    Serial.println(command);

    String response = handleCommand(command);
    Serial.println(response); // Echo response for debugging
  }
}

void checkWiFi() {
  WiFiClient client = server.available();
  if (!client) return;

  unsigned long start = millis();
  while (!client.available() && millis() - start < 1000);  // Wait max 1 sec

  if (!client.available()) {
    client.stop();
    return;
  }

  String command = client.readStringUntil('\n');
  command.trim();
  command.toLowerCase();

  Serial.print("WiFi received: ");
  Serial.println(command);

  String response = handleCommand(command);

  client.println(response);
  client.flush();            // Ensure data is sent before closing
  delay(20);                 // Give time to complete transmission
  client.stop();             // Always close after sending response
}

String handleCommand(String cmd) {
  if (cmd == "led on") {
    digitalWrite(ARDUINO_LED, HIGH);
    return "LED was turned on.";
  } else if (cmd == "led off") {
    digitalWrite(ARDUINO_LED, LOW);
    return "LED was turned off.";
  } else if (cmd == "ping") {
    return "PONG";
  } else if (cmd == "turn left") {
    digitalWrite(STEPPER_EN, LOW);
    stepperStep(false, 200); // Move 200 steps in "left" direction - turn left command
    digitalWrite(STEPPER_EN, HIGH);
    return "Stepper turned left.";
  } else if (cmd == "turn right") {
    digitalWrite(STEPPER_EN, LOW);
    stepperStep(true, 200);  // Move 200 steps in "right" direction - turn right command
    digitalWrite(STEPPER_EN, HIGH);
    return "Stepper turned right.";
  } else if (cmd.startsWith("rotate ")) {
    digitalWrite(STEPPER_EN, LOW);
    int steps = cmd.substring(7).toInt();  // Extract number after "rotate "
    if (steps == 0) {
      return "Invalid step count.";
    }
    bool direction = (steps > 0);
    stepperStep(direction, abs(steps));  // Use absolute value for step count
    digitalWrite(STEPPER_EN, HIGH);
    return "Rotated " + String(steps) + " steps.";
  } else if (cmd == "start") {
    delay(200); // allow WiFi command to finish
    // kick off a forward run
    motorState      = MOTOR_OPEN_LOOP;
    motorStartTime  = millis(); 
    return "Open Loop Command Started";
  }
  else if (cmd == "motor backward") {
    // kick off a backward run
    motorState      = MOTOR_BACKWARD;
    motorStartTime  = millis();     // ← reset your timer here
    wasLastForward  = false;
    isMotorRunning  = true;
    return "Motors starting backward.";
  }
  else if (cmd == "run" || cmd == "motor forward") {
    delay(200); // allow WiFi command to finish
    // kick off a forward run
    motorState      = MOTOR_FORWARD;
    motorStartTime  = millis(); 
    wasLastForward  = true;
    isMotorRunning  = true;
    return "Motors starting forward.";
  }
  else if (cmd == "stop") {
    motorState      = MOTOR_IDLE;
    isMotorRunning  = false;
    stopMotors();
    return "Motors stopped.";
  }
  else if (cmd == "emergency stop") {
    // stop DC motors
    motorState      = MOTOR_IDLE;
    isMotorRunning  = false;
    stopMotors();

    //stop arms steppers 
    isStepperMotorRunning = false;
    stepperMotorState = MOTOR_IDLE;
    digitalWrite(STEPPER_EN_ARMS, HIGH);  // disable motors immediately
    digitalWrite(STEPPER1_STEP, LOW);
    digitalWrite(STEPPER2_STEP, LOW);
    Serial.println("Arm Stepper Motors stopped.");

    // stop turning stepper
    digitalWrite(STEPPER_EN, HIGH);
    digitalWrite(STEPPER_STEP, LOW);

    return "Emergency stop activated!";
  }
  // ---------------

  else if (cmd == "stop step") {
    isStepperMotorRunning = false;
    stepperMotorState = MOTOR_IDLE;
    digitalWrite(STEPPER_EN_ARMS, HIGH);  // disable motors immediately
    digitalWrite(STEPPER1_STEP, LOW);
    digitalWrite(STEPPER2_STEP, LOW);
    Serial.println("Arm Stepper Motors stopped.");
    return "Arm Stepper Motors stopped.";
  }
  else if (cmd == "2turn left") {
    startMovement(false, 200, 800);
    return "Turned left.";
  } 
  else if (cmd == "2turn right") {
    startMovement(true, 200, 800);
    return "Turned right.";
  } 
  else if (cmd.startsWith("2rotate ")) {
    int s = cmd.substring(7).toInt();
    if (s == 0) return "Invalid step count.";
    bool d = (s > 0);
    startMovement(d, abs(s), 800);
    return "Rotated " + String(s) + " steps.";
  } 
  else if (cmd == "raise") {
    // Set fixed directions for raise
    digitalWrite(STEPPER1_DIR, HIGH);
    digitalWrite(STEPPER2_DIR, LOW);
    startMovement(true, 200, 800);
    return "Explorover raised.";
  }
  else if (cmd == "lower") {
    // Set fixed directions for lowering
    digitalWrite(STEPPER1_DIR, LOW);
    digitalWrite(STEPPER2_DIR, HIGH);
    startMovement(true, 200, 800);
    return "Explorover raised.";
  }
  else if (cmd == "hold") {
    // enables arms stepper motors for holding the 
    stepperMotorState = STEPPER_MOTOR_RUNNING;
    digitalWrite(STEPPER_EN_ARMS, LOW); 
    return "ExploRover holding height.";
  }


  // ---------------

  else if (cmd == "i2c") {
    // implement stepper stop
    check_i2c_addresses();
    return "Checked i2c addresses";
  }
  else if (cmd == "wifi") {
    Serial.print("WiFi ready. Access Point IP: ");
    Serial.println(WiFi.localIP());
    return "Checked wifi.";
  }
  else if (cmd == "collect data") {
    currentIndex = 0;
    dataLoggingActive = true;
    return "Data collection started.";
  }
  else if (cmd == "stop data") {
    dataLoggingActive = false;
    return "Data collection stopped.";
  }
  else if (cmd == "get data") {
    String output = "time,temp,press,ax,ay,az\n";

    for (int i = 0; i < MAX_SAMPLES; i++) {
      SensorSample& s = logBuffer[i];
      if (s.timestamp == 0) continue; // skip empty entries

      output += String(s.timestamp) + ",";
      output += String(s.temperature, 1) + ",";
      output += String(s.pressure, 1) + ",";
      output += String(s.accelX, 2) + ",";
      output += String(s.accelY, 2) + ",";
      output += String(s.accelZ, 2) + "\n";

      // Optional: break early if output is getting too large (safety for Wi-Fi)
    }

    // Clear the buffer after sending
    for (int i = 0; i < MAX_SAMPLES; i++) {
      logBuffer[i].timestamp = 0;
    }
    currentIndex = 0;

    return output;
  }

  return "UNKNOWN COMMAND";
}

void stepperStep(bool direction, int steps, int delayMicros) {
  digitalWrite(STEPPER_DIR, direction ? HIGH : LOW);
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEPPER_STEP, HIGH);
    delayMicroseconds(delayMicros);
    digitalWrite(STEPPER_STEP, LOW);
    delayMicroseconds(delayMicros);
  }
}

void controlMotor() {
  unsigned long now = millis();

  switch (motorState) {
    case MOTOR_FORWARD:
      isMotorRunning = true;
      moveForward();
      if (now - motorStartTime >= driveDuration) {
        stopMotors();
        motorStartTime = now;
        motorState = MOTOR_PAUSE;
      }
      break;

    case MOTOR_BACKWARD:
      isMotorRunning = true;
      moveBackward();
      if (now - motorStartTime >= driveDuration) {
        stopMotors();
        motorStartTime = now;
        motorState = MOTOR_PAUSE;
      }
      break;

    case MOTOR_PAUSE:
      if (now - motorStartTime >= pauseDuration) {
        motorState = (motorState == MOTOR_PAUSE && wasLastForward)
                     ? MOTOR_BACKWARD
                     : MOTOR_FORWARD;
        motorStartTime = now;
      }
      break;

    case MOTOR_IDLE:
      isMotorRunning = false;
      stopMotors();
      break;
    
    case MOTOR_OPEN_LOOP:
      int openLoopDuration = 15000;
      isMotorRunning = true;
      moveForward();
      if (now - motorStartTime >= openLoopDuration ) {
        stopMotors();
        motorStartTime = now;
        motorState = MOTOR_IDLE;
      }
      break;
  }
}

void moveForward() {
  digitalWrite(DC_EN, HIGH);
  digitalWrite(DC_IN1, LOW);
  digitalWrite(DC_IN2, HIGH);
  digitalWrite(DC_IN3, HIGH);
  digitalWrite(DC_IN4, LOW);
  wasLastForward = true;
}

void moveBackward() {
  digitalWrite(DC_EN, HIGH);
  digitalWrite(DC_IN1, HIGH);
  digitalWrite(DC_IN2, LOW);
  digitalWrite(DC_IN3, LOW);
  digitalWrite(DC_IN4, HIGH);
  wasLastForward = false;
}

void stopMotors() {
  digitalWrite(DC_EN, LOW);
  digitalWrite(DC_IN1, LOW);
  digitalWrite(DC_IN2, LOW);
  digitalWrite(DC_IN3, LOW);
  digitalWrite(DC_IN4, LOW);
}
