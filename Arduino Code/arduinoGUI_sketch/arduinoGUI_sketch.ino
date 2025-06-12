#include <WiFiS3.h>

// WiFi and connectivity settings
const char* ssid = "ExploRover";
const char* password = "explorover";
WiFiServer server(8888);

// definitions
#define ARDUINO_LED 13
#define DC_IN1 4
#define DC_IN2 5
#define DC_EN  3 // enable EnA and EnB
#define DC_IN3 6
#define DC_IN4 7
#define STEPPER_DIR 11
#define STEPPER_STEP 10

void stepperStep(bool direction, int steps, int delayMicros = 800);

// variables for motor control
enum MotorState {
  MOTOR_IDLE,
  MOTOR_FORWARD,
  MOTOR_BACKWARD,
  MOTOR_PAUSE
};

MotorState motorState = MOTOR_IDLE;
unsigned long motorStartTime = 0;
bool wasLastForward = true;
bool isMotorRunning = false;

// motor durations
const unsigned long driveDuration = 5000;
const unsigned long pauseDuration = 1000;

void setup() {
  // Setup LED and motor pins
  pinMode(ARDUINO_LED, OUTPUT);
  digitalWrite(ARDUINO_LED, LOW);

  pinMode(DC_IN1, OUTPUT);
  pinMode(DC_IN2, OUTPUT);
  pinMode(DC_EN, OUTPUT); 
  pinMode(DC_IN3, OUTPUT);
  pinMode(DC_IN4, OUTPUT);

  // Stepper activation
  pinMode(STEPPER_DIR, OUTPUT);
  pinMode(STEPPER_STEP, OUTPUT);
  digitalWrite(STEPPER_DIR, LOW);
  digitalWrite(STEPPER_STEP, LOW);
  
  // Disable motors initially
  digitalWrite(DC_EN, LOW); 

  connection_setup();
}

void connection_setup() { 
  // Start Serial
  Serial.begin(9600);
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

void loop() {
  checkSerial();
  checkWiFi();

  controlMotor();
}

void blink_led(int counter, int ledPin) {
  // exception
  if(counter < 1 || counter > 20) return;

  for(int i = 1; i <= counter; i++) {
    digitalWrite(ledPin, HIGH);
    delay(1000);
    digitalWrite(ledPin, LOW);
    delay(1000);
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
  if (client) {
    String command = client.readStringUntil('\n');
    command.trim();
    command.toLowerCase();

    Serial.print("WiFi received: ");
    Serial.println(command);

    String response = handleCommand(command);
    client.println(response);
    client.stop();
  }
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
  } else if (cmd == "motor forward") {
    // insert code logic for moving forward with the motor
    return "Motor forward command received";
  }
  else if (cmd == "motor backward") {
    // insert code logic for moving backward with the motor
    return "Motor forward command received";
  }
  else if (cmd == "turn left") {
    turnLeft();
    return "Stepper turned left.";
  }
  else if (cmd == "turn right") {
    turnRight();
    return "Stepper turned right.";
  }
  else if (cmd.startsWith("rotate ")) {
    int steps = cmd.substring(7).toInt();  // Extract number after "rotate "
    if (steps == 0) {
      return "Invalid step count.";
    }
    bool direction = (steps > 0);
    stepperStep(direction, abs(steps));  // Use absolute value for step count
    return "Rotated " + String(steps) + " steps.";
  }
  else if (cmd == "start") {
    motorState = MOTOR_FORWARD;
    isMotorRunning = true;
    return "Start test movement for motors command received.";
  }
  else if (cmd == "stop") {
    motorState = MOTOR_IDLE;
    isMotorRunning = false;
    stopMotors();
    return "Stop command received.";
  }
  else if (cmd == "stop step") {
    // implement stepper stop
    return "Stop stepper command received.";
  }
  else if (cmd == "wifi") {
    Serial.print("WiFi ready. Access Point IP: ");
    Serial.println(WiFi.localIP());
    return "Checked wifi.";
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
      moveForward();
      if (now - motorStartTime >= driveDuration) {
        stopMotors();
        motorStartTime = now;
        motorState = MOTOR_PAUSE;
      }
      break;

    case MOTOR_BACKWARD:
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
      stopMotors();
      break;
  }
}

void moveForward() {
  digitalWrite(DC_IN1, HIGH);
  digitalWrite(DC_IN2, LOW);
  digitalWrite(DC_IN3, LOW);
  digitalWrite(DC_IN4, HIGH);
  digitalWrite(DC_EN, HIGH);
  wasLastForward = true;
}

void moveBackward() {
  digitalWrite(DC_IN1, LOW);
  digitalWrite(DC_IN2, HIGH);
  digitalWrite(DC_IN3, HIGH);
  digitalWrite(DC_IN4, LOW);
  digitalWrite(DC_EN, HIGH);
  wasLastForward = false;
}

void stopMotors() {
  digitalWrite(DC_EN, LOW);
  digitalWrite(DC_IN1, LOW);
  digitalWrite(DC_IN2, LOW);
  digitalWrite(DC_IN3, LOW);
  digitalWrite(DC_IN4, LOW);
}

void turnLeft() {
  stepperStep(false, 200); // Move 200 steps in "left" direction
}

void turnRight() {
  stepperStep(true, 200);  // Move 200 steps in "right" direction
}
