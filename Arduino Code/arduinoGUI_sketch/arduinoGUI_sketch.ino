// definitions
#define ARDUINO_LED 13
#define DC_IN1 4
#define DC_IN2 5
#define DC_EN  3 // enable EnA and EnB
#define DC_IN3 6
#define DC_IN4 7

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
  
  // Disable motors initially
  digitalWrite(DC_EN, LOW); 

  // Start Serial
  Serial.begin(9600);
  while (!Serial); // Wait for Serial connection

  // test once
  //String response = handleCommand("start");
  //Serial.println(response); // Echo response for debugging
}

void loop() {
  checkSerial();
  controlMotor();
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
    // insert code logic for turning left using stepper
    return "Motor forward command received";
  }
  else if (cmd == "turn right") {
    // insert code logic for turning right using stepper
    return "Motor forward command received";
  }
  else if (cmd == "start") {
    motorState = MOTOR_FORWARD;
    isMotorRunning = true;
    return "Start test movement for motors command received.";
  }
  else if (cmd == "stop") {
    motorState = MOTOR_IDLE;
    isMotorRunning = false;
    return "Stop test movement for motors command received.";
  }

  return "UNKNOWN COMMAND";
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
    default:
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
  digitalWrite(DC_IN1, LOW);
  digitalWrite(DC_IN2, LOW);
  digitalWrite(DC_IN3, LOW);
  digitalWrite(DC_IN4, LOW);
  digitalWrite(DC_EN, LOW);
}
