// definitions
#define ARDUINO_LED 13
#define DC_IN1 2
#define DC_IN2 3
#define DC_EN  4 // enable EnA and EnB
#define DC_IN3 5
#define DC_IN4 6

// variables for motor control
enum MotorState {
  MOTOR_IDLE,
  MOTOR_FORWARD,
  MOTOR_BACKWARD,
  MOTOR_DONE
};

MotorState motorState = MOTOR_FORWARD;
unsigned long motorStartTime = 0;

void setup() {
  // Setup LED and motor pins
  pinMode(ARDUINO_LED, OUTPUT);
  digitalWrite(ARDUINO_LED, LOW);

  pinMode(DC_IN1, OUTPUT);
  pinMode(DC_IN2, OUTPUT);
  pinMode(DC_EN, OUTPUT); 
  pinMode(DC_IN3, OUTPUT);
  pinMode(DC_IN4, OUTPUT);
  digitalWrite(DC_EN, HIGH); // Enable motors initially

  // Start Serial
  Serial.begin(9600);
  // while (!Serial); // Wait for Serial connection
  Serial.println("Serial mode ready. Waiting for commands...");
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
  }

  return "UNKNOWN COMMAND";
}

void controlMotor() {
  unsigned long currentMillis = millis();

  switch (motorState) {
    case MOTOR_FORWARD:
      digitalWrite(DC_IN1, HIGH);
      digitalWrite(DC_IN2, LOW);
      digitalWrite(DC_IN3, HIGH);
      digitalWrite(DC_IN4, LOW);
      digitalWrite(DC_EN, HIGH);
      motorStartTime = currentMillis;
      motorState = MOTOR_BACKWARD;
      break;

    case MOTOR_BACKWARD:
      if (currentMillis - motorStartTime >= 5000) {
        digitalWrite(DC_IN1, LOW);
        digitalWrite(DC_IN2, HIGH);
        digitalWrite(DC_IN3, LOW);
        digitalWrite(DC_IN4, HIGH);
        digitalWrite(DC_EN, HIGH);
        motorStartTime = currentMillis;
        motorState = MOTOR_DONE;
      }
      break;

    case MOTOR_DONE:
      if (currentMillis - motorStartTime >= 5000) {
        digitalWrite(DC_IN1, LOW);
        digitalWrite(DC_IN2, LOW);
        digitalWrite(DC_IN3, LOW);
        digitalWrite(DC_IN4, LOW);
        digitalWrite(DC_EN, LOW);
        motorState = MOTOR_IDLE;
      }
      break;

    case MOTOR_IDLE:
    default:
      // Do nothing
      break;
  }
}
