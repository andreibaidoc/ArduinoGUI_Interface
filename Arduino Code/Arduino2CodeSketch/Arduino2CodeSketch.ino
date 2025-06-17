// definitions
#define ARDUINO_LED 13

// stepper definitions
#define STEPPER1_STEP 10
#define STEPPER1_DIR 11
#define STEPPER1_EN 12
#define STEPPER2_STEP 10
#define STEPPER2_DIR 11
#define STEPPER2_EN 12
#define STEPPER3_STEP 10
#define STEPPER3_DIR 11
#define STEPPER3_EN 13
#define STEPPER4_STEP 10
#define STEPPER4_DIR 11
#define STEPPER4_EN 13

// stepper function declaration
void stepperStep(int stepper_dir, int stepper_step, bool direction, int steps, int delayMicros = 800);

// ---------------------------------------------------------------------------

void setup() {
  pins_setup();
  connection_setup();
}

void pins_setup() {
  // Setup LED pins
  pinMode(ARDUINO_LED, OUTPUT);
  digitalWrite(ARDUINO_LED, LOW);

  // Stepper activation
  pinMode(STEPPER_DIR, OUTPUT);
  pinMode(STEPPER_STEP, OUTPUT);
  pinMode(STEPPER_EN, OUTPUT);
  digitalWrite(STEPPER_DIR, LOW);
  digitalWrite(STEPPER_STEP, LOW);
  digitalWrite(STEPPER_EN, HIGH);
  
  // Disable motors initially
  digitalWrite(DC_EN, LOW); 
}

void connection_setup() { 
  // Start Serial
  Serial.begin(9600);
  delay(2000); // Allow Serial time to initialize
  blink_led(1, ARDUINO_LED);

  // implement serial connection between arduinos here ... ------------------
}

// ---------------------------------------------------------------------------

void loop() {
  checkSerial();
  controlMotor();
}

// ---------------------------------------------------------------------------

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
  
  // Stepper commands ----------------------
  
  else if (cmd == "turn left") {
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
  } else if (cmd == "raise") { 
    digitalWrite(STEPPER_EN1, LOW);
    // put all of the steppers...

    stepperStep(STEPPER1_DIR, STEPPER1_STEP, true, 200);  // Move 200 steps in "right" direction - turn right command
    stepperStep(STEPPER2_DIR, STEPPER2_STEP, true, 200);
    stepperStep(STEPPER3_DIR, STEPPER3_STEP, true, -200);
    stepperStep(STEPPER4_DIR, STEPPER4_STEP, true, -200);

    digitalWrite(STEPPER_EN, HIGH);
    // put all of the steppers...

    return "Explorover raised.";
  } 
  else if (cmd == "stop step") {
    // implement stepper stop
    return "Stop stepper command received.";
  }

  return "UNKNOWN COMMAND";
}

void stepperStep(int stepper_dir, int stepper_step, bool direction, int steps, int delayMicros) {
  digitalWrite(stepper_dir, direction ? HIGH : LOW);
  for (int i = 0; i < steps; i++) {
    digitalWrite(stepper_step, HIGH);
    delayMicroseconds(delayMicros);
    digitalWrite(stepper_step, LOW);
    delayMicroseconds(delayMicros);
  }
}
