// definitions
#define ARDUINO_LED 13

// stepper definitions
#define STEPPER1_STEP 4
#define STEPPER1_DIR 3
#define STEPPER2_STEP 6
#define STEPPER2_DIR 5
#define STEPPER_EN 12

// Motor states
#define MOTOR_IDLE 0
#define MOTOR_RUNNING 1

// Global state variables
int motorState = MOTOR_IDLE;
bool isMotorRunning = false;

// Movement control variables
bool dirStepper1 = false;
int stepsLeft = 0;
unsigned int stepDelayMicros = 800;
unsigned long lastStepTime = 0;
bool stepPinHigh = false;

void setup() {
    pins_setup();
    connection_setup();
}

void pins_setup() {
    pinMode(ARDUINO_LED, OUTPUT);
    digitalWrite(ARDUINO_LED, LOW);

    pinMode(STEPPER1_DIR, OUTPUT);
    pinMode(STEPPER1_STEP, OUTPUT);
    digitalWrite(STEPPER1_DIR, LOW);
    digitalWrite(STEPPER1_STEP, LOW);

    pinMode(STEPPER2_DIR, OUTPUT);
    pinMode(STEPPER2_STEP, OUTPUT);
    digitalWrite(STEPPER2_DIR, LOW);
    digitalWrite(STEPPER2_STEP, LOW);

    pinMode(STEPPER_EN, OUTPUT);
    digitalWrite(STEPPER_EN, HIGH); // disable motors initially
}

void connection_setup() {
    Serial.begin(9600);
    delay(2000);
    blink_led(1, ARDUINO_LED);
}

void blink_led(int counter, int ledPin) {
    if (counter < 1 || counter > 20) return;
    for (int i = 1; i <= counter; i++) {
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
        Serial.println(response);
    }
}

String handleCommand(String cmd) {
    if (cmd == "led on") {
        digitalWrite(ARDUINO_LED, HIGH);
        return "LED was turned on.";
    }
    else if (cmd == "led off") {
        digitalWrite(ARDUINO_LED, LOW);
        return "LED was turned off.";
    }
    else if (cmd == "ping") {
        return "PONG";
    }
    else if (cmd == "stop") {
        isMotorRunning = false;
        motorState = MOTOR_IDLE;
        digitalWrite(STEPPER_EN, HIGH);  // disable motors immediately
        digitalWrite(STEPPER1_STEP, LOW);
        digitalWrite(STEPPER2_STEP, LOW);
        Serial.println("Motors stopped.");
        return "Motors stopped.";
    }

    if (motorState == MOTOR_RUNNING) {
        return "Motor busy, stop first.";
    }

    // Start motor movement commands here (non-blocking)
    if (cmd == "2turn left") {
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

    return "UNKNOWN COMMAND";
}

void startMovement(bool direction, int steps, int delayMicros) {
    motorState = MOTOR_RUNNING;
    isMotorRunning = true;
    dirStepper1 = direction;
    stepsLeft = steps;
    stepDelayMicros = delayMicros;
    lastStepTime = 0;
    stepPinHigh = false;

    digitalWrite(STEPPER_EN, LOW);  // Enable motor drivers
    digitalWrite(STEPPER1_DIR, dirStepper1 ? HIGH : LOW);
    digitalWrite(STEPPER2_DIR, dirStepper1 ? LOW : HIGH);
}

void performStep() {
    if (!isMotorRunning || motorState == MOTOR_IDLE) {
        return; // nothing to do
    }

    unsigned long now = micros();

    if (stepsLeft <= 0) {
        // Movement done
        isMotorRunning = false;
        motorState = MOTOR_IDLE;
        digitalWrite(STEPPER_EN, HIGH); // disable motors
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
    }
    else if (stepPinHigh && (now - lastStepTime >= stepDelayMicros)) {
        // Step pins go LOW
        digitalWrite(STEPPER1_STEP, LOW);
        digitalWrite(STEPPER2_STEP, LOW);
        lastStepTime = now;
        stepsLeft--;
        stepPinHigh = false;
    }
}

void loop() {
    checkSerial();
    performStep();
}
