// definitions
#define ARDUINO_LED 13

// stepper definitions
#define STEPPER1_STEP 4
#define STEPPER1_DIR 3
#define STEPPER2_STEP 6
#define STEPPER2_DIR 5
#define STEPPER_EN 12

#define DIR_RAISE_STEPPER1 HIGH
#define DIR_RAISE_STEPPER2 LOW

// stepper function declarations
void stepperStep(int dirPin, int stepPin, bool direction, int steps, int delayMicros = 800);
void dualStepperStep(bool dirStepper1, int steps, int delayMicros = 800);

void setup() {
    pins_setup();
    connection_setup();
}

void pins_setup() {
    pinMode(ARDUINO_LED, OUTPUT);
    digitalWrite(ARDUINO_LED, LOW);

    int dirPins[] = { STEPPER1_DIR, STEPPER2_DIR };
    int stepPins[] = { STEPPER1_STEP, STEPPER2_STEP };
    for (int i = 0; i < 2; i++) {
        pinMode(dirPins[i], OUTPUT);
        pinMode(stepPins[i], OUTPUT);
        digitalWrite(dirPins[i], LOW);
        digitalWrite(stepPins[i], LOW);
    }

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

    // Stepper commands ----------------------

    else if (cmd == "turn left") {
        digitalWrite(STEPPER_EN, LOW);
        dualStepperStep(false, 200);
        digitalWrite(STEPPER_EN, HIGH);
        return "Turned left.";
    }
    else if (cmd == "turn right") {
        digitalWrite(STEPPER_EN, LOW);
        dualStepperStep(true, 200);
        digitalWrite(STEPPER_EN, HIGH);
        return "Turned right.";
    }
    else if (cmd.startsWith("rotate ")) {
        int steps = cmd.substring(7).toInt();
        if (steps == 0) return "Invalid step count.";
        bool direction = (steps > 0);
        digitalWrite(STEPPER_EN, LOW);
        dualStepperStep(direction, abs(steps));
        digitalWrite(STEPPER_EN, HIGH);
        return "Rotated " + String(steps) + " steps.";
    }
    else if (cmd == "raise") {
        digitalWrite(STEPPER_EN, LOW);

        // Explicit, fixed directions
        digitalWrite(STEPPER1_DIR, HIGH);  // Always same
        digitalWrite(STEPPER2_DIR, LOW);   // Always opposite
        delayMicroseconds(50);

        for (int i = 0; i < 200; i++) {
            digitalWrite(STEPPER1_STEP, HIGH);
            digitalWrite(STEPPER2_STEP, HIGH);
            delayMicroseconds(800);

            digitalWrite(STEPPER1_STEP, LOW);
            digitalWrite(STEPPER2_STEP, LOW);
            delayMicroseconds(800);
        }

        digitalWrite(STEPPER_EN, HIGH);
        return "Explorover raised.";
    }
    else if (cmd == "stop step") {
        return "Stop stepper command received.";
    }

    return "UNKNOWN COMMAND";
}

void stepperStep(int dirPin, int stepPin, bool direction, int steps, int delayMicros) {
    digitalWrite(dirPin, direction ? HIGH : LOW);
    for (int i = 0; i < steps; i++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(delayMicros);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(delayMicros);
    }
}

void dualStepperStep(bool dirStepper1, int steps, int delayMicros) {
    // Rotate stepper 1 in dirStepper1 and stepper 2 in the opposite direction
    digitalWrite(STEPPER1_DIR, dirStepper1 ? HIGH : LOW);
    digitalWrite(STEPPER2_DIR, dirStepper1 ? LOW : HIGH);

    for (int i = 0; i < steps; i++) {
        digitalWrite(STEPPER1_STEP, HIGH);
        digitalWrite(STEPPER2_STEP, HIGH);
        delayMicroseconds(delayMicros);

        digitalWrite(STEPPER1_STEP, LOW);
        digitalWrite(STEPPER2_STEP, LOW);
        delayMicroseconds(delayMicros);
    }
}

void loop() {
    checkSerial();
}
