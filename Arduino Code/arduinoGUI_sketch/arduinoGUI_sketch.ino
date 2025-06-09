// libraries
#include <WiFiS3.h>

// definitions
#define ARDUINO_LED 13
#define DC_IN1 2
#define DC_IN2 3
#define DC_EN  4 // enable EnA and EnB
#define DC_IN3 5
#define DC_IN4 6

// variables for WiFi connection
const char* ssid = "ExploRover";
const char* password = "explorover";

WiFiServer server(8888); // Port 8888 is arbitrary but common

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
  // Setup the pins
  pinMode(ARDUINO_LED, OUTPUT);
  digitalWrite(ARDUINO_LED, LOW);

  // Motor pins setup
  pinMode(DC_IN1, OUTPUT);
  pinMode(DC_IN2, OUTPUT);
  pinMode(DC_EN, OUTPUT); 
  pinMode(DC_IN3, OUTPUT);
  pinMode(DC_IN4, OUTPUT);
  digitalWrite(DC_EN, HIGH); // Enable both motors

  Serial.begin(9600); // initialize a serial port
  while (!Serial) {
    ; // wait for Serial to initialize
  }

  Serial.println("Initializing WiFi...");
  int status = WiFi.begin(ssid, password);

  while (status != WL_CONNECTED) {
    Serial.println("Connecting...");
    delay(1000);
    status = WiFi.status();
  }

  Serial.print("WiFi connected. IP address: ");
  Serial.println(WiFi.localIP());

  server.begin();
  Serial.println("TCP server started.");
}

void loop() {
  checkSerial();
  checkWiFi();
  controlMotor();
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
        // Stop the motors
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

void checkSerial() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim(); // remove newline/extra spaces
    handleCommand(command);
  }
}

void checkWiFi() {
  WiFiClient client = server.available();
  if (client) {
    String command = client.readStringUntil('\n');
    command.trim();
    command.toLowerCase();

    String response = handleCommand(command);
    client.println(response);
    client.stop(); // close connection
  }
}

String handleCommand(String cmd) {
  if (cmd == "led on") {
    digitalWrite(ARDUINO_LED, HIGH);
    return "LED ON";
  } else if (cmd == "led off") {
    digitalWrite(ARDUINO_LED, LOW);
    return "LED OFF";
  } else if (cmd == "ping") {
    return "PONG";
  }

  return "UNKNOWN COMMAND";
}
