// libraries
#include <WiFiS3.h>

// definitions
#define ARDUINO_LED 13

const char* ssid = "ExploRover";
const char* password = "explorover";

WiFiServer server(8888); // Port 8888 is arbitrary but common

void setup() {
  pinMode(ARDUINO_LED, OUTPUT);
  digitalWrite(ARDUINO_LED, LOW);

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
