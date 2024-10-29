#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266_ISR_Servo.h> // Include ISR Servo library for ESP8266

// Wi-Fi Credentials
const char* ssid = "Your_SSID";
const char* password = "Your_Password";

// Servo control pins (use GPIO numbers)
#define SERVO1_PIN 0  // D3 is GPIO 0
#define SERVO2_PIN 2  // D4 is GPIO 2
#define SERVO3_PIN 12 // D6 is GPIO 12
#define SERVO4_PIN 13 // D7 is GPIO 13
#define SERVO5_PIN 15 // D8 is GPIO 15

// Define min and max pulse widths in microseconds
#define MIN_MICROS 800
#define MAX_MICROS 2450

// Servo indices
int servo1Index, servo2Index, servo3Index, servo4Index, servo5Index;

// Create web server on port 80
ESP8266WebServer server(80);

// HTML page with sliders and buttons for each servo
String createHTMLPage() {
  String html = "<html><head><title>ESP8266 Servo Control</title></head><body>";
  html += "<h1>ESP8266 Servo Control</h1>";
  html += "<p>Use the sliders to control the servos</p>";
  
  for (int i = 1; i <= 5; i++) {
    html += "<label for='servo" + String(i) + "'>Servo " + String(i) + ":</label>";
    html += "<input type='range' id='servo" + String(i) + "' name='servo" + String(i) + "' min='0' max='180' value='90' oninput='updateServo(" + String(i) + ", this.value)'><br>";
    html += "<button onclick='centerServo(" + String(i) + ")'>Center Servo " + String(i) + "</button><br><br>";
  }

  // Add new buttons for specific servo positions
  html += "<button onclick='setServos180_0()'>Set Servos to [180,180,0,0]</button><br><br>";
  html += "<button onclick='setServos0_180()'>Set Servos to [0,0,180,180]</button><br><br>";

  html += "<script>function updateServo(servo, angle) {";
  html += "var xhr = new XMLHttpRequest();";
  html += "xhr.open('GET', '/setServo?servo=' + servo + '&angle=' + angle, true);";
  html += "xhr.send();";
  html += "}";

  html += "function centerServo(servo) {";
  html += "updateServo(servo, 90);"; // Center the servo at 90 degrees
  html += "}";

  // JavaScript functions for new buttons
  html += "function setServos180_0() {";
  html += "updateServo(1, 180); updateServo(2, 180); updateServo(3, 0); updateServo(4, 0);";
  html += "}";

  html += "function setServos0_180() {";
  html += "updateServo(1, 0); updateServo(2, 0); updateServo(3, 180); updateServo(4, 180);";
  html += "}";

  html += "</script>";
  
  html += "</body></html>";
  
 return html;
}

// Handler for root
void handleRoot() {
 server.send(200, "text/html", createHTMLPage());
}

// Set servo angle based on the request
void handleSetServo() {
 if (server.hasArg("servo") && server.hasArg("angle")) {
   int servoNum = server.arg("servo").toInt();
   int angle = server.arg("angle").toInt();
   
   // Set the appropriate servo angle
   switch (servoNum) {
     case 1: ISR_Servo.setPosition(servo1Index, angle); break;
     case 2: ISR_Servo.setPosition(servo2Index, angle); break;
     case 3: ISR_Servo.setPosition(servo3Index, angle); break;
     case 4: ISR_Servo.setPosition(servo4Index, angle); break;
     case 5: ISR_Servo.setPosition(servo5Index, angle); break;
     default: break;
   }
   server.send(200, "text/plain", "OK");
 } else {
   server.send(400, "text/plain", "Invalid Request");
 }
}

void setup() {
 Serial.begin(115200);

 // Connect to Wi-Fi
 WiFi.begin(ssid, password);
 while (WiFi.status() != WL_CONNECTED) {
   delay(500);
   Serial.print(".");
 }
 
 Serial.println("");
 Serial.print("Connected to WiFi. IP Address: ");
 Serial.println(WiFi.localIP());

 // Initialize ISR Servo library and add servos
 servo1Index = ISR_Servo.setupServo(SERVO1_PIN, MIN_MICROS, MAX_MICROS);
 servo2Index = ISR_Servo.setupServo(SERVO2_PIN, MIN_MICROS, MAX_MICROS);
 servo3Index = ISR_Servo.setupServo(SERVO3_PIN, MIN_MICROS, MAX_MICROS);
 servo4Index = ISR_Servo.setupServo(SERVO4_PIN, MIN_MICROS, MAX_MICROS);
 servo5Index = ISR_Servo.setupServo(SERVO5_PIN, MIN_MICROS, MAX_MICROS);

 // Configure server routes
 server.on("/", handleRoot);
 server.on("/setServo", handleSetServo);
 
 server.begin();
 
 Serial.println("Server started");
}

void loop() {
 server.handleClient();
}