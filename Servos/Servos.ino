#include <WiFi.h>
#include <ESP32Servo.h>

const char* ssid = "SKINETR2D2";
const char* password = "Volvere3000";

WiFiServer server(12345);

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;

void setup() {
  servo1.attach(26);
  servo2.attach(13);
  servo3.attach(12);
  servo4.attach(14);
  servo5.attach(27);
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  server.begin();
}

void loop() {
 WiFiClient client = server.available();
  if (client) {
    Serial.println("Client connected");
    while (client.connected()) {
      if (client.available()) {
        String angles = client.readStringUntil('\n');
        Serial.println(angles);
        // Parse angles and set robot arm positions here
        int servoIndex = 0;
        int startPos = 0;
        for (int i = 0; i <= angles.length(); i++) {
          if (angles.charAt(i) == ',' || i == angles.length()) {
            float angle = angles.substring(startPos, i).toFloat();
            switch (servoIndex) {
              case 0:
                servo1.write(angle);
                break;
              case 1:
                servo2.write(angle);
                break;
              case 2:
                servo3.write(angle);
                break;
              case 3:
                servo4.write(angle);
                break;
              case 4:
                servo5.write(angle);
                break;
            }
            servoIndex++;
            startPos = i + 1;
          }
        }
        String state = servoIndex == 5 ? "ready" : "moving";
        client.println(state);
      }
    }
    client.stop();
    Serial.println("Client disconnected");
  }
}
