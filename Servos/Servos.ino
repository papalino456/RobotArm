#include <WiFi.h>
#include <ESP32Servo.h>
#include <ArduinoOTA.h>

const char* ssid = "HUAWEI P30";
const char* password = "papalino";

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
  Serial.println("Connected to WiFi");
  
  // Initialize OTA with callbacks
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }
    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.begin();
}

void loop() {
  ArduinoOTA.handle();
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
                delay(500);
                break;
              case 1:
                servo2.write(angle);
                delay(500);
                break;
              case 2:
                servo3.write(angle);
                delay(500);
                break;
              case 3:
                servo4.write(angle);
                delay(500);
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
        delay(1000);
        client.println(state);
      }
    }
    client.stop();
    Serial.println("Client disconnected");
  }
}
