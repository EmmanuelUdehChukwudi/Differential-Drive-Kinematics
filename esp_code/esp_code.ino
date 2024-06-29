#include <WiFi.h>
#include <HTTPClient.h>

// Wi-Fi credentials
const char* ssid = "Emmanuel prime";
const char* password = "55105322G";

// Server URLs
const char* updateSpeedsServer = "http://192.168.43.81:5000/update_speeds";
const char* getSetpointServer = "http://192.168.43.81:5000/get_setpoint";
const char* getUpdateRateServer = "http://192.168.43.81:5000/get_update_rate";

// Pins for serial communication
const int RXPin = 16;
const int TXPin = 17;

float set_point = 0.0; 
int interval_send = 1000; // Default update rate

unsigned long last_time_send = 0;
unsigned long last_time_fetch = 0;
const int interval_fetch = 1000; 

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, RXPin, TXPin);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  fetchUpdateRate();
}

void loop() {
  // Check if data is available from Nano
  if (Serial2.available()) {
    String dataFromNano = Serial2.readStringUntil('\n');
    dataFromNano.trim();
    Serial.println("Received from Nano: " + dataFromNano);

    // Send the wheel speeds to Flask server
    if (WiFi.status() == WL_CONNECTED) {
      HTTPClient http;
      http.begin(updateSpeedsServer);

      http.addHeader("Content-Type", "application/json");
      String httpRequestData = "{\"wheel_speeds\":\"" + dataFromNano + "\"}";
      Serial.println("Sending POST request: " + httpRequestData);
      int httpResponseCode = http.POST(httpRequestData);

      if (httpResponseCode > 0) {
        String response = http.getString();
        Serial.println("HTTP Response code: " + String(httpResponseCode));
        Serial.println("Response: " + response);
      } else {
        Serial.println("Error on sending POST: " + String(httpResponseCode));
      }

      http.end();
    } else {
      Serial.println("WiFi Disconnected");
    }
  }

  // Fetch the desired setpoint from the server
  unsigned long current_time_fetch = millis();
  if (current_time_fetch - last_time_fetch >= interval_fetch) {
    last_time_fetch = current_time_fetch;

    if (WiFi.status() == WL_CONNECTED) {
      HTTPClient http;
      http.begin(getSetpointServer);

      int httpResponseCode = http.GET();

      if (httpResponseCode > 0) {
        String response = http.getString();
        Serial.println("HTTP Response code: " + String(httpResponseCode));
        Serial.println("Setpoint Response: " + response);

        // Parse the JSON response
        int startIdx = response.indexOf("set_point") + 11;
        int endIdx = response.indexOf("}", startIdx);
        set_point = response.substring(startIdx, endIdx).toFloat();
        Serial.println("Updated Setpoint: " + String(set_point));
      } else {
        Serial.println("Error on GET: " + String(httpResponseCode));
      }

      http.end();
    } else {
      Serial.println("WiFi Disconnected");
    }
  }

  // Send the desired setpoint to Nano
  unsigned long current_time_send = millis();
  if (current_time_send - last_time_send >= interval_send) {
    last_time_send = current_time_send;
    Serial2.println(set_point); 
  }
}

void fetchUpdateRate() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(getUpdateRateServer);

    int httpResponseCode = http.GET();

    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println("HTTP Response code: " + String(httpResponseCode));
      Serial.println("Update Rate Response: " + response);

      // Parse the JSON response
      int startIdx = response.indexOf("update_rate") + 13;
      int endIdx = response.indexOf("}", startIdx);
      interval_send = response.substring(startIdx, endIdx).toInt();
      Serial.println("Updated Update Rate: " + String(interval_send));
    } else {
      Serial.println("Error on GET: " + String(httpResponseCode));
    }

    http.end();
  } else {
    Serial.println("WiFi Disconnected");
  }
}
