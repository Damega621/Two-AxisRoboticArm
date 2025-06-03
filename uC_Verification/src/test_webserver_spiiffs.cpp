#ifdef TEST_WEBSERVER

#include <WiFi.h>
#include "esp_wpa2.h"  // WPA2 Enterprise support
#include <SPIFFS.h>
#include <WebServer.h>

const char* ssid = "OSU_Secure";  // Replace with actual WPA2-Enterprise SSID
#define EAP_IDENTITY "mejiagad@oregonstate.edu"
#define EAP_USERNAME "mejiagad@oregonstate.edu"
#define EAP_PASSWORD "Skill_Hunter.621"

WebServer server(80);

void handleFileUpload() {
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    Serial.printf("Upload Start: %s\n", upload.filename.c_str());
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    Serial.printf("Upload Progress: %u bytes\n", upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END) {
    Serial.printf("Upload End: %u bytes\n", upload.totalSize);
  }
}

void setup() {
  Serial.begin(115200);
  
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed");
    return;
  }
  Serial.println("SPIFFS Mount Successful");

  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);

  // WPA2-Enterprise Configuration
  esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)EAP_IDENTITY, strlen(EAP_IDENTITY));
  esp_wifi_sta_wpa2_ent_set_username((uint8_t *)EAP_USERNAME, strlen(EAP_USERNAME));
  esp_wifi_sta_wpa2_ent_set_password((uint8_t *)EAP_PASSWORD, strlen(EAP_PASSWORD));

  esp_wifi_sta_wpa2_ent_enable();

  WiFi.begin(ssid);

  Serial.println("Connecting to WPA2-Enterprise WiFi...");

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected.");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi connection failed!");
    return;
  }

  // Start Web Server
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html",
      "<form method='POST' action='/upload' enctype='multipart/form-data'>"
      "<input type='file' name='upload'><input type='submit' value='Upload'></form>");
  });

  server.on("/upload", HTTP_POST, []() {
    server.send(200);
  }, handleFileUpload);

  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
}

#endif