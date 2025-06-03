#ifdef TEST_WEBSERVER_HOTSPOT

#include <WiFi.h>
#include <SPIFFS.h>
#include <WebServer.h>

WebServer server(80);

#ifdef USE_HOTSPOT_WIFI
// Phone Hotspot Credentials
const char* ssid = "DamegaHotspot";
const char* password = "5414993391";
#else
#include "esp_wpa2.h"
// WPA2-Enterprise (e.g. OSU_Secure)
const char* ssid = "OSU_Secure";
#define EAP_IDENTITY "mejiagad@oregonstate.edu"
#define EAP_USERNAME "mejiagad@oregonstate.edu"
#define EAP_PASSWORD "Skill_Hunter.621"
#endif

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

#ifdef USE_HOTSPOT_WIFI
  WiFi.begin(ssid, password);
#else
  esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)EAP_IDENTITY, strlen(EAP_IDENTITY));
  esp_wifi_sta_wpa2_ent_set_username((uint8_t *)EAP_USERNAME, strlen(EAP_USERNAME));
  esp_wifi_sta_wpa2_ent_set_password((uint8_t *)EAP_PASSWORD, strlen(EAP_PASSWORD));
  esp_wifi_sta_wpa2_ent_enable();
  WiFi.begin(ssid);
#endif

  Serial.println("Connecting to WiFi...");
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
