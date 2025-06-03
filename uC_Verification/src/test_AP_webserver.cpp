#ifdef TEST_AP_SERVER

#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>

const char* ssid = "ESP32-Web";
const char* password = "TestAPESP32";

WebServer server(80);

unsigned long uploadStartTime = 0;

void handleFileUpload() {
  HTTPUpload& upload = server.upload();

  if (upload.status == UPLOAD_FILE_START) {
    Serial.printf("Upload Start: %s\n", upload.filename.c_str());
    uploadStartTime = millis();  // Start timer
    File file = SPIFFS.open("/" + upload.filename, FILE_WRITE);
    file.close();
  } 
  else if (upload.status == UPLOAD_FILE_WRITE) {
    File file = SPIFFS.open("/" + upload.filename, FILE_APPEND);
    if (file) {
      file.write(upload.buf, upload.currentSize);
      file.close();
    }
  } 
  else if (upload.status == UPLOAD_FILE_END) {
    unsigned long uploadDuration = millis() - uploadStartTime;  // Stop timer
    Serial.printf("Upload End: %s (%u bytes)\n", upload.filename.c_str(), upload.totalSize);
    Serial.printf("Upload Time: %lu ms (%.2f seconds)\n", uploadDuration, uploadDuration / 1000.0);
    server.send(200, "text/plain", "File Uploaded Successfully");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting ESP32 Access Point...");

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed!");
    return;
  }
  Serial.println("SPIFFS Mounted Succesfully!");
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html",
      "<h2>ESP32 File Upload</h2>"
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
