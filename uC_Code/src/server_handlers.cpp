
#include "server_handlers.hpp"


extern void handleRoot(WebServer &server) {
    server.sendHeader("Location", "/index.html", true);
    server.send(302, "text/plain", "");
}

extern void handlePresetUpload(WebServer &server) {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
        String filename = "/preset" + upload.filename.substring(upload.filename.length() - 5, upload.filename.length() - 4) + ".gcode";
        fs::File file = SPIFFS.open(filename, FILE_WRITE);
        file.close();
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        String filename = "/preset" + upload.filename.substring(upload.filename.length() - 5, upload.filename.length() - 4) + ".gcode";
        fs::File file = SPIFFS.open(filename, FILE_APPEND);
        if (file) file.write(upload.buf, upload.currentSize);
        file.close();
    }
}

extern void setupWebRoutes(WebServer &server) {
    server.on("/", HTTP_GET, [&]() { handleRoot(server); });
    server.on("/emergency_stop", HTTP_POST, [&]() { handleEmergencyStop(server); });
    server.on("/upload_preset", HTTP_POST, []() {}, [&server]() { handlePresetUpload(server); });
}


extern void handleEmergencyStop(WebServer &server) {
    extern volatile bool emergency_triggered;
    emergency_triggered = true;
    server.send(200, "text/plain", "Emergency Stop Triggered");
}
