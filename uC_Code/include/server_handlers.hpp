
#ifndef SERVER_HANDLERS_H
#define SERVER_HANDLERS_H

#include <WebServer.h>
#include <FS.h>
#include <SPIFFS.h>
#include <WiFi.h>

void handleRoot(WebServer &server);
void handlePresetUpload(WebServer &server);
void setupWebRoutes(WebServer &server);
void handleEmergencyStop(WebServer &server);

#endif // SERVER_HANDLERS_H
