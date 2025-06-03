
#ifndef SERVER_HANDLERS_H
#define SERVER_HANDLERS_H

#include <WebServer.h>

void handleRoot(WebServer &server);
void handlePresetUpload(WebServer &server);
void setupWebRoutes(WebServer &server);

#endif // SERVER_HANDLERS_H
