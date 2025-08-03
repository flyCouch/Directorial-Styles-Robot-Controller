#include <ESPmDNS.h>
#include <WiFiClient.h>

void connectToServer() {
  int n = MDNS.queryService("robot_controller", "tcp");
  if (n == 0) {
    Serial.println("No robot controller service found.");
  } else {
    Serial.printf("%d services found!\n", n);
    
    // Connect to the first service found
    String serverName = MDNS.hostname(0);
    IPAddress serverIP = MDNS.IP(0);
    int serverPort = MDNS.port(0);

    Serial.printf("Found server: %s at IP %s on port %d\n", serverName.c_str(), serverIP.toString().c_str(), serverPort);

    // Now, establish the socket connection
    if (client.connect(serverIP, serverPort)) {
        Serial.println("Connected to the robot controller!");
    } else {
        Serial.println("Connection failed.");
    }
  }
}

void setup() {
  // ... your existing Wi-Fi setup ...

  // Start mDNS
  if (MDNS.begin("camera1")) { /* ... */ }

  // Search for the service
  connectToServer();
}

void loop() {
  if (!client.connected()) {
    // If connection lost, try to reconnect
    connectToServer();
  }
  // ... your other loop code ...
}
