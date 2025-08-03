void setup() {
    // ... other setup code ...

    // Wi-Fi connection
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.print("ESP32-CAM IP: ");
    Serial.println(WiFi.localIP());

    // Start the mDNS service for the camera
    if (!MDNS.begin("camera1")) {
        // ... error handling ...
    }
    Serial.println("mDNS responder started");

    // --- New: Connect to the PC server via mDNS ---
    IPAddress resolvedIP;
    if (MDNS.queryHost("your-pc-hostname.local")) {
      resolvedIP = MDNS.IP(0);
      Serial.print("Resolved PC IP: ");
      Serial.println(resolvedIP);
      client.connect(resolvedIP, serverPort);
    } else {
      Serial.println("PC hostname not found!");
      // Fallback to old hardcoded IP or handle retry later
      client.connect(serverIP, serverPort); // This line can be a fallback
    }

    startCameraServer();
}
