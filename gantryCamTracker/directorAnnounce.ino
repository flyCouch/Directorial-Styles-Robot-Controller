// In your setup() function
// Find the IP address of your PC's hostname
IPAddress resolvedIP;
if (MDNS.queryHost("your-pc-hostname")) {
  resolvedIP = MDNS.IP(0);
  Serial.print("Resolved PC IP: ");
  Serial.println(resolvedIP);
  // Now you can use this IP for the socket connection
  client.connect(resolvedIP, serverPort);
} else {
  Serial.println("PC hostname not found!");
}
