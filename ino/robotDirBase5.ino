#include <SPI.h>
#include <RF24.h> // Include RF24 library

// Define RF24 radio pins (adjust if different for your setup)
RF24 radio(10, 9); // CE, CSN pins - Common for NRF24L01 on Arduino Nano/Uno

// Radio pipe address (must match the address on the robot Arduino)
const byte address[6] = "00001"; 

// Structure to hold incoming command data from Python and for radio transmission
struct CommandData {
  float x;      // X-axis movement velocity (m/s)
  float y;      // Y-axis movement velocity (m/s)
  float r;      // Rotation velocity (rad/s)
  byte laser;   // Laser state (0=OFF, 1=ON)
  byte power;   // Laser power (0-255, inverted: 255=OFF, 0=ON)
  float speed;  // Overall speed factor (0.0-1.0)
};

// Global buffer for serial input from Python
String serialBuffer = "";
bool newData = false; // Flag to indicate a complete new line has been received

void setup() {
  Serial.begin(115200); // Start serial communication with the Python script
  
  // Initialize RF24 radio
  radio.begin();
  radio.openWritingPipe(address); // Set the address for sending data
  radio.stopListening(); // Put radio in transmitter mode
  
  Serial.println("Base Station Radio Ready");
}

// Function to read a string from serial until a newline character is found.
// This is a non-blocking function, suitable for use in loop().
void readSerialString() {
  while (Serial.available()) {
    char inChar = Serial.read(); // Read one character at a time
    if (inChar == '\n') { // If newline character is received, a complete command is in the buffer
      newData = true; // Set flag to process the data
    } else {
      serialBuffer += inChar; // Append character to the buffer
    }
  }
}

void loop() {
  readSerialString(); // Continuously read serial data

  if (newData) { // If a complete new line has been received
    String command = serialBuffer; // Get the command string
    serialBuffer = ""; // Clear the buffer for the next command
    newData = false;   // Reset the flag

    command.trim(); // Remove leading/trailing whitespace

    Serial.print("Received from Python: ");
    Serial.println(command); // Print the raw received command for debugging

    // --- Parse the command string (e.g., "MX:0.00000000,MY:0.00000000,R:0.00000000,L:0,P:255,S:0.50000000") ---
    float motion_x = 0.0;
    float motion_y = 0.0;
    float rotation = 0.0;
    int laser_on = 0;
    int laser_power = 0;
    float target_speed = 0.0;

    // Find the indices of each parameter
    int mx_index = command.indexOf("MX:");
    int my_index = command.indexOf("MY:");
    int r_index = command.indexOf("R:");
    int l_index = command.indexOf("L:");
    int p_index = command.indexOf("P:");
    int s_index = command.indexOf("S:"); 

    // Check if all expected parameters are found
    if (mx_index != -1 && my_index != -1 && r_index != -1 && 
        l_index != -1 && p_index != -1 && s_index != -1) {
      
      // Extract and convert values using substring and toFloat/toInt
      // The substring end index is the start of the *next* parameter, ensuring correct parsing
      motion_x = command.substring(mx_index + 3, my_index).toFloat();
      motion_y = command.substring(my_index + 3, r_index).toFloat();
      rotation = command.substring(r_index + 2, l_index).toFloat();
      laser_on = command.substring(l_index + 2, p_index).toInt();
      laser_power = command.substring(p_index + 2, s_index).toInt(); 
      target_speed = command.substring(s_index + 2).toFloat(); // Read to the end of the string

      // Print parsed values for debugging
      Serial.print("Parsed: X="); Serial.print(motion_x, 8);
      Serial.print(", Y="); Serial.print(motion_y, 8);
      Serial.print(", R="); Serial.print(rotation, 8);
      Serial.print(", L="); Serial.print(laser_on);
      Serial.print(", P="); Serial.print(laser_power); // This should now be 0 or 255 correctly
      Serial.print(", S="); Serial.println(target_speed, 8);

      // --- Package data for radio transmission ---
      CommandData dataToSend;
      dataToSend.x = motion_x;
      dataToSend.y = motion_y;
      dataToSend.r = rotation;
      dataToSend.laser = (byte)laser_on;
      dataToSend.power = (byte)laser_power; // This will now be the correctly parsed inverted value
      dataToSend.speed = target_speed;

      // Attempt to send data over nRF24L01 radio
      if (radio.write(&dataToSend, sizeof(dataToSend))) {
        Serial.println("Data packaged and sent via radio");
        // Optionally send a success message back to Python if needed
        // Serial.println("Radio Success: 1"); 
      } else {
        Serial.println("Radio transmission failed");
        // Optionally send a failure message back to Python
        // Serial.println("Radio Success: 0");
      }
    } else {
      Serial.print("Error: Incomplete or malformed command received: ");
      Serial.println(command); // Print the problematic command
    }
  }
}
