#include <SPI.h>
#include <RF24.h> // Example

RF24 radio(10, 9); // CE, CSN - Adjust to your pins
const byte address[6] = "00001"; // Radio pipe address

struct CommandData {
  float x;
  float y;
  float r;
  byte laser;
  byte power;
  float speed; 
};

void setup() {
  Serial.begin(115200);
  radio.begin();
  radio.openWritingPipe(address);
  radio.stopListening(); // Transmitter mode
  Serial.println("Base Station Radio Ready");
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    Serial.print("Received from Python: ");
    Serial.println(command);

    // --- Parse the command string ---
    float motion_x = 0.0;
    float motion_y = 0.0;
    float rotation = 0.0;
    int laser_on = 0;
    int laser_power = 0;
    float target_speed = 0.0; // Initialize target speed

    int mx_index = command.indexOf("MX:");
    int my_index = command.indexOf("MY:");
    int r_index = command.indexOf("R:");
    int l_index = command.indexOf("L:");
    int p_index = command.indexOf("P:");
    int s_index = command.indexOf("S:"); // Find the index of 'S:'

    if (mx_index != -1 && my_index != -1 && r_index != -1 && l_index != -1 && p_index != -1 && s_index != -1) {
      motion_x = command.substring(mx_index + 3, my_index).toFloat();
      motion_y = command.substring(my_index + 3, r_index).toFloat();
      rotation = command.substring(r_index + 2, l_index).toFloat();
      laser_on = command.substring(l_index + 2, p_index).toInt();
      laser_power = command.substring(p_index + 2, s_index).toInt(); // Parse before 'S:'
      target_speed = command.substring(s_index + 2).toFloat(); // Parse target speed

      Serial.print("Parsed: X="); Serial.print(motion_x);
      Serial.print(", Y="); Serial.print(motion_y);
      Serial.print(", R="); Serial.print(rotation);
      Serial.print(", L="); Serial.print(laser_on);
      Serial.print(", P="); Serial.print(laser_power);
      Serial.print(", S="); Serial.println(target_speed);

      // --- Package data for radio transmission ---
      CommandData dataToSend;
      dataToSend.x = motion_x;
      dataToSend.y = motion_y;
      dataToSend.r = rotation;
      dataToSend.laser = (byte)laser_on;
      dataToSend.power = (byte)laser_power;
      dataToSend.speed = target_speed; // Assign the parsed speed

      radio.write(&dataToSend, sizeof(dataToSend));
      Serial.println("Data packaged and sent via radio");
    } else {
      Serial.println("Error parsing command from Python");
    }
  }
  delay(10);
}