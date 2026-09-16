#include "esp_camera.h"
#include <WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ===========================
// Select camera model in board_config.h
// ===========================
#include "board_config.h"

// ===========================
// Enter your WiFi credentials
// ===========================
const char *ssid = "";
const char *password = "";

// Set the LCD address to 0x27 (or 0x3F) for a 16x2 display
LiquidCrystal_I2C lcd(0x27, 16, 2);

void startCameraServer();
void setupLedFlash();

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disables brownout detector
  Serial.begin(115200);
  
  // Required Project Header & Compile Prints
  Serial.println(F("--- PROJECT: Kinetic Eye ---"));
  Serial.print(F("FILE: ")); Serial.println(__FILE__);
  Serial.print(F("DATE: ")); Serial.println(__DATE__);
  Serial.print(F("TIME: ")); Serial.println(__TIME__);
  Serial.println(); 
  
  Serial.setDebugOutput(true);
  Serial.println();

  // Initialize I2C and 1602 LCD on GPIO 15 (SDA) and GPIO 14 (SCL)
  Wire.begin(15, 14);
  lcd.init();                      
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print(F("LYTTLE reSearch"));
  lcd.setCursor(0, 1);
  lcd.print(F("Connecting WiFi"));

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_VGA; 
  config.pixel_format = PIXFORMAT_JPEG;  
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      config.frame_size = FRAMESIZE_QQVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Cam Init Failed"));
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);        
    s->set_brightness(s, 1);   
    s->set_saturation(s, -2);  
  }
  // s->set_framesize(s, FRAMESIZE_VGA); // Left commented out so webpage handles sizes

#if defined(LED_GPIO_NUM)
  setupLedFlash();
#endif

  // Connect to WiFi via DHCP
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  Serial.print("WiFi connecting");
  int dots = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    lcd.setCursor(13, 1);
    lcd.print(F("   "));
    lcd.setCursor(13, 1);
    for(int i = 0; i < (dots % 3) + 1; i++) lcd.print(F("."));
    dots++;
  }
  Serial.println("");
  Serial.println("WiFi connected");

  // Completely clear the LCD to remove debris, then print IP cleanly
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Kinetic Eye IP:"));
  lcd.setCursor(0, 1);
  lcd.print(WiFi.localIP());

  // Print final IP to Serial
  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");

  // Hold display active for 15 seconds safely before server starts
  delay(15000);
  
  // Completely blank text, turn off backlight, and shut down I2C usage
  lcd.clear();
  lcd.noDisplay();
  lcd.noBacklight();

  // NOW start the camera server so it runs without any I2C interference
  startCameraServer();
}

void loop() {
  // Left completely empty so the web server task runs without interruption
  vTaskDelay(portMAX_DELAY);
}
