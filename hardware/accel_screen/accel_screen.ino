#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_ADDR 0x3C

#define MPU9250_ADDR 0x68

#define SDA_PIN 8
#define SCL_PIN 9

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

int16_t ax, ay, az;
int16_t gx, gy, gz;

void setup() {
  Serial.begin(115200);

  // Shared I2C bus
  Wire.begin(SDA_PIN, SCL_PIN);

  // OLED init
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED init failed");
    while (1);
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Wake up MPU9250
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  delay(100);

  Serial.println("System ready");
}

void loop() {

  // -------- Read Accelerometer --------
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDR, 6, true);

  ax = (Wire.read() << 8 | Wire.read());
  ay = (Wire.read() << 8 | Wire.read());
  az = (Wire.read() << 8 | Wire.read());

  // -------- Print to Serial --------
  Serial.print("AX: ");
  Serial.println(ax);

  // -------- OLED Display --------
  display.clearDisplay();
  display.setTextSize(2);

  display.setCursor(0, 10);
  display.print("AX:");

  display.setCursor(50, 10);
  display.print(ax);

  display.display();

  delay(200);
}