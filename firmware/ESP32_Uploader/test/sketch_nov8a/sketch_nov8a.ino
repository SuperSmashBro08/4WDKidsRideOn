#include <Wire.h>
#define SDA_PIN 8
#define SCL_PIN 9
#define AS5600  0x36

uint16_t read16(uint8_t reg) {
  Wire.beginTransmission(AS5600);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFFFF;
  if (Wire.requestFrom((int)AS5600, 2) != 2) return 0xFFFF;
  uint16_t hi = Wire.read();
  uint16_t lo = Wire.read();
  return (uint16_t)((hi<<8) | lo);
}

uint16_t raw_angle() {             // RAW_ANGLE = 0x0C/0x0D, 12-bit
  uint16_t v = read16(0x0C);
  if (v == 0xFFFF) return v;
  return v & 0x0FFF;
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nAS5600 raw reader");
  Wire.begin(SDA_PIN, SCL_PIN, 100000);
}

void loop() {
  uint16_t r = raw_angle();
  if (r == 0xFFFF) Serial.println("I2C read FAIL");
  else {
    float deg = (360.0f * r) / 4096.0f;
    Serial.printf("raw=%u  deg=%.2f\n", r, deg);
  }
  delay(100);
}
