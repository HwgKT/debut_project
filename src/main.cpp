#include <Arduino.h>
#include <Wire.h>

namespace {

constexpr uint8_t kMpu6050Address = 0x68;
constexpr uint8_t kAlternateMpu6050Address = 0x69;
constexpr uint8_t kWhoAmIRegister = 0x75;
constexpr uint8_t kPowerManagementRegister = 0x6B;
constexpr uint8_t kSampleRateDividerRegister = 0x19;
constexpr uint8_t kConfigurationRegister = 0x1A;
constexpr uint8_t kGyroConfigurationRegister = 0x1B;
constexpr uint8_t kAccelConfigurationRegister = 0x1C;
constexpr uint8_t kSensorDataRegister = 0x3B;
constexpr uint16_t kGyroCalibrationSamples = 500;
constexpr uint32_t kGyroSampleIntervalUs = 10000;
constexpr uint32_t kPrintIntervalMs = 200;

uint8_t mpuAddress = kMpu6050Address;
uint32_t lastGyroSampleUs = 0;
uint32_t lastPrintMs = 0;
float gyroZOffset = 0.0f;
float yawDegrees = 0.0f;
bool sensorReady = false;



bool readRegister(uint8_t address, uint8_t reg, uint8_t &value)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0 ||
      Wire.requestFrom(address, static_cast<uint8_t>(1), static_cast<uint8_t>(true)) != 1 ||
      Wire.available() < 1) {
    return false;
  }

  value = static_cast<uint8_t>(Wire.read());
  return true;
}

bool writeRegister(uint8_t address, uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool findMpu6050()
{
  const uint8_t addresses[] = {kMpu6050Address, kAlternateMpu6050Address};
  for (uint8_t address : addresses) {
    uint8_t identity = 0;
    if (readRegister(address, kWhoAmIRegister, identity) && identity == 0x68) {
      mpuAddress = address;
      return true;
    }
  }

  return false;
}

bool initializeMpu6050()
{
  return writeRegister(mpuAddress, kPowerManagementRegister, 0x00) &&
         writeRegister(mpuAddress, kConfigurationRegister, 0x03) &&
         writeRegister(mpuAddress, kSampleRateDividerRegister, 0x09) &&
         writeRegister(mpuAddress, kGyroConfigurationRegister, 0x00) &&
         writeRegister(mpuAddress, kAccelConfigurationRegister, 0x00);
}

bool readSensorData(int16_t values[7])
{
  Wire.beginTransmission(mpuAddress);
  Wire.write(kSensorDataRegister);
  if (Wire.endTransmission(false) != 0 ||
      Wire.requestFrom(mpuAddress, static_cast<uint8_t>(14), static_cast<uint8_t>(true)) != 14 ||
      Wire.available() < 14) {
    return false;
  }

  for (uint8_t i = 0; i < 7; ++i) {
    const uint8_t high = static_cast<uint8_t>(Wire.read());
    const uint8_t low = static_cast<uint8_t>(Wire.read());
    values[i] = static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
  }
  return true;
}

bool calibrateGyroZ()
{
  int32_t gyroZTotal = 0;
  int16_t values[7];

  Serial1.println("Calibrating gyro Z; keep the sensor stationary...");
  for (uint16_t i = 0; i < kGyroCalibrationSamples; ++i) {
    if (!readSensorData(values)) {
      Serial1.println("ERROR: Failed to read MPU6050 during gyro calibration");
      return false;
    }
    gyroZTotal += values[6];
    delay(2);
  }

  gyroZOffset = (gyroZTotal / static_cast<float>(kGyroCalibrationSamples)) / 131.0f;
  yawDegrees = 0.0f;
  lastGyroSampleUs = micros();
  Serial1.print("Gyro Z offset: ");
  Serial1.print(gyroZOffset, 3);
  Serial1.println(" deg/s; yaw reset to 0");
  return true;
}

void updateYaw(const int16_t values[7])
{
  const uint32_t now = micros();
  const float deltaTime = (now - lastGyroSampleUs) / 1000000.0f;
  lastGyroSampleUs = now;

  const float gyroZ = values[6] / 131.0f;
  yawDegrees += (gyroZ - gyroZOffset) * deltaTime;
}

void printSensorData(const int16_t values[7])
{
  const float accelX = values[0] / 16384.0f;
  const float accelY = values[1] / 16384.0f;
  const float accelZ = values[2] / 16384.0f;
  const float temperature = values[3] / 340.0f + 36.53f;
  const float gyroX = values[4] / 131.0f;
  const float gyroY = values[5] / 131.0f;
  const float gyroZ = values[6] / 131.0f - gyroZOffset;

  Serial1.print("Accel (g): X=");
  Serial1.print(accelX, 3);
  Serial1.print(" Y=");
  Serial1.print(accelY, 3);
  Serial1.print(" Z=");
  Serial1.print(accelZ, 3);
  Serial1.print(" | Gyro (deg/s): X=");
  Serial1.print(gyroX, 2);
  Serial1.print(" Y=");
  Serial1.print(gyroY, 2);
  Serial1.print(" Z=");
  Serial1.print(gyroZ, 2);
  Serial1.print(" | Temp (C): ");
  Serial1.print(temperature, 2);
  Serial1.print(" | Yaw (deg): ");
  Serial1.println(yawDegrees, 2);
}

}  // namespace

void setup()
{
  Serial1.begin(115200);
  delay(100);
  Serial1.println();
  Serial1.println("STM32F411 - MPU6050 test");
  Serial1.println("UART: USART1 TX=PA9, 115200 baud");
  Serial1.println("I2C: SCL=PB8, SDA=PB9");

  Wire.setSCL(PB8);
  Wire.setSDA(PB9);
  Wire.begin();
  Wire.setClock(100000);

  Serial1.println("Searching for MPU6050...");
  if (!findMpu6050()) {
    Serial1.println("ERROR: MPU6050 not found (check power, wiring, and address 0x68/0x69)");
    return;
  }

  if (!initializeMpu6050()) {
    Serial1.println("ERROR: Failed to configure MPU6050");
    return;
  }

  if (!calibrateGyroZ()) {
    return;
  }

  Serial1.print("MPU6050 ready at I2C address 0x");
  Serial1.println(mpuAddress, HEX);
  Serial1.println("Accel (g) | Gyro corrected (deg/s) | Temp (C) | Yaw (deg)");
  sensorReady = true;
}

void loop()
{
  if (!sensorReady) {
    return;
  }

  static int16_t values[7];
  static bool hasNewSample = false;
  const uint32_t nowUs = micros();
  if (nowUs - lastGyroSampleUs >= kGyroSampleIntervalUs) {
    if (readSensorData(values)) {
      updateYaw(values);
      hasNewSample = true;
    } else {
      Serial1.println("ERROR: Failed to read MPU6050 over I2C");
    }
  }

  const uint32_t nowMs = millis();
  if (hasNewSample && nowMs - lastPrintMs >= kPrintIntervalMs) {
    lastPrintMs = nowMs;
    hasNewSample = false;
    printSensorData(values);
  }
}
