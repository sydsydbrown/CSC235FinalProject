
/*
   Arduino and MPU6050 Accelerometer and Gyroscope Sensor Code
   by Dejan, https://howtomechatronics.com

   Edited by Sydney :)
*/

// neopixel stuff
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel strip(138,6);

#include <Wire.h>
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float prevRoll, prevPitch, prevYaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;


void setup() {
  Serial.begin(19200);
  Wire.begin();                      // initialize comunication
  Wire.beginTransmission(MPU);       // start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // talk to the register 6B
  Wire.write(0x00);                  // make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        // end the transmission

  // neopixel stuff
  strip.begin();
  strip.clear();
  strip.show();

  // call this function if you need to get the IMU error values for your module
  calculate_IMU_error();
  delay(20);
}


void loop() {

  // read ACCELEROMETER data
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  // for a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value

  // calculating ROLL and PITCH from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.47;//AccErrorX; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + AccErrorY; // AccErrorY ~(-1.58)

  // read GYROSCOPE data
  previousTime = currentTime;        // previous time is stored before the actual time read
  currentTime = millis();            // current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // for a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // correct the outputs with the calculated error values
  GyroX = GyroX + -2.43;//GyroErrorX; // GyroErrorX ~(-0.56)
  GyroY = GyroY - GyroErrorY; // GyroErrorY ~(2)
  GyroZ = GyroZ + GyroErrorZ; // GyroErrorZ ~ (-0.8)
  // currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  prevYaw = yaw;
  yaw =  yaw + GyroZ * elapsedTime;
  // complementary filter - combine acceleromter and gyro angle values
  prevRoll = roll;
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  prevPitch = pitch;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;


  /// if statements to change colors
  // check roll and change neopixels to blue
  if (abs(roll-prevRoll) > 2) {
    setStripColor(strip.Color(0, 0, 255));
  }
  // check pitch value and change to white
  if (abs(pitch-prevPitch) > 2) {
    setStripColor(strip.Color(255, 255, 255));
    strip.setBrightness(75);
  }

  // check yaw value and change to purple
  if (abs(yaw-prevYaw) > 2) {
    setStripColor(strip.Color(255, 0, 255));
  }

  // PRINT the values on the serial monitor
  Serial.print(roll);
  Serial.print("/");
  Serial.print(pitch);
  Serial.print("/");
  Serial.println(yaw);
}

void calculate_IMU_error() {
  // we can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // add up all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  // divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // add up all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }

  // divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  // print the error values on the monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}


// neopixel functions
void setStripColor (uint32_t aColor) {
  ///strip.clear();
  for (int i = 0; i <= 100; i+=6) {
    strip.setPixelColor(i, aColor);
  }
  strip.show();
}
