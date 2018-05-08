#include <Wire.h> //The I2C library
        
int gyroResult[3], accelResult[3];

//Function for writing a byte to an address on an I2C device
void writeTo(byte device, byte toAddress, byte val) {
  Wire.beginTransmission(device);  
  Wire.write(toAddress);        
  Wire.write(val);        
  Wire.endTransmission();
}

//Function for reading num bytes from addresses on an I2C device
void readFrom(byte device, byte fromAddress, int num, byte result[]) {
  Wire.beginTransmission(device);
  Wire.write(fromAddress);
  Wire.endTransmission();
  Wire.requestFrom((int)device, num);
  int i = 0;
  while(Wire.available()) {
    result[i] = Wire.read();
    i++;
  }
}

//Function for reading the gyros.
void getGyroscopeReadings(int gyroResult[]) {
  byte buffer[6];
  readFrom(0x68,0x1D,6,buffer);
  gyroResult[0] = (((int)buffer[0]) << 8 ) | buffer[1]; //Combine two bytes into one int
  gyroResult[1] = (((int)buffer[2]) << 8 ) | buffer[3];
  gyroResult[2] = (((int)buffer[4]) << 8 ) | buffer[5];
} 

//Function for reading the accelerometers
void getAccelerometerReadings(int accelResult[]) {
  byte buffer[6];
  readFrom(0x53,0x32,6,buffer);
  accelResult[0] = (((int)buffer[1]) << 8 ) | buffer[0]; //Yes, byte order different from gyros'
  accelResult[1] = (((int)buffer[3]) << 8 ) | buffer[2];
  accelResult[2] = (((int)buffer[5]) << 8 ) | buffer[4];
}

void setup() {
  Wire.begin();            //Open I2C communications as master
  Serial.begin(115200);    //Open serial communications to the PC to see what's happening
  
  writeTo(0x53,0x31,0x09); //Set accelerometer to 11bit, +/-4g
  writeTo(0x53,0x2D,0x08); //Set accelerometer to measure mode
  writeTo(0x68,0x16,0x1A); //Set gyro to +/-2000deg/sec and 98Hz low pass filter
  writeTo(0x68,0x15,0x09); //Set gyro to 100Hz sample rate
}

void loop() {
  getGyroscopeReadings(gyroResult);
  getAccelerometerReadings(accelResult);

  Serial.print(gyroResult[0]);
  Serial.print("\t");
  Serial.print(gyroResult[1]);
  Serial.print("\t"); 
  Serial.print(gyroResult[2]);
  Serial.print("\t\t");
  Serial.print(accelResult[0]);
  Serial.print("\t");
  Serial.print(accelResult[1]);
  Serial.print("\t");
  Serial.print(accelResult[2]);
  Serial.print("\n");

  delay(50);
}
