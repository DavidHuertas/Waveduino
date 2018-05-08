//CÓDIGO ArduIMU

#include <Wire.h>

#define ADXL345_ADDRESS (0xA6 >> 1)
#define ADXL345_REGISTER_XLSB (0x32)
#define ADXL345_BW_RATE (0x2C)
#define ADXL345_3200HZ (0x0F)
#define ADXL_REGISTER_PWRCTL (0x2D)
#define ADXL_PWRCTL_MEASURE (1 << 3)


#define ITG3200_ADDRESS (0xD0 >> 1)
#define ITG3200_REGISTER_XMSB (0x1B)
#define ITG3200_REGISTER_DLPF_FS (0x16)
#define ITG3200_FULLSCALE (0x03 << 3)
#define ITG3200_256HZ (0x00)


int accelerometer_data[3];
int gyro_data[3];
int a;
int b;
char c;

void setup() {
  Wire.begin();
  Serial.begin(57600);

  for(int i = 0; i < 3; ++i) {
    accelerometer_data[i] = gyro_data[i] = 0;
  }
  
  init_adxl345();
  init_itg3200();
  
  a=0;
}

void loop() {
  
  a=Serial.read();
  
  if (a=='a'){
  
   while (a!='b')
   {
  
   read_adxl345();
   read_itg3200();

   Serial.println(-accelerometer_data[1]);
   Serial.println(-accelerometer_data[2]);
   Serial.println(-accelerometer_data[0]);   
   Serial.println(gyro_data[0]);
   Serial.println(-gyro_data[2]);
   Serial.println(-gyro_data[3]);
   Serial.println(-gyro_data[1]);
   
   a=Serial.read();
   
   if (a=='b'){
     break;
   }
}
}
}



void i2c_write(int address, byte reg, byte data) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

void i2c_read(int address, byte reg, int count, byte* data) {
 int i = 0;

 Wire.beginTransmission(address);
 Wire.write(reg);
 Wire.endTransmission();
 Wire.beginTransmission(address);
 Wire.requestFrom(address,count);
 while(Wire.available()){
   c = Wire.read();
   data[i] = c;
   i++;
 }
 Wire.endTransmission();
} 

void init_adxl345() {
  byte data = 0;

  i2c_write(ADXL345_ADDRESS, ADXL_REGISTER_PWRCTL, ADXL_PWRCTL_MEASURE);
  i2c_write(ADXL345_ADDRESS, ADXL345_BW_RATE, ADXL345_3200HZ);
  
  i2c_read(ADXL345_ADDRESS, ADXL_REGISTER_PWRCTL, 1, &data);
  Serial.println((unsigned int)data);
}

void read_adxl345() {
 byte bytes[6];
 memset(bytes,0,6);
 
 i2c_write(ADXL345_ADDRESS, ADXL_REGISTER_PWRCTL, ADXL_PWRCTL_MEASURE);
 
 i2c_read(ADXL345_ADDRESS, ADXL345_REGISTER_XLSB, 6, bytes);

 for (int i=0;i<3;++i) {
 accelerometer_data[i] = (int)bytes[2*i] + (((int)bytes[2*i + 1]) << 8);
 }
}

void init_itg3200() {
  byte data = 0;

  i2c_write(ITG3200_ADDRESS, ITG3200_REGISTER_DLPF_FS, ITG3200_FULLSCALE | ITG3200_256HZ);

  i2c_read(ITG3200_ADDRESS, ITG3200_REGISTER_DLPF_FS, 1, &data);

  Serial.println((unsigned int)data);
}

void read_itg3200() {
  byte bytes[8];
  memset(bytes,0,8);

  i2c_read(ITG3200_ADDRESS, ITG3200_REGISTER_XMSB, 8, bytes);
  for (int i=0;i<4;++i) {
  gyro_data[i] = (int)bytes[2*i + 1] + (((int)bytes[2*i]) << 8);
  }
}
