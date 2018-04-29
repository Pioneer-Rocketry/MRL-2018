/**
*	sensorhub.c
*	Implementation of logic that takes in sensor readings and calculates orientations
*	Author: Grant Oberhauser
*/

#include "sensorhub.h"
//#include <SparkFunLSM9DS1.h>

#include <i2c_t3.h>


#define PITOT_PIN A14


#define LSM9DS1_M	0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG	0x6B // Would be 0x6A if SDO_AG is LOW
#define MPL3115A2_ADDRESS 0x60

#define MAG_RATE    80   //80Hz
#define ACCEL_RATE  119  //119Hz
#define GYRO_RATE   238  //238Hz

#define MAX_PASCALS 24485.86
#define PITOT_MIN_VOLTS 0.25
#define PITOT_MAX_VOLTS 3.3
#define AIR_DENSITY 1.225


uint8_t OSR = ADC_4096;      // set pressure amd temperature oversample rate
uint8_t Gscale = GFS_2000DPS; // gyro full scale
uint8_t Godr = GODR_952Hz;   // gyro data sample rate
uint8_t Gbw = GBW_med;       // gyro data bandwidth
uint8_t Ascale = AFS_4G;     // accel full scale
uint8_t Aodr = AODR_952Hz;   // accel data sample rate
uint8_t Abw = ABW_50Hz;      // accel data bandwidth
uint8_t Mscale = MFS_4G;     // mag full scale
uint8_t Modr = MODR_80Hz;    // mag data sample rate
uint8_t Mmode = MMode_HighPerformance;  // magnetometer operation mode
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

uint16_t Pcal[8];         // calibration constants from MS5611 PROM registers
unsigned char nCRC;       // calculated check sum to ensure PROM integrity
uint32_t D1 = 0, D2 = 0;  // raw MS5611 pressure and temperature data
double dT, OFFSET, SENS, T2, OFFSET2, SENS2;  // First order and second order corrections for raw S5637 temperature and pressure data
int16_t accelCount[3], gyroCount[3], magCount[3];  // Stores the 16-bit signed accelerometer, gyro, and mag sensor output
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0},  magBias[3] = {0, 0, 0}; // Bias corrections for gyro, accelerometer, and magnetometer
int16_t tempCount;            // temperature raw count output
float   temperature;          // Stores the LSM9DS1gyro internal chip temperature in degrees Celsius
double Temperature, Pressure; // stores MS5611 pressures sensor pressure and temperature

point SensorHub::accel;
point SensorHub::mag;
point SensorHub::gyro;

float SensorHub::altitude;

uint16_t SensorHub::raw_pitot_pressure;

quaternion SensorHub::orient;

//MPU6050 mpu;

uint16_t SensorHub::lastUpdate = 0;    
uint16_t SensorHub::now = 0;           

double SensorHub::deltat = 0.0;

ctrl_reg1 SensorHub::_ctrl_reg1;

float SensorHub::pitot_velocity = 0.0f;

unsigned long last_baro_read = 0;

void accelgyrocalLSM9DS1(float * dest1, float * dest2);

void readMagData(int16_t * destination);

void readAccelData(int16_t * destination);

void readGyroData(int16_t * destination);

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data; // `data` will store the register data   
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
//  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
//  Wire.requestFrom(address, 1);  // Read one byte from slave register address 
  Wire.requestFrom(address, (size_t) 1);   // Read one byte from slave register address 
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  Wire.write(subAddress);            // Put slave register address in Tx buffer
  Wire.endTransmission(I2C_NOSTOP);  // Send the Tx buffer, but send a restart to keep connection alive
//  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
//        Wire.requestFrom(address, count);  // Read bytes from slave register address 
        Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address 
  while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

void selftestLSM9DS1()
{
  float accel_noST[3] = {0., 0., 0.}, accel_ST[3] = {0., 0., 0.};
  float gyro_noST[3] = {0., 0., 0.}, gyro_ST[3] = {0., 0., 0.};

  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG10,   0x00); // disable self test
  accelgyrocalLSM9DS1(gyro_noST, accel_noST);
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG10,   0x05); // enable gyro/accel self test
  accelgyrocalLSM9DS1(gyro_ST, accel_ST);

  float gyrodx = (gyro_ST[0] - gyro_noST[0]);
  float gyrody = (gyro_ST[1] - gyro_noST[1]);
  float gyrodz = (gyro_ST[2] - gyro_noST[2]);

  Serial.println("Gyro self-test results: ");
  Serial.print("x-axis = "); Serial.print(gyrodx); Serial.print(" dps"); Serial.println(" should be between 20 and 250 dps");
  Serial.print("y-axis = "); Serial.print(gyrody); Serial.print(" dps"); Serial.println(" should be between 20 and 250 dps");
  Serial.print("z-axis = "); Serial.print(gyrodz); Serial.print(" dps"); Serial.println(" should be between 20 and 250 dps");

  float accdx = 1000.*(accel_ST[0] - accel_noST[0]);
  float accdy = 1000.*(accel_ST[1] - accel_noST[1]);
  float accdz = 1000.*(accel_ST[2] - accel_noST[2]);

  Serial.println("Accelerometer self-test results: ");
  Serial.print("x-axis = "); Serial.print(accdx); Serial.print(" mg"); Serial.println(" should be between 60 and 1700 mg");
  Serial.print("y-axis = "); Serial.print(accdy); Serial.print(" mg"); Serial.println(" should be between 60 and 1700 mg");
  Serial.print("z-axis = "); Serial.print(accdz); Serial.print(" mg"); Serial.println(" should be between 60 and 1700 mg");
  
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG10,   0x00); // disable self test
  delay(200);
}

void accelgyrocalLSM9DS1(float * dest1, float * dest2)
{  
  uint8_t data[6] = {0, 0, 0, 0, 0, 0};
  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  uint16_t samples, ii;

   // enable the 3-axes of the gyroscope
   writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG4, 0x38);
   // configure the gyroscope
   writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG1_G, Godr << 5 | Gscale << 3 | Gbw);
   delay(200);
   // enable the three axes of the accelerometer 
   writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG5_XL, 0x38);
   // configure the accelerometer-specify bandwidth selection with Abw
   writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG6_XL, Aodr << 5 | Ascale << 3 | 0x04 |Abw);
   delay(200);
   // enable block data update, allow auto-increment during multiple byte read
   writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG8, 0x44);
  
  // First get gyro bias
  byte c = readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9);
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9, c | 0x02);     // Enable gyro FIFO  
  delay(50);                                                       // Wait for change to take effect
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_FIFO_CTRL, 0x20 | 0x1F);  // Enable gyro FIFO stream mode and set watermark at 32 samples
  delay(1000);  // delay 1000 milliseconds to collect FIFO samples
  
  samples = (readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_FIFO_SRC) & 0x2F); // Read number of stored samples

  for(ii = 0; ii < samples ; ii++) {            // Read the gyro data stored in the FIFO
    int16_t gyro_temp[3] = {0, 0, 0};
    readBytes(LSM9DS1XG_ADDRESS, LSM9DS1XG_OUT_X_L_G, 6, &data[0]);
    gyro_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]); // Form signed 16-bit integer for each sample in FIFO
    gyro_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]);
    gyro_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]);

    gyro_bias[0] += (int32_t) gyro_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    gyro_bias[1] += (int32_t) gyro_temp[1]; 
    gyro_bias[2] += (int32_t) gyro_temp[2]; 
  }  

  gyro_bias[0] /= samples; // average the data
  gyro_bias[1] /= samples; 
  gyro_bias[2] /= samples; 
  
  dest1[0] = (float)gyro_bias[0]*gRes;  // Properly scale the data to get deg/s
  dest1[1] = (float)gyro_bias[1]*gRes;
  dest1[2] = (float)gyro_bias[2]*gRes;
  
  c = readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9);
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9, c & ~0x02);   //Disable gyro FIFO  
  delay(50);
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_FIFO_CTRL, 0x00);  // Enable gyro bypass mode
 
  // now get the accelerometer bias
  c = readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9);
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9, c | 0x02);     // Enable accel FIFO  
  delay(50);                                                       // Wait for change to take effect
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_FIFO_CTRL, 0x20 | 0x1F);  // Enable accel FIFO stream mode and set watermark at 32 samples
  delay(1000);  // delay 1000 milliseconds to collect FIFO samples
  
  samples = (readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_FIFO_SRC) & 0x2F); // Read number of stored samples

  for(ii = 0; ii < samples ; ii++) {            // Read the accel data stored in the FIFO
    int16_t accel_temp[3] = {0, 0, 0};
    readBytes(LSM9DS1XG_ADDRESS, LSM9DS1XG_OUT_X_L_XL, 6, &data[0]);
    accel_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]); // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]);
    accel_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]);

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1]; 
    accel_bias[2] += (int32_t) accel_temp[2]; 
  }  

  accel_bias[0] /= samples; // average the data
  accel_bias[1] /= samples; 
  accel_bias[2] /= samples; 
  
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) (1.0/aRes);}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) (1.0/aRes);}
  
  dest2[0] = (float)accel_bias[0]*aRes;  // Properly scale the data to get g
  dest2[1] = (float)accel_bias[1]*aRes;
  dest2[2] = (float)accel_bias[2]*aRes;
  
  c = readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9);
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9, c & ~0x02);   //Disable accel FIFO  
  delay(50);
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_FIFO_CTRL, 0x00);  // Enable accel bypass mode
}

void initLSM9DS1()
{  
   // enable the 3-axes of the gyroscope
   writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG4, 0x38);
   // configure the gyroscope
   writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG1_G, Godr << 5 | Gscale << 3 | Gbw);
   delay(200);
   // enable the three axes of the accelerometer 
   writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG5_XL, 0x38);
   // configure the accelerometer-specify bandwidth selection with Abw
   writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG6_XL, Aodr << 5 | Ascale << 3 | 0x04 |Abw);
   delay(200);
   // enable block data update, allow auto-increment during multiple byte read
   writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG8, 0x44);
   // configure the magnetometer-enable temperature compensation of mag data
   writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG1_M, 0x80 | Mmode << 5 | Modr << 2); // select x,y-axis mode
   writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG2_M, Mscale << 5 ); // select mag full scale
   writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG3_M, 0x00 ); // continuous conversion mode
   writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG4_M, Mmode << 2 ); // select z-axis mode
   writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG5_M, 0x40 ); // select block update mode
}

void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(LSM9DS1XG_ADDRESS, LSM9DS1XG_OUT_X_L_XL, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
}


void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(LSM9DS1XG_ADDRESS, LSM9DS1XG_OUT_X_L_G, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
}

void readMagData(int16_t * destination)
{
    uint8_t rawData[6];  // x/y/z gyro register data stored here
    readBytes(LSM9DS1M_ADDRESS, LSM9DS1M_OUT_X_L_M, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
    destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn tFhe MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
    destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
}

void setSeaPressure(float pascal) {
  uint16_t bar = pascal/2;

  Wire.beginTransmission(MPL3115A2_ADDRESS);
  Wire.write((uint8_t)MPL3115A2_BAR_IN_MSB);
  Wire.write((uint8_t)(bar>>8));
  Wire.write((uint8_t)bar);
  Wire.endTransmission(false);
}

float get_Altitude_MPL3115A2() {
  int32_t alt;

  while(readByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1) & MPL3115A2_CTRL_REG1_OST);

/*  SensorHub::_ctrl_reg1.bit.ALT = 1;
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, SensorHub::_ctrl_reg1.reg);

  SensorHub::_ctrl_reg1.bit.OST = 1;
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, SensorHub::_ctrl_reg1.reg);*/

/*  uint8_t sta = 0;
  while (! (sta & MPL3115A2_REGISTER_STATUS_PDR)) {
    sta = readByte(MPL3115A2_ADDRESS, MPL3115A2_REGISTER_STATUS);
  }*/

  Wire.beginTransmission(MPL3115A2_ADDRESS); // start transmission to device 
  Wire.write(MPL3115A2_REGISTER_PRESSURE_MSB); 
  Wire.endTransmission(false); // end transmission
  

  Wire.requestFrom((uint8_t)MPL3115A2_ADDRESS, (uint8_t)3);// send data n-bytes read
  alt  = ((uint32_t)Wire.read()) << 24; // receive DATA
  alt |= ((uint32_t)Wire.read()) << 16; // receive DATA
  alt |= ((uint32_t)Wire.read()) << 8; // receive DATA

  float altitude = alt;
  altitude /= 65536.0;
  return altitude;
}


int pitot_min_reading = (int)((float)PITOT_MIN_VOLTS * 1024.0f/3.3f);

int pitot_max_reading = (int)((float)PITOT_MAX_VOLTS * 1024.0f/3.3f); 

int pitot_reading_0 = 0;

int pitot_reading_1 = 0;

int pitot_reading_2 = 0;

int pitot_reading_3 = 0;

int pitot_reading_4 = 0;

//Initialize sensors and sensor readings.
bool SensorHub::init()
{

  bool success = true;

  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400);

  Serial.println("Sensorhub Init!");


  orient.a = 1.0f;
  orient.b = 0.0f;
  orient.c = 0.0f;
  orient.d = 0.0f;

  Serial.println("Reading from 9DoF sensor...");

  byte c = readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_WHO_AM_I);
  byte d = readByte(LSM9DS1M_ADDRESS, LSM9DS1M_WHO_AM_I);

  Serial.printf("XG Address: %02x", c);
  Serial.printf("M Address: %02x", c);

  //Make sure we are actually connected.
  if (c == 0x68 && d == 0x3D)
  {

       // get sensor resolutions, only need to do this once
    switch (Ascale)
    {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 16 Gs (01), 4 Gs (10), and 8 Gs  (11). 
      case AFS_2G:
            aRes = 2.0/32768.0;
            break;
      case AFS_16G:
            aRes = 16.0/32768.0;
            break;
      case AFS_4G:
            aRes = 4.0/32768.0;
            break;
      case AFS_8G:
            aRes = 8.0/32768.0;
            break;
    }

    switch (Gscale)
    {
     // Possible gyro scales (and their register bit settings) are:
    // 245 DPS (00), 500 DPS (01), and 2000 DPS  (11). 
      case GFS_245DPS:
            gRes = 245.0/32768.0;
            break;
      case GFS_500DPS:
            gRes = 500.0/32768.0;
            break;
      case GFS_2000DPS:
            gRes = 2000.0/32768.0;
            break;
    }

    switch (Mscale)
    {
     // Possible magnetometer scales (and their register bit settings) are:
    // 4 Gauss (00), 8 Gauss (01), 12 Gauss (10) and 16 Gauss (11)
      case MFS_4G:
            mRes = 4.0/32768.0;
            break;
      case MFS_8G:
            mRes = 8.0/32768.0;
            break;
      case MFS_12G:
            mRes = 12.0/32768.0;
            break;
      case MFS_16G:
            mRes = 16.0/32768.0;
            break;
    }
    

    Serial.println("Perform gyro and accel self test");
    selftestLSM9DS1(); // check function of gyro and accelerometer via self test
    
    Serial.println(" Calibrate gyro and accel");
    accelgyrocalLSM9DS1(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
    Serial.println("accel biases (mg)"); Serial.println(1000.*accelBias[0]); Serial.println(1000.*accelBias[1]); Serial.println(1000.*accelBias[2]);
    Serial.println("gyro biases (dps)"); Serial.println(gyroBias[0]); Serial.println(gyroBias[1]); Serial.println(gyroBias[2]);

    //Don't calibrate magnetometer. TODO: Add biases we have calculated previously.
//    magcalLSM9DS1(magBias);
//    Serial.println("mag biases (mG)"); Serial.println(1000.*magBias[0]); Serial.println(1000.*magBias[1]); Serial.println(1000.*magBias[2]); 
//    delay(2000); // add delay to see results before serial spew of data
    
    initLSM9DS1(); 
    Serial.println("LSM9DS1 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature

    //Init altimeter

/*    writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, MPL3115A2_CTRL_REG1_RST);
    delay(10);
  
    while(readByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1) & MPL3115A2_CTRL_REG1_RST) delay(10);
  
    SensorHub::_ctrl_reg1.reg = MPL3115A2_CTRL_REG1_OS2 | MPL3115A2_CTRL_REG1_ALT | MPL3115A2_CTRL_REG1_SBYB;
  
    writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, SensorHub::_ctrl_reg1.reg);

    writeByte(MPL3115A2_ADDRESS, MPL3115A2_PT_DATA_CFG,  
     MPL3115A2_PT_DATA_CFG_TDEFE |
     MPL3115A2_PT_DATA_CFG_PDEFE |
     MPL3115A2_PT_DATA_CFG_DREM);
  
    writeByte(MPL3115A2_ADDRESS, MPL3115A2_PT_DATA_CFG, 
     MPL3115A2_PT_DATA_CFG_TDEFE |
     MPL3115A2_PT_DATA_CFG_PDEFE |
     MPL3115A2_PT_DATA_CFG_DREM);*/

    
  }
  else
  {
    success = false; 
  }
  

  return success;  
  

}

boolean accelUpdated = false, gyroUpdated = false, magUpdated = false;

long lastGyroUpdate = 0;
long lastAccelUpdate = 0;
long lastMagUpdate = 0;

long updateMillisGyro  = 1000/GYRO_RATE;
long updateMillisAccel = 1000/ACCEL_RATE;
long updateMillisMag   = 1000/MAG_RATE;

//Update sensors and filterss
void SensorHub::update()
{

  
  
  if (readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_STATUS_REG) & 0x01) {  // check if new accel data is ready  
    readAccelData(accelCount);  // Read the x/y/z adc values

 
    // Now we'll calculate the accleration value into actual g's
    accel.x = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
    accel.y = (float)accelCount[1]*aRes - accelBias[1];   
    accel.z = (float)accelCount[2]*aRes - accelBias[2];
    accelUpdated = true; 
  } 
   
  if (readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_STATUS_REG) & 0x02) {  // check if new gyro data is ready  
    readGyroData(gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    gyro.x = (float)gyroCount[0]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
    gyro.y = (float)gyroCount[1]*gRes - gyroBias[1];  
    gyro.z = (float)gyroCount[2]*gRes - gyroBias[2];   
    
    gyroUpdated = true;
  }
  
  if (readByte(LSM9DS1M_ADDRESS, LSM9DS1M_STATUS_REG_M) & 0x08) {  // check if new mag data is ready  
    readMagData(magCount);  // Read the x/y/z adc values
    
    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
    //The accel and gyro on the board aren't aligned, doing -x aligns them.
    mag.x = -(float)magCount[0]*mRes; // - magBias[0];  // get actual magnetometer value, this depends on scale being set
    mag.y = (float)magCount[1]*mRes; // - magBias[1];  
    mag.z = (float)magCount[2]*mRes; // - magBias[2];   
    magUpdated = true;
  }

  //TODO: Use readByte instead.
  if(!(readByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1) & MPL3115A2_CTRL_REG1_OST))
  {
    altitude = get_Altitude_MPL3115A2();
    last_baro_read = millis();
  }



  float reading_to_pascal = MAX_PASCALS / (pitot_max_reading - pitot_min_reading);

  int cur_reading = analogRead(PITOT_PIN);

  pitot_reading_4 = pitot_reading_3;

  pitot_reading_3 = pitot_reading_2;

  pitot_reading_2 = pitot_reading_1;

  pitot_reading_1 = pitot_reading_0;

  pitot_reading_0 = cur_reading;

  int smoothed_reading = (pitot_reading_0 + 2 * pitot_reading_1 + 3 * pitot_reading_2 + 2 * pitot_reading_3 + pitot_reading_4)/9;

  double pascals = (smoothed_reading - pitot_min_reading) * reading_to_pascal;

  if(pascals < 0)
    pascals = 0;

  pitot_velocity = sqrt((2 *pascals)/AIR_DENSITY);

  //Transforming into the vectors we need.

  //Nothing for X.

  //Make the new y, -z.
  float tempY_gyro = gyro.y;
  float tempY_accel = accel.y;
  float tempY_mag = mag.y;

  gyro.y = -gyro.z;
  accel.y = -accel.z;
  mag.y = -mag.z;

  //Make the new z, y
  gyro.z = tempY_gyro;
  accel.z = tempY_accel;
  mag.z = tempY_mag;
  
  //The madgwick function works in radians. So the gyro readings need to be converted quick. 
  point radGyro;
  radGyro.x = gyro.x*PI/180.0f;
  radGyro.y = gyro.y*PI/180.0f;
  radGyro.z = gyro.z*PI/180.0f;
  
  if(magUpdated || accelUpdated || gyroUpdated)
  {
    now = micros();
    deltat = (now - lastUpdate) / 1000000.0f;
    lastUpdate = now;
          
    orient = KalmanFilter::MadgwickQuaternionUpdate(accel, radGyro, mag, orient, deltat);
         
    magUpdated = false;
    accelUpdated = false;
    gyroUpdated = false;
  
  }



}

quaternion SensorHub::filteredOrientation()
{
	return orient;
}

point SensorHub::localToGlobal(point p)
{
	quaternion pointQ;

	pointQ.a = 0;
	pointQ.b = p.x;
	pointQ.c = p.y;
	pointQ.d = p.z;

	quaternion tempPoint = QuatOps::hProd(QuatOps::hProd(orient, pointQ),QuatOps::conj(orient));

	point finalPoint;

	finalPoint.x = tempPoint.b;
	finalPoint.y = tempPoint.c;
	finalPoint.z = tempPoint.d;

	return finalPoint;


}

point SensorHub::globalToLocal(point p)
{
	quaternion pointQ;

	pointQ.a = 0;
	pointQ.b = p.x;
	pointQ.c = p.y;
	pointQ.d = p.z;

	quaternion tempPoint = QuatOps::hProd(QuatOps::hProd(QuatOps::conj(orient), pointQ),orient);

	point finalPoint;

	finalPoint.x = tempPoint.b;
	finalPoint.y = tempPoint.c;
	finalPoint.z = tempPoint.d; 

	return finalPoint;

}

point SensorHub::eulerAnglesOrientation()
{

  point result;

  // roll (x-axis rotation)
  double sinr = +2.0 * (orient.a * orient.b + orient.c * orient.d);
  double cosr = +1.0 - 2.0 * (orient.b * orient.b + orient.c * orient.c);
  result.x = atan2(sinr, cosr);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (orient.a * orient.c - orient.d * orient.b);
  if (fabs(sinp) >= 1)
    result.y = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    result.y = asin(sinp);

  // yaw (z-axis rotation)
  double siny = +2.0 * (orient.a * orient.d + orient.b * orient.c);
  double cosy = +1.0 - 2.0 * (orient.c * orient.c + orient.d * orient.d);  
  result.z = atan2(siny, cosy);

  return result;
  
}

float SensorHub::getSpeed()
{

  return pitot_velocity;
  
}

point SensorHub::getAccel()
{
  return accel;
}

point SensorHub::getMag()
{
  return mag;
}

point SensorHub::getGyro()
{
  return gyro;
}


float SensorHub::getAltitude()
{
  return altitude;
}


