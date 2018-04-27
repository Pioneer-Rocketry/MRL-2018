/**
*	SensorHub.h
*	Header file that takes in sensor readings and calculates orientations
*	Author: Grant Oberhauser
*/
#ifndef SENSORHUB_H
#define SENSORHUB_H

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

//#include <i2c_t3.h>

//#include <SparkFunLSM9DS1.h>

#include "dimensionals.h"
#include "quatops.h"
#include "kalmanfilter.h"
//#include <SPI.h> // Included for SFE_LSM9DS0 library

//#include <mpu9250.h>

// SDO_XM and SDO_G are both grounded, so our addresses are:
#define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0x6B // Would be 0x6A if SDO_G is LOW

//This interupt pins are used to obtain data only when it's ready.
//This helps with better filtering.
//They are not used currently.
#define INT1XM = 2; // INT1XM tells us when accel data is ready
#define INT2XM = 3; // INT2XM tells us when mag data is ready
#define DRDYG  = 4; // DRDYG  tells us when gyro data is ready

typedef union {
struct {
    uint8_t SBYB:1;
    uint8_t OST:1;
    uint8_t RST:1;
    uint8_t OS:3;
    uint8_t RAW:1;
    uint8_t ALT:1;
} bit;
uint8_t reg;
} ctrl_reg1;

class SensorHub
{

public: 

	static bool init();

	static void update();

	static point localToGlobal(point p);

	static point globalToLocal(point p);

  static point eulerAnglesOrientation();

	static point getAccel();

	static point getMag();

	static point getGyro();

  static float getAltitude();

  static float getSpeed();

	static quaternion filteredOrientation();

  static float getDeltaT(){return deltat;}

  static ctrl_reg1 _ctrl_reg1;

protected:


	static point accel;
	static point mag;
	static point gyro;

  static float pitot_velocity;

  static float altitude;

  static uint16_t raw_pitot_pressure;

	static quaternion orient;


	static uint16_t lastUpdate;    
	static uint16_t now;           

	static double deltat;


};

//
// Accelerometer and Gyroscope registers
#define LSM9DS1XG_ACT_THS      0x04
#define LSM9DS1XG_ACT_DUR     0x05
#define LSM9DS1XG_INT_GEN_CFG_XL    0x06
#define LSM9DS1XG_INT_GEN_THS_X_XL  0x07
#define LSM9DS1XG_INT_GEN_THS_Y_XL  0x08
#define LSM9DS1XG_INT_GEN_THS_Z_XL  0x09
#define LSM9DS1XG_INT_GEN_DUR_XL    0x0A
#define LSM9DS1XG_REFERENCE_G       0x0B
#define LSM9DS1XG_INT1_CTRL         0x0C
#define LSM9DS1XG_INT2_CTRL         0x0D
#define LSM9DS1XG_WHO_AM_I          0x0F  // should return 0x68
#define LSM9DS1XG_CTRL_REG1_G       0x10
#define LSM9DS1XG_CTRL_REG2_G       0x11
#define LSM9DS1XG_CTRL_REG3_G       0x12
#define LSM9DS1XG_ORIENT_CFG_G      0x13
#define LSM9DS1XG_INT_GEN_SRC_G     0x14
#define LSM9DS1XG_OUT_TEMP_L        0x15
#define LSM9DS1XG_OUT_TEMP_H        0x16
#define LSM9DS1XG_STATUS_REG        0x17
#define LSM9DS1XG_OUT_X_L_G         0x18
#define LSM9DS1XG_OUT_X_H_G         0x19
#define LSM9DS1XG_OUT_Y_L_G         0x1A
#define LSM9DS1XG_OUT_Y_H_G         0x1B
#define LSM9DS1XG_OUT_Z_L_G         0x1C
#define LSM9DS1XG_OUT_Z_H_G         0x1D
#define LSM9DS1XG_CTRL_REG4         0x1E
#define LSM9DS1XG_CTRL_REG5_XL      0x1F
#define LSM9DS1XG_CTRL_REG6_XL      0x20
#define LSM9DS1XG_CTRL_REG7_XL      0x21
#define LSM9DS1XG_CTRL_REG8         0x22
#define LSM9DS1XG_CTRL_REG9         0x23
#define LSM9DS1XG_CTRL_REG10        0x24
#define LSM9DS1XG_INT_GEN_SRC_XL    0x26
//#define LSM9DS1XG_STATUS_REG        0x27 // duplicate of 0x17!
#define LSM9DS1XG_OUT_X_L_XL        0x28
#define LSM9DS1XG_OUT_X_H_XL        0x29
#define LSM9DS1XG_OUT_Y_L_XL        0x2A
#define LSM9DS1XG_OUT_Y_H_XL        0x2B
#define LSM9DS1XG_OUT_Z_L_XL        0x2C
#define LSM9DS1XG_OUT_Z_H_XL        0x2D
#define LSM9DS1XG_FIFO_CTRL         0x2E
#define LSM9DS1XG_FIFO_SRC          0x2F
#define LSM9DS1XG_INT_GEN_CFG_G     0x30
#define LSM9DS1XG_INT_GEN_THS_XH_G  0x31
#define LSM9DS1XG_INT_GEN_THS_XL_G  0x32
#define LSM9DS1XG_INT_GEN_THS_YH_G  0x33
#define LSM9DS1XG_INT_GEN_THS_YL_G  0x34
#define LSM9DS1XG_INT_GEN_THS_ZH_G  0x35
#define LSM9DS1XG_INT_GEN_THS_ZL_G  0x36
#define LSM9DS1XG_INT_GEN_DUR_G     0x37
//
// Magnetometer registers
#define LSM9DS1M_OFFSET_X_REG_L_M   0x05
#define LSM9DS1M_OFFSET_X_REG_H_M   0x06
#define LSM9DS1M_OFFSET_Y_REG_L_M   0x07
#define LSM9DS1M_OFFSET_Y_REG_H_M   0x08
#define LSM9DS1M_OFFSET_Z_REG_L_M   0x09
#define LSM9DS1M_OFFSET_Z_REG_H_M   0x0A
#define LSM9DS1M_WHO_AM_I           0x0F  // should be 0x3D
#define LSM9DS1M_CTRL_REG1_M        0x20
#define LSM9DS1M_CTRL_REG2_M        0x21
#define LSM9DS1M_CTRL_REG3_M        0x22
#define LSM9DS1M_CTRL_REG4_M        0x23
#define LSM9DS1M_CTRL_REG5_M        0x24
#define LSM9DS1M_STATUS_REG_M       0x27
#define LSM9DS1M_OUT_X_L_M          0x28
#define LSM9DS1M_OUT_X_H_M          0x29
#define LSM9DS1M_OUT_Y_L_M          0x2A
#define LSM9DS1M_OUT_Y_H_M          0x2B
#define LSM9DS1M_OUT_Z_L_M          0x2C
#define LSM9DS1M_OUT_Z_H_M          0x2D
#define LSM9DS1M_INT_CFG_M          0x30
#define LSM9DS1M_INT_SRC_M          0x31
#define LSM9DS1M_INT_THS_L_M        0x32
#define LSM9DS1M_INT_THS_H_M        0x33

// Using the LSM9DS1+MS5611 Teensy 3.1 Add-On shield, ADO is set to 1 
// Seven-bit device address of accel/gyro is 110101 for ADO = 0 and 110101 for ADO = 1
#define ADO 1
#if ADO
#define LSM9DS1XG_ADDRESS 0x6B  //  Device address when ADO = 1
#define LSM9DS1M_ADDRESS  0x1E  //  Address of magnetometer
#define MS5611_ADDRESS    0x77  //  Address of altimeter
#else
#define LSM9DS1XG_ADDRESS 0x6A   //  Device address when ADO = 0
#define LSM9DS1M_ADDRESS  0x1D   //  Address of magnetometer
#define MS5611_ADDRESS    0x77   //  Address of altimeter
#endif  

#define SerialDebug true  // set to true to get Serial output for debugging

// Set initial input parameters
enum Ascale {  // set of allowable accel full scale settings
  AFS_2G = 0,
  AFS_16G,
  AFS_4G,
  AFS_8G
};

enum Aodr {  // set of allowable gyro sample rates
  AODR_PowerDown = 0,
  AODR_10Hz,
  AODR_50Hz,
  AODR_119Hz,
  AODR_238Hz,
  AODR_476Hz,
  AODR_952Hz
};

enum Abw {  // set of allowable accewl bandwidths
   ABW_408Hz = 0,
   ABW_211Hz,
   ABW_105Hz,
   ABW_50Hz
};

enum Gscale {  // set of allowable gyro full scale settings
  GFS_245DPS = 0,
  GFS_500DPS,
  GFS_NoOp,
  GFS_2000DPS
};

enum Godr {  // set of allowable gyro sample rates
  GODR_PowerDown = 0,
  GODR_14_9Hz,
  GODR_59_5Hz,
  GODR_119Hz,
  GODR_238Hz,
  GODR_476Hz,
  GODR_952Hz
};

enum Gbw {   // set of allowable gyro data bandwidths
  GBW_low = 0,  // 14 Hz at Godr = 238 Hz,  33 Hz at Godr = 952 Hz
  GBW_med,      // 29 Hz at Godr = 238 Hz,  40 Hz at Godr = 952 Hz
  GBW_high,     // 63 Hz at Godr = 238 Hz,  58 Hz at Godr = 952 Hz
  GBW_highest   // 78 Hz at Godr = 238 Hz, 100 Hz at Godr = 952 Hz
};

enum Mscale {  // set of allowable mag full scale settings
  MFS_4G = 0,
  MFS_8G,
  MFS_12G,
  MFS_16G
};

enum Mmode {
  MMode_LowPower = 0, 
  MMode_MedPerformance,
  MMode_HighPerformance,
  MMode_UltraHighPerformance
};

enum Modr {  // set of allowable mag sample rates
  MODR_0_625Hz = 0,
  MODR_1_25Hz,
  MODR_2_5Hz,
  MODR_5Hz,
  MODR_10Hz,
  MODR_20Hz,
  MODR_80Hz
};

#define ADC_256  0x00 // define pressure and temperature conversion rates
#define ADC_512  0x02
#define ADC_1024 0x04
#define ADC_2048 0x06
#define ADC_4096 0x08
#define ADC_D1   0x40
#define ADC_D2   0x50

enum {
    MPL3115A2_REGISTER_STATUS        =       (0x00),

    MPL3115A2_REGISTER_PRESSURE_MSB     =    (0x01),
    MPL3115A2_REGISTER_PRESSURE_CSB    =     (0x02),
    MPL3115A2_REGISTER_PRESSURE_LSB    =     (0x03),

    MPL3115A2_REGISTER_TEMP_MSB       =      (0x04),
    MPL3115A2_REGISTER_TEMP_LSB      =       (0x05),

    MPL3115A2_REGISTER_DR_STATUS     =       (0x06),

    MPL3115A2_OUT_P_DELTA_MSB        =       (0x07),
    MPL3115A2_OUT_P_DELTA_CSB        =       (0x08),
    MPL3115A2_OUT_P_DELTA_LSB        =       (0x09),

    MPL3115A2_OUT_T_DELTA_MSB        =       (0x0A),
    MPL3115A2_OUT_T_DELTA_LSB        =       (0x0B),

    MPL3115A2_WHOAMI            =            (0x0C),

    MPL3115A2_BAR_IN_MSB         =           (0x14),
    MPL3115A2_BAR_IN_LSB         =           (0x15),
};

/**************************************************************************/
/*!
    @brief  MPL3115A2 status register bits
*/
/**************************************************************************/
enum {
    MPL3115A2_REGISTER_STATUS_TDR       = 0x02,
    MPL3115A2_REGISTER_STATUS_PDR       = 0x04,
    MPL3115A2_REGISTER_STATUS_PTDR      = 0x08,
};


/**************************************************************************/
/*!
    @brief  MPL3115A2 PT DATA register bits
*/
/**************************************************************************/
enum {
    MPL3115A2_PT_DATA_CFG       = 0x13,
    MPL3115A2_PT_DATA_CFG_TDEFE     = 0x01, 
    MPL3115A2_PT_DATA_CFG_PDEFE     = 0x02,
    MPL3115A2_PT_DATA_CFG_DREM      = 0x04,
};

/**************************************************************************/
/*!
    @brief  MPL3115A2 control registers
*/
/**************************************************************************/
enum {

    MPL3115A2_CTRL_REG1     =    (0x26),
    MPL3115A2_CTRL_REG2         =   (0x27),
    MPL3115A2_CTRL_REG3         =   (0x28),
    MPL3115A2_CTRL_REG4         =   (0x29),
    MPL3115A2_CTRL_REG5         =    (0x2A),
};

/**************************************************************************/
/*!
    @brief  MPL3115A2 control register bits
*/
/**************************************************************************/
enum {
    MPL3115A2_CTRL_REG1_SBYB   = 0x01,
    MPL3115A2_CTRL_REG1_OST     = 0x02,
    MPL3115A2_CTRL_REG1_RST      = 0x04,
    MPL3115A2_CTRL_REG1_RAW     = 0x40,
    MPL3115A2_CTRL_REG1_ALT     = 0x80,
    MPL3115A2_CTRL_REG1_BAR     = 0x00,
};


/**************************************************************************/
/*!
    @brief  MPL3115A2 oversample values
*/
/**************************************************************************/
enum {
    MPL3115A2_CTRL_REG1_OS1     = 0x00,
    MPL3115A2_CTRL_REG1_OS2     = 0x08,
    MPL3115A2_CTRL_REG1_OS4     = 0x10,
    MPL3115A2_CTRL_REG1_OS8     = 0x18,
    MPL3115A2_CTRL_REG1_OS16        = 0x20,
    MPL3115A2_CTRL_REG1_OS32        = 0x28,
    MPL3115A2_CTRL_REG1_OS64        = 0x30,
    MPL3115A2_CTRL_REG1_OS128       = 0x38,
};

#endif
