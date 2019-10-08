/*
MPU9250.h

MIT License

Copyright (c) 2019 Alexandros Benetatos

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit
persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


/*
 Note: The MPU9250 is an I2C sensor and uses the Arduino Wire library.
 Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or
 a 3.3 V Teensy 3.1. We have disabled the internal pull-ups used by the Wire
 library in the Wire.h/twi.c utility file. We are also using the 400 kHz fast
 I2C mode by setting the TWI_FREQ to 400000L /twi.h utility file.
 */


#ifndef _MPU9250_H_
#define _MPU9250_H_

#include <SPI.h>
#include <Wire.h>


/*--------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------- CHIP REGISTERS ---------------------------------------------------*/
/*--------------------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------- MAGNETOMETER REGISTERS ----------------------------------------------*/

#define  WHO_AM_I_AK8963     0x00 // (AKA WIA) should return 0x48
#define  INFO                0x01
#define  AK8963_ST1          0x02 // data ready status bit 0
#define  AK8963_XOUT_L       0x03 // data
#define  AK8963_XOUT_H       0x04
#define  AK8963_YOUT_L       0x05
#define  AK8963_YOUT_H       0x06
#define  AK8963_ZOUT_L       0x07
#define  AK8963_ZOUT_H       0x08
#define  AK8963_ST2          0x09 // Data overflow bit 3 and data read error status bit 2
// Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define  AK8963_CNTL1        0x0A
#define  AK8963_CNTL2        0x0B
#define  AK8963_ASTC         0x0C // Self test control
#define  AK8963_I2CDIS       0x0F // I2C disable
#define  AK8963_ASAX         0x10 // Fuse ROM x-axis sensitivity adjustment value
#define  AK8963_ASAY         0x11 // Fuse ROM y-axis sensitivity adjustment value
#define  AK8963_ASAZ         0x12 // Fuse ROM z-axis sensitivity adjustment value

/*-------------------------------------- GYROSCOPE AND ACCELEROMETER REGISTERS ---------------------------------------*/

#define  SELF_TEST_X_GYRO    0x00
#define  SELF_TEST_Y_GYRO    0x01
#define  SELF_TEST_Z_GYRO    0x02

#define  SELF_TEST_X_ACCEL   0x0D
#define  SELF_TEST_Y_ACCEL   0x0E
#define  SELF_TEST_Z_ACCEL   0x0F

//#define  SELF_TEST_A       0x10 //??

#define  XG_OFFSET_H         0x13 // User-defined trim values for gyroscope
#define  XG_OFFSET_L         0x14
#define  YG_OFFSET_H         0x15
#define  YG_OFFSET_L         0x16
#define  ZG_OFFSET_H         0x17
#define  ZG_OFFSET_L         0x18

#define  SMPLRT_DIV          0x19
#define  CONFIG              0x1A
#define  GYRO_CONFIG         0x1B
#define  ACCEL_CONFIG        0x1C
#define  ACCEL_CONFIG2       0x1D
#define  LP_ACCEL_ODR        0x1E
#define  WOM_THR             0x1F

// Duration counter threshold for motion interrupt generation, 1 kHz rate,
// LSB = 1 ms
#define  MOT_DUR             0x20 //??
// Zero-motion detection threshold bits [7:0]
#define  ZMOT_THR            0x21 //??
// Duration counter threshold for zero motion interrupt generation, 16 Hz rate,
// LSB = 64 ms
#define  ZRMOT_DUR           0x22 //??

#define  FIFO_EN             0x23

#define  I2C_MST_CTRL        0x24
#define  I2C_SLV0_ADDR       0x25
#define  I2C_SLV0_REG        0x26
#define  I2C_SLV0_CTRL       0x27

#define  I2C_SLV1_ADDR       0x28
#define  I2C_SLV1_REG        0x29
#define  I2C_SLV1_CTRL       0x2A

#define  I2C_SLV2_ADDR       0x2B
#define  I2C_SLV2_REG        0x2C
#define  I2C_SLV2_CTRL       0x2D

#define  I2C_SLV3_ADDR       0x2E
#define  I2C_SLV3_REG        0x2F
#define  I2C_SLV3_CTRL       0x30

#define  I2C_SLV4_ADDR       0x31
#define  I2C_SLV4_REG        0x32
#define  I2C_SLV4_DO         0x33
#define  I2C_SLV4_CTRL       0x34
#define  I2C_SLV4_DI         0x35

#define  I2C_MST_STATUS      0x36

#define  INT_PIN_CFG         0x37
#define  INT_ENABLE          0x38
#define  DMP_INT_STATUS      0x39 // Check DMP interrupt  //?
#define  INT_STATUS          0x3A

#define  ACCEL_XOUT_H        0x3B
#define  ACCEL_XOUT_L        0x3C
#define  ACCEL_YOUT_H        0x3D
#define  ACCEL_YOUT_L        0x3E
#define  ACCEL_ZOUT_H        0x3F
#define  ACCEL_ZOUT_L        0x40

#define  TEMP_OUT_H          0x41
#define  TEMP_OUT_L          0x42

#define  GYRO_XOUT_H         0x43
#define  GYRO_XOUT_L         0x44
#define  GYRO_YOUT_H         0x45
#define  GYRO_YOUT_L         0x46
#define  GYRO_ZOUT_H         0x47
#define  GYRO_ZOUT_L         0x48

#define  EXT_SENS_DATA_00    0x49
#define  EXT_SENS_DATA_01    0x4A
#define  EXT_SENS_DATA_02    0x4B
#define  EXT_SENS_DATA_03    0x4C
#define  EXT_SENS_DATA_04    0x4D
#define  EXT_SENS_DATA_05    0x4E
#define  EXT_SENS_DATA_06    0x4F
#define  EXT_SENS_DATA_07    0x50
#define  EXT_SENS_DATA_08    0x51
#define  EXT_SENS_DATA_09    0x52
#define  EXT_SENS_DATA_10    0x53
#define  EXT_SENS_DATA_11    0x54
#define  EXT_SENS_DATA_12    0x55
#define  EXT_SENS_DATA_13    0x56
#define  EXT_SENS_DATA_14    0x57
#define  EXT_SENS_DATA_15    0x58
#define  EXT_SENS_DATA_16    0x59
#define  EXT_SENS_DATA_17    0x5A
#define  EXT_SENS_DATA_18    0x5B
#define  EXT_SENS_DATA_19    0x5C
#define  EXT_SENS_DATA_20    0x5D
#define  EXT_SENS_DATA_21    0x5E
#define  EXT_SENS_DATA_22    0x5F
#define  EXT_SENS_DATA_23    0x60

#define  MOT_DETECT_STATUS   0x61 //??

#define  I2C_SLV0_DO         0x63
#define  I2C_SLV1_DO         0x64
#define  I2C_SLV2_DO         0x65
#define  I2C_SLV3_DO         0x66

#define  I2C_MST_DELAY_CTRL  0x67

#define  SIGNAL_PATH_RESET   0x68

#define  MOT_DETECT_CTRL     0x69

#define  USER_CTRL           0x6A // Bit 7 enable DMP, bit 3 reset DMP
#define  PWR_MGMT_1          0x6B // Device defaults to the SLEEP mode
#define  PWR_MGMT_2          0x6C

#define  DMP_BANK            0x6D // Activates a specific bank in the DMP //??
#define  DMP_RW_PNT          0x6E // Set read/write pointer to a specific start address in specified DMP bank //??
#define  DMP_REG             0x6F // Register in DMP from which to read or to which to write //??
#define  DMP_REG_1           0x70 //??
#define  DMP_REG_2           0x71 //??

#define  FIFO_COUNTH         0x72
#define  FIFO_COUNTL         0x73
#define  FIFO_R_W            0x74

#define  WHO_AM_I_MPU6500    0x75 // Should return 0x71

#define  XA_OFFSET_H         0x77
#define  XA_OFFSET_L         0x78
#define  YA_OFFSET_H         0x7A
#define  YA_OFFSET_L         0x7B
#define  ZA_OFFSET_H         0x7D
#define  ZA_OFFSET_L         0x7E


/*--------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------ LIBRARY CONSTANTS -------------------------------------------------*/
/*--------------------------------------------------------------------------------------------------------------------*/

// Simply define MPU6500_ADDRESS as one of the two following depending on your application/IC
#define  MPU6500_ADDRESS_AD1         0x69 // Device address when ADO = 1
#define  MPU6500_ADDRESS_AD0         0x68 // Device address when ADO = 0
#define  AK8963_ADDRESS              0x0C // Address of magnetometer

/*------------------------------------------------ SPI Configuration -------------------------------------------------*/

#define  READ_FLAG                   0x80
#define  NOT_SPI                     -1
#define  SPI_DATA_RATE               1000000 // 1MHz is the max speed of the MPU-9250
#define  SPI_MODE                    SPI_MODE3

/*------------------------------------------------ DLPF CONFIGURATION ------------------------------------------------*/

#define  GYRO_DLPF_CFG_MASK          0x07
#define  GYRO_DLPF_BANDWIDTH_250HZ   0x00
#define  GYRO_DLPF_BANDWIDTH_184HZ   0x01
#define  GYRO_DLPF_BANDWIDTH_92HZ    0x02
#define  GYRO_DLPF_BANDWIDTH_41HZ    0x03
#define  GYRO_DLPF_BANDWIDTH_20HZ    0x04
#define  GYRO_DLPF_BANDWIDTH_10HZ    0x05
#define  GYRO_DLPF_BANDWIDTH_5HZ     0x06
#define  GYRO_DLPF_BANDWIDTH_3600HZ  0x07

#define  ACCEL_DLPF_CFG_MASK         0x07
//#define  ACCEL_DLPF_BANDWIDTH_218HZ  0x00
#define  ACCEL_DLPF_BANDWIDTH_218HZ  0x01 // probably 198Hz
#define  ACCEL_DLPF_BANDWIDTH_99HZ   0x02
#define  ACCEL_DLPF_BANDWIDTH_45HZ   0x03
#define  ACCEL_DLPF_BANDWIDTH_21HZ   0x04
#define  ACCEL_DLPF_BANDWIDTH_10HZ   0x05
#define  ACCEL_DLPF_BANDWIDTH_5HZ    0x06
#define  ACCEL_DLPF_BANDWIDTH_420HZ  0x07

/*------------------------------------------ FULL SCALE RANGE CONFIGURATION ------------------------------------------*/

#define  ACCEL_FULL_SCALE_SEL_MASK   0x18
#define  ACCEL_FULL_SCALE_2G         0x00
#define  ACCEL_FULL_SCALE_4G         0x08
#define  ACCEL_FULL_SCALE_8G         0x10
#define  ACCEL_FULL_SCALE_16G        0x18

#define  GYRO_FULL_SCALE_SEL_MASK    0x18
#define  GYRO_FULL_SCALE_250DPS      0x00
#define  GYRO_FULL_SCALE_500DPS      0x08
#define  GYRO_FULL_SCALE_1000DPS     0x10
#define  GYRO_FULL_SCALE_2000DPS     0x18

/*--------------------------------------------- MAGNETOMETER RESOLUTION ----------------------------------------------*/

#define  MAG_RESOLUTION_SEL_MASK     0x10
#define  MAG_RESOLUTION_14BITS       0x00 // 0.6 mG per LSB
#define  MAG_RESOLUTION_16BITS       0x10 // 0.15 mG per LSB

/*---------------------------------------------------- FIFO MODE -----------------------------------------------------*/

#define  FIFO_MODE_MASK              0x20
#define  FIFO_FULL_REPLACE_OLD       0x00
#define  FIFO_FULL_THROW_NEW         0x20

/*----------------------------------------------- FSYNC CONFIGURATION ------------------------------------------------*/

#define  FSYNC_SET_MASK              0x1C
#define  FSYNC_DISABLE               0x00
#define  FSYNC_TEMP_OUT_L            0x04
#define  FSYNC_GYRO_XOUT_L           0x08
#define  FSYNC_GYRO_YOUT_L           0x0C
#define  FSYNC_GYRO_ZOUT_L           0x10
#define  FSYNC_ACCEL_XOUT_L          0x14
#define  FSYNC_ACCEL_YOUT_L          0x18
#define  FSYNC_ACCEL_ZOUT_L          0x1C

/*--------------------------------------------------- LP_ACCEL_ODR ---------------------------------------------------*/

#define  LP_ACCEL_ODR_MASK           0x0F
#define  LP_ACCEL_ODR_0_24           0x00
#define  LP_ACCEL_ODR_0_49           0x01
#define  LP_ACCEL_ODR_0_98           0x02
#define  LP_ACCEL_ODR_1_95           0x03
#define  LP_ACCEL_ODR_3_91           0x04
#define  LP_ACCEL_ODR_7_81           0x05
#define  LP_ACCEL_ODR_15_63          0x06
#define  LP_ACCEL_ODR_31_25          0x07
#define  LP_ACCEL_ODR_62_50          0x08
#define  LP_ACCEL_ODR_125            0x09
#define  LP_ACCEL_ODR_250            0x0A
#define  LP_ACCEL_ODR_500            0x0B

/*------------------------------------------------ MAGNETOMETER MODE -------------------------------------------------*/

#define  MAG_MODE_MASK               0x0F
#define  MAG_POWER_DOWN              0x00
#define  MAG_SINGLE_MEAS             0x01
#define  MAG_SAMPLE_RATE_8HZ         0x02 // 8 Hz continuous magnetometer
#define  MAG_EXT_TRIG_MEAS           0x04
#define  MAG_SAMPLE_RATE_100HZ       0x06 // 100 Hz continuous magnetometer
#define  MAG_SELF_TEST               0x08
#define  MAG_FUSO_ROM_ACCESS         0x0F


#define  WHOAMI_DEFAULT1_MPU6500     0x71
#define  WHOAMI_DEFAULT2_MPU6500     0x73
#define  WHOAMI_DEFAULT_AK8963       0x48

#define  CLOCK_SEL_MASK              0x07


/*--------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------- CLASS IMPLEMENTATION -----------------------------------------------*/
/*--------------------------------------------------------------------------------------------------------------------*/

class MPU9250{
	public:
/*------------------------------------------------ CLASS CONSTRUCTORS ------------------------------------------------*/

		          MPU9250                       (const int8_t    csPin,
		                                         SPIClass  &spiInterface                 =  SPI,
		                                         const uint32_t  spi_freq                =  SPI_DATA_RATE);
		          MPU9250                       (const uint8_t   address                 =  MPU6500_ADDRESS_AD0,
		                                         TwoWire   &wirePort                     =  Wire,
		                                         const uint32_t  clock_frequency         =  100000);

/*------------------------------------------------- INITIALIZE CHIPS -------------------------------------------------*/

		void      initMPU6500                   (const uint8_t   gyroFullScaleRange      =  GYRO_FULL_SCALE_250DPS,
		                                         const uint8_t   accelFullScaleRange     =  ACCEL_FULL_SCALE_2G,
		                                         const uint8_t   gyroDLPFbandwidth       =  GYRO_DLPF_BANDWIDTH_41HZ,
		                                         const uint8_t   accelDLPFbandwidth      =  ACCEL_DLPF_BANDWIDTH_45HZ,
		                                         const uint8_t   sampleRateDivider       =  0x04,
		                                         const bool      gyroEnableDLPF          =  true,
		                                         const uint8_t   gyroDisabledDLPFfchoise =  0x00,
		                                         const bool      accelEnableDLPF         =  true);
		void      initAK8963                    (const uint8_t   magResolution           =  MAG_RESOLUTION_16BITS,
		                                         const uint8_t   magSampleRate           =  MAG_SAMPLE_RATE_100HZ);
		void      initMPU9250                   (const uint8_t   gyroFullScaleRange      =  GYRO_FULL_SCALE_250DPS,
		                                         const uint8_t   accelFullScaleRange     =  ACCEL_FULL_SCALE_2G,
		                                         const uint8_t   magResolution           =  MAG_RESOLUTION_16BITS,
		                                         const uint8_t   magSampleRate           =  MAG_SAMPLE_RATE_100HZ,
		                                         const uint8_t   gyroDLPFbandwidth       =  GYRO_DLPF_BANDWIDTH_250HZ,
		                                         const uint8_t   accelDLPFbandwidth      =  ACCEL_DLPF_BANDWIDTH_218HZ,
		                                         const uint8_t   sampleRateDivider       =  0x04,
		                                         const bool      gyroEnableDLPF          =  true,
		                                         const uint8_t   gyroDisabledDLPFfchoise =  0x00,
		                                         const bool      accelEnableDLPF         =  true);

/*------------------------------------------------- CONNECTION TESTS -------------------------------------------------*/

		uint8_t   MPU6500whoAmI                 (      void);
		uint8_t   AK8963whoAmI                  (      void);
		bool      testConnection                (      void);

/*------------------------------------------------- FULL SCALE RANGE -------------------------------------------------*/

		uint8_t   getFullScaleGyroRange         (      void);
		bool      setFullScaleGyroRange         (const uint8_t   range);

		uint8_t   getFullScaleAccelRange        (      void);
		bool      setFullScaleAccelRange        (const uint8_t   range);

/*------------------------------------------------ DLPF CONFIGURATION ------------------------------------------------*/

		void      setGyroDLPFenable             (const bool      enable,
		                                         const uint8_t   fchoice                 =  0x00);
		uint8_t   getGyroDLPFenable             (      void);

		void      setAccelDLPFenable            (const bool      enable);
		bool      getAccelDLPFenable            (      void);

		bool      setAccelDLPFmode              (const uint8_t   A_DLPF_CFG);
		uint8_t   getAccelDLPFmode              (      void);

		bool      setGyroDLPFmode               (const uint8_t   DLPF_CFG);
		uint8_t   getGyroDLPFmode               (      void);

/*--------------------------------------------------- CLOCK SOURCE ---------------------------------------------------*/

		uint8_t   getClockSource                (      void);
		bool      setClockSource                (const uint8_t   source);

/*--------------------------------------------------- RATE DIVIDER ---------------------------------------------------*/

		uint8_t   getRateDivider                (      void);
		void      setRateDivider                (const uint8_t   rate);

/*--------------------------------------------------- SENSOR OFFSET --------------------------------------------------*/

		void      getGyroOffset                 (      int16_t   *destination);
		void      setGyroOffset                 (      int16_t   *offset);

		void      getAccelOffset                (      int16_t   *destination);
		void      setAccelOffset                (      int16_t   *offset);

/*--------------------------------------------- SENSOR MEASUREMENT SCALE ---------------------------------------------*/

		float     getMagScale                   (      void);
		float     getGyroScale                  (      void);
		float*    getAccelScale                 (      void);

/*------------------------------------------------- READ SENSOR DATA -------------------------------------------------*/

		void      readRawAccelData              (      int16_t   *destination);
		void      readRawGyroData               (      int16_t   *destination);
		void      readRawTempData               (      int16_t   *destination);
		bool      readRawMagData                (      int16_t   *destination);

		void      readAccelData                 (      float     *destination); // in g
		void      readGyroData                  (      float     *destination); // in degrees/second
		void      readTempData                  (      float     *destination); // in degrees Celsius
		bool      readMagData                   (      float     *destination); // in milliGauss

/*--------------------------------------------- MAGNETOMETER RESOLUTION ----------------------------------------------*/

		uint8_t   getMagResolution              (      void);
		bool      setMagResolution              (const uint8_t   resolution);

/*---------------------------------------------------- SELF TEST -----------------------------------------------------*/
		/*bool    MPU6500selfTest               (      float     *destination,
		                                               bool      *pass); //is it better? */

		bool      GyroSelfTest                  (      float     *destination);
		bool      AccelSelfTest                 (      float     *destination);
		bool      MagSelfTest                   (      void);

/*--------------------------------------------------- CALIBRATION ----------------------------------------------------*/

		void      calibrateMPU9250              (      void);
		void      calibrateAccel                (      float     *accelBiasOut           = NULL);
		void      calibrateGyro                 (      float     *gyroBiasOut            = NULL);
		void      calibrateMag                  (      float     *mag_bias,
		                                               float     *mag_scale,
													   int16_t   *mag_bias_raw           = NULL);

/*------------------------------------------------------- FIFO -------------------------------------------------------*/

		uint8_t   getFIFOmode                   (      void);
		bool      setFIFOmode                   (const uint8_t   FIFO_MODE);

		uint8_t   getFIFOenable                 (      void);
		void      setFIFOenable                 (const uint8_t   FIFO_ENABLE);

/*---------------------------------------------------- INTERRUPTS ----------------------------------------------------*/

		uint8_t   getInterruptPinConfiguration  (      void);
		void      setInterruptPinConfiguration  (const uint8_t   INTERRUPT_PIN_CONFIGURATION);

		uint8_t   getInterruptEnable            (      void);
		void      setInterruptEnable            (const uint8_t   INTERRUPT_ENABLE);

		uint8_t   getInterruptStatus            (      void);


		uint8_t   getAccelLowPowerODR           (      void);
		bool      setAccelLowPowerODR           (const uint8_t   lowPowerAccelODR);

		uint8_t   getAccelWOMthreshold          (      void);
		void      setAccelWOMthreshold          (const uint8_t   WOM_Threshold);

		uint8_t   getUserControl                (      void);
		void      setUserControl                (const uint8_t   USER_CONTROL_CONFIGURATION);

		uint8_t   getPowerManagement            (      void);
		void      setPowerManagement            (const uint8_t   PWR_MNGM_CONFIG);

		uint8_t   getSensorsDisabled            (      void);
		void      setSensorsDisabled            (const uint8_t   SENSOR_ENABLE);

		uint8_t   getMagMode                    (      void);
		bool      setMagMode                    (const uint8_t   mode);

		void      getMagSensitivity             (      float     *destination);

/*------------------------------------------------- READ/WRITE BYTE --------------------------------------------------*/

		uint8_t   writeByte                     (const uint8_t   deviceAddress,
		                                         const uint8_t   registerAddress,
		                                         const uint8_t   data);

		uint8_t   readByte                      (const uint8_t   deviceAddress,
		                                         const uint8_t   registerAddress);

	private:
		uint8_t   cnt = 0;                               // universal counter
		uint8_t   rawData[7];                            // for temp/accel/gyro/mag raw readings
		int16_t   tempRegister[3];                       // for accel/gyro/mag double register readings
		int16_t   tempTempRegister;                      // for temperature
		float     magSensitivityAdj[3];
		// Scale resolutions per LSB for the sensors
		float     accelScale;
		float     gyroScale;
		float     magScale[3];                           // it incorporates magSensitivityAdjustment values

		uint8_t   MPU6500_I2Caddr = MPU6500_ADDRESS_AD0; // Use AD0 by default
		TwoWire   *_wire;                                // Allows for use of various I2C ports

		SPIClass  *_spi;                                 // Allows for use of different SPI ports
		int8_t    _csPin;                                // SPI chip select pin

		uint32_t  _interfaceSpeed;                       // Stores the desired I2C or SPi clock rate

/*------------------------------------------- MPU9250 READ/WRITE PROTOCOLS -------------------------------------------*/

		void      writeMaskedByte               (const uint8_t   deviceAddress,
		                                         const uint8_t   registerAddress,
		                                         const uint8_t   mask,
		                                         const uint8_t   value);

		uint8_t   readMaskedByte                (const uint8_t   deviceAddress,
		                                         const uint8_t   registerAddress,
		                                         const uint8_t   mask);

		void      writeMPU6500doubleByte        (const uint8_t   register_addr,
		                                         const  int16_t    data);
		int16_t   readMPU6500doubleByte         (const uint8_t   register_addr);

		void      readBytes                     (const uint8_t   deviceAddress,
		                                         const uint8_t   registerAddress,
		                                         const uint8_t   count,
		                                               uint8_t   *dest);

/*--------------------------------------------- SPI READ/WRITE PROTOCOLS ---------------------------------------------*/

		void      select                        (      void);
		void      deselect                      (      void);

		uint8_t   writeByteSPI                  (const uint8_t   registerAddress,
		                                         const uint8_t   writeData);
		uint8_t   readByteSPI                   (const uint8_t   registerAddress);

		uint8_t   writeMagByteSPI               (const uint8_t   registerAddress,
		                                         const uint8_t   data);
		uint8_t   readMagByteSPI                (const uint8_t   registerAddress);

		void      readBytesSPI                  (const uint8_t   registerAddress,
		                                         const uint8_t   count,
		                                               uint8_t   *dest);

/*--------------------------------------------- I2C READ/WRITE PROTOCOLS ---------------------------------------------*/

		uint8_t   writeByteWire                 (const uint8_t   deviceAddress,
		                                         const uint8_t   registerAddress,
		                                         const uint8_t   data);
		uint8_t   readByteWire                  (const uint8_t   deviceAddress,
		                                         const uint8_t   registerAddress);

		void      readBytesWire                 (const uint8_t   deviceAddress,
		                                         const uint8_t   registerAddress,
		                                         const uint8_t   count,
		                                               uint8_t   *dest);
};
#endif // _MPU9250_H_