/*
MPU9250_SensorsRead.cpp

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

#include "MPU9250.h"

/*--------------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------- READ RAW DATA ---------------------------------------------------*/
/*--------------------------------------------------------------------------------------------------------------------*/

/** Reads the Raw Data from the Accelerometer Registers
 * @returns (through passed pointer) -- ACCEL_X_RAW, ACCEL_Y_RAW, ACCEl_Z_RAW
 * @see MPU6500_ADDRESS_AD*
 * @see ACCEL_*OUT_*
 */
void MPU9250::readRawAccelData(int16_t * destination) {
	// Read the six raw data registers into data array
	readBytes(MPU6500_I2Caddr, ACCEL_XOUT_H, 6, &rawData[0]);

	// Turn the MSB and LSB into a signed 16-bit value
	destination[0] = ((int16_t)rawData[0] << 8) | rawData[1];
	destination[1] = ((int16_t)rawData[2] << 8) | rawData[3];
	destination[2] = ((int16_t)rawData[4] << 8) | rawData[5];
}

/** Reads the Raw Data from the Gyroscope Registers
 * @returns (through passed pointer) -- GYRO_X_RAW, GYRO_Y_RAW, GYRO_Z_RAW
 * @see MPU6500_ADDRESS_AD*
 * @see GYRO_*OUT_*
 */
void MPU9250::readRawGyroData(int16_t * destination) {
	// Read the six raw data registers sequentially into data array
	readBytes(MPU6500_I2Caddr, GYRO_XOUT_H, 6, &rawData[0]);

	// Turn the MSB and LSB into a signed 16-bit value
	destination[0] = ((int16_t)rawData[0] << 8) | rawData[1];
	destination[1] = ((int16_t)rawData[2] << 8) | rawData[3];
	destination[2] = ((int16_t)rawData[4] << 8) | rawData[5];
}

/** Reads the Raw Data from the Temperature Registers
 * @returns (through passed pointer) -- TEMP_RAW
 * @see TEMP_OUT_*
 */
void MPU9250::readRawTempData(int16_t * destination) {
	readBytes(MPU6500_I2Caddr, TEMP_OUT_H, 2, &rawData[0]);
	*destination = ((int16_t)rawData[0] << 8) | rawData[1];
}

/** Reads the Raw Data from the Magnetometer Registers
 * @returns -- false if magnetic sensor overflow has occured, true elsewise
 * @returns (through passed pointer) -- MAG_X_RAW, MAG_Y_RAW, MAG_Z_RAW
 * @see AK8963_ST1
 * @see AK8963_ST2
 * @see AK8963_*OUT_*
 */
bool MPU9250::readRawMagData(int16_t * destination) {
	// x/y/z gyro register data, ST2 register stored here, must read ST2 at end
	// of data acquisition
	
	/*// Wait for magnetometer data ready bit to be set
	if (readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) {
		// Read the six raw data and ST2 registers sequentially into data array
		readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);

		// Check if magnetic sensor overflow set, if not then report data
		if (!(rawData[6] & 0x08)){ //read ST2 register
			// Turn the MSB and LSB into a signed 16-bit value
			destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];
			destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];
			destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
			return 1;
		} else return 0;
	}*/
	// Read the six raw data and ST2 registers sequentially into data array
	readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);

	// Check if magnetic sensor overflow set, if not then report data
	if (!(rawData[6] & 0x08)){ //read ST2 register
		// Turn the MSB and LSB into a signed 16-bit value
		destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];
		destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];
		destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
		return 1;
	} else
		return 0;
}


/*--------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------- READ DATA WITH UNITS -----------------------------------------------*/
/*--------------------------------------------------------------------------------------------------------------------*/

/** Reads the Data from the Accelerometer in g
 * 
 * ACCEL_*_G = ACCEL_*_RAW * accelScale
 * 
 * @returns (through passed pointer) -- ACCEL_X_G, ACCEl_Y_G, ACCEl_Z_G
 * @see MPU6500_ADDRESS_AD*
 * @see readRawAccelData()
 * @see getFullScaleAccelRange()
 * @see setFullScaleAccelRange()
 * @see getAccelScale()
 */
void MPU9250::readAccelData(float * destination) {
	readRawAccelData(tempRegister);
	destination[0] = ((float)(tempRegister[0])) * accelScale;
	destination[1] = ((float)(tempRegister[1])) * accelScale;
	destination[2] = ((float)(tempRegister[2])) * accelScale;
}

/** Reads the Data from the Gyroscope in degrees/second
 * 
 * GYRO_*_G = GYRO_*_RAW * gyroScale
 * 
 * @returns (through passed pointer) -- GYRO_X_DPS, GYRO_Y_DPS, GYRO_Z_DPS
 * @see MPU6500_ADDRESS_AD*
 * @see readRawGyroData()
 * @see getFullScaleGyroRange()
 * @see setFullScaleGyroRange()
 * @see getGyroScale()
 */
void MPU9250::readGyroData(float * destination) {
	readRawGyroData(tempRegister);
	destination[0] = ((float)(tempRegister[0])) * gyroScale;
	destination[1] = ((float)(tempRegister[1])) * gyroScale;
	destination[2] = ((float)(tempRegister[2])) * gyroScale;
}

/** Read Temperature in degrees Celsius
 * 
 * TEMP_degC = ((TEMP_OUT – RoomTemp_Offset)/Temp_Sensitivity) + 21degC
 * 
 * Where Temp_degC is the temperature in degrees C
 * measured by the temperature sensor. TEMP_OUT is
 * the actual output of the temperature sensor.
 */
void MPU9250::readTempData(float * destination) {
	readRawTempData(&tempTempRegister);
	*destination = ((float)(tempTempRegister)) / 333.87 + 21.0; //(data/340)+36.53??
}

/** Reads the Data from the Magnetometer in milliGauss
 * 
 * MAG_*_G = MAG_*_RAW * magScale
 * 
 * @returns -- false if magnetic sensor overflow has occured, true elsewise
 * @returns (through passed pointer) -- MAG_X_mG, MAG_Y_mG, MAG_Z_mG
 * @see MPU6500_ADDRESS_AD*
 * @see readRawMagData()
 * @see getMagResolution()
 * @see setMagResolution()
 * @see getMagScale()
 */
bool MPU9250::readMagData(float * destination) {
	if ( ! readRawMagData(tempRegister))
		return 0;
	destination[0] = ((float)(tempRegister[0])) * magScale[0];
	destination[1] = ((float)(tempRegister[1])) * magScale[1];
	destination[2] = ((float)(tempRegister[2])) * magScale[2];
	return 1;
}