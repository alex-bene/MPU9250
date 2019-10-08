/*
MPU9250_Calibrate.cpp

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
/*---------------------------------------------------- CALIBRATE -----------------------------------------------------*/
/*--------------------------------------------------------------------------------------------------------------------*/

/** Function which accumulates accel data after device initialization. It calculates the average of the at-rest readings 
 * and then, offset of them from the ideal at-rest readings (acceleration vector magnitude should be 1 g). Then adds the
 * resulting offset (bias) to the factory-set offset in the corresponding bias registers.
 */
void MPU9250::calibrateAccel(float *accelBiasOut /*= NULL*/) {
	int16_t accelRawData[3] = {0, 0, 0};
	int32_t accelBias[3] = {0, 0, 0};
	float accelMag = 16383.0;

	// Store previous values of registers to restore at the end
	const float   prAccelScale   = accelScale;
	const uint8_t prRate         = getRateDivider();
	const uint8_t prPwrMngmt     = getPowerManagement();
	const uint8_t prSensorsDis   = getSensorsDisabled();
	const uint8_t prConfig       = readByte(MPU6500_I2Caddr, CONFIG);
	const uint8_t prAccelConfig  = readByte(MPU6500_I2Caddr, ACCEL_CONFIG);
	const uint8_t prAccelConfig2 = readByte(MPU6500_I2Caddr, ACCEL_CONFIG2);
	const uint8_t prIntPinCFG    = readByte(MPU6500_I2Caddr, INT_PIN_CFG);
	const uint8_t prIntEnable    = readByte(MPU6500_I2Caddr, INT_ENABLE);

	// Get stable time source
	// Auto select clock source to be PLL gyroscope reference if ready else use internal 20MHz oscillator
	// Clear sleep mode bit (6)
	setPowerManagement(0x01);
	delay(200);	// Wait for all registers to reset
	// Enable all sensors
	setSensorsDisabled(0x00);
	delay(200);	// Wait for all registers to reset

	// Disable FSYNC and set FIFO mode to FIFO_FULL_REPLACE_OLD
	writeByte(MPU6500_I2Caddr, CONFIG, 0x00);

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, hold interrupt pin level HIGH
	// until interrupt cleared, clear on read of INT_STATUS, and enable
	// I2C_BYPASS_EN so additional chips can join the I2C bus and all can be
	// controlled by the Arduino as master.
	writeByte(MPU6500_I2Caddr, INT_PIN_CFG, 0x22); // 0x02??
	// Enable data ready (bit 0) interrupt
	writeByte(MPU6500_I2Caddr, INT_ENABLE, 0x01);
	delay(100);

	// Set sample rate to 1kHz
	setRateDivider(0x00);
	setFullScaleAccelRange(ACCEL_FULL_SCALE_2G);
	setAccelDLPFmode(ACCEL_DLPF_BANDWIDTH_218HZ); //this is probably the 188hz bandwidth
	setAccelDLPFenable(true);
	readBytes(MPU6500_I2Caddr, ACCEL_XOUT_H, 6, &rawData[0]);
	for (uint16_t ii = 0; ii < 1024;){
		if (getInterruptStatus() & 0x01){
			// Read the six raw data registers sequentially into data array
			readBytes(MPU6500_I2Caddr, ACCEL_XOUT_H, 6, &rawData[0]);
			// Turn the MSB and LSB into a signed 16-bit value and sum individual signed 16-bit biases to get 
			// accumulated signed 32-bit biases.
			accelBias[0] += ((int16_t)rawData[0] << 8) | rawData[1];
			accelBias[1] += ((int16_t)rawData[2] << 8) | rawData[3];
			accelBias[2] += ((int16_t)rawData[4] << 8) | rawData[5];
			ii++;
		}
	}
	// Divide by 1024 to get average
	accelBias[0] = accelBias[0] >> 10;
	accelBias[1] = accelBias[1] >> 10;
	accelBias[2] = accelBias[2] >> 10;
	// Calculate average vector magnitude
	accelMag = sqrt((float)accelBias[0]*accelBias[0]
	              + (float)accelBias[1]*accelBias[1]
	              + (float)accelBias[2]*accelBias[2]);
	// Calculate the bias for each for each axis
	accelBias[0] = round(accelBias[0] * (1 - (16383.0/accelMag)));
	accelBias[1] = round(accelBias[1] * (1 - (16383.0/accelMag)));
	accelBias[2] = round(accelBias[2] * (1 - (16383.0/accelMag)));

	// A place to hold the factory accelerometer trim biases
	int16_t accelBiasReg[3] = {0, 0, 0};
	// Define array to hold mask bit for each accelerometer bias axis
	uint8_t maskBit[3] = {0, 0, 0};
	for (cnt = 0; cnt < 3; cnt++) {
		// if a destination pointer has been provided then assign to it the values of each axis average bias in dps
		if (accelBiasOut != NULL)
			accelBiasOut[cnt] = (float)(accelBias[cnt])*accelScale;
		// Read factory accelerometer trim values
		accelBiasReg[cnt] = readMPU6500doubleByte(XA_OFFSET_H + (3*cnt));
		// If temperature compensation bit is set, record that fact in mask_bit
		maskBit[cnt] = (uint8_t)(accelBiasReg[cnt] & 0x01);
		// Construct total accelerometer bias, including calculated average accelerometer bias from above
		// Subtract calculated averaged accelerometer bias scaled to 16 g full scale 2048 LSB/g
		accelBiasReg[cnt] -= (int16_t)(accelBias[cnt]/8);
		// preserve temperature compensation bit when writing back to accelerometer bias registers
		accelBiasReg[cnt] = accelBiasReg[cnt] & ~0x01; // Zero Mask bits --- 0x01 is our mask for the first bit
		accelBiasReg[cnt] = accelBiasReg[cnt] | (int16_t)maskBit[cnt]; // Set Mask value
		// Push accelerometer biases to hardware registers
		writeMPU6500doubleByte(XA_OFFSET_H + (3*cnt), accelBiasReg[cnt]);
	}

	// Restore initial values of registers
	accelScale = prAccelScale;
	setRateDivider(prRate);
	writeByte(MPU6500_I2Caddr, CONFIG, prConfig);
	writeByte(MPU6500_I2Caddr, ACCEL_CONFIG, prAccelConfig);
	writeByte(MPU6500_I2Caddr, ACCEL_CONFIG2, prAccelConfig2);
	writeByte(MPU6500_I2Caddr, INT_PIN_CFG, prIntPinCFG);
	writeByte(MPU6500_I2Caddr, INT_ENABLE, prIntEnable);
	setSensorsDisabled(prSensorsDis);
	setPowerManagement(prPwrMngmt);
	delay(200);
}

/** Function which accumulates gyro data after device initialization. It calculates the average of the at-rest readings 
 * and then loads the resulting offsets into the gyro bias registers.
 */
void MPU9250::calibrateGyro(float *gyroBiasOut /*= NULL*/) {
	int32_t gyroBias[3] = {0, 0, 0};

	// Store previous values of registers to restore at the end
	const float   prGyroScale  = gyroScale;
	const uint8_t prRate       = getRateDivider();
	const uint8_t prPwrMngmt   = getPowerManagement();
	const uint8_t prSensorsDis = getSensorsDisabled();
	const uint8_t prConfig     = readByte(MPU6500_I2Caddr, CONFIG);
	const uint8_t prGyroConfig = readByte(MPU6500_I2Caddr, GYRO_CONFIG);
	const uint8_t prIntPinCFG  = readByte(MPU6500_I2Caddr, INT_PIN_CFG);
	const uint8_t prIntEnable  = readByte(MPU6500_I2Caddr, INT_ENABLE);

	// Get stable time source
	// Auto select clock source to be PLL gyroscope reference if ready else use internal 20MHz oscillator
	// Clear sleep mode bit (6)
	setPowerManagement(0x01);
	delay(200);	// Wait for all registers to reset
	// Enable all sensors
	setSensorsDisabled(0x00);
	delay(200);	// Wait for all registers to reset

	// Disable FSYNC and set FIFO mode to FIFO_FULL_REPLACE_OLD
	writeByte(MPU6500_I2Caddr, CONFIG, 0x00);

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared, clear on read of
	// INT_STATUS, and enable I2C_BYPASS_EN so additional chips can join the I2C bus and all can be controlled by the
	// Arduino as master.
	writeByte(MPU6500_I2Caddr, INT_PIN_CFG, 0x22); // 0x02??
	// Enable data ready (bit 0) interrupt
	writeByte(MPU6500_I2Caddr, INT_ENABLE, 0x01);
	delay(100);

	// Set sample rate to 1kHz
	setRateDivider(0x00);
	setFullScaleGyroRange(GYRO_FULL_SCALE_250DPS);
	setGyroDLPFmode(GYRO_DLPF_BANDWIDTH_184HZ);
	setGyroDLPFenable(true);

	for (uint16_t ii = 0; ii < 1024;){
		if (getInterruptStatus() & 0x01){
			// Read the six raw data registers sequentially into data array
			readBytes(MPU6500_I2Caddr, GYRO_XOUT_H, 6, &rawData[0]);
			// Turn the MSB and LSB into a signed 16-bit value and sum individual signed 16-bit biases to get 
			// accumulated signed 32-bit biases.
			gyroBias[0] += (int32_t)(((int16_t)rawData[0] << 8) | rawData[1]);
			gyroBias[1] += (int32_t)(((int16_t)rawData[2] << 8) | rawData[3]);
			gyroBias[2] += (int32_t)(((int16_t)rawData[4] << 8) | rawData[5]);
			ii++;
		}
	}
	// Divide by 1024 to get average and divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format.
	// Also, Biases are additive, so change sign on calculated average gyro biases
	for (cnt = 0; cnt < 3; cnt++){
		// if a destination pointer has been provided then assign to it the values of each axis average bias in dps
		if (gyroBiasOut != NULL)
			gyroBiasOut[cnt] = (float)((gyroBias[cnt] >> 10) * gyroScale);
		gyroBias[cnt]  = -(gyroBias[cnt] >> 12);
		// Push accelerometer biases to hardware registers
		writeMPU6500doubleByte(XG_OFFSET_H + (2*cnt), gyroBias[cnt]);
	}
	
	// Restore initial values of registers
	gyroScale = prGyroScale;
	setRateDivider(prRate);
	writeByte(MPU6500_I2Caddr, CONFIG, prConfig);
	writeByte(MPU6500_I2Caddr, GYRO_CONFIG, prGyroConfig);
	writeByte(MPU6500_I2Caddr, INT_PIN_CFG, prIntPinCFG);
	writeByte(MPU6500_I2Caddr, INT_ENABLE, prIntEnable);
	setSensorsDisabled(prSensorsDis);
	setPowerManagement(prPwrMngmt);
	delay(200);
}

/** Function which accumulates magnetometer data after device initialization. 
 * It calculates the bias and scale in the x, y, and z axes.
 */
void MPU9250::calibrateMag(float * mag_bias, float * mag_scale, int16_t * mag_bias_raw /*= NULL*/) {
	int16_t mag_max[3], mag_min[3];
	float avg_scale = 0;

	// Store previous values of registers to restore at the end
	const uint8_t prMode = getMagMode();
	const uint8_t prRes  = getMagResolution();

	setMagMode(MAG_SAMPLE_RATE_100HZ);
	setMagResolution(MAG_RESOLUTION_16BITS);

	Serial.println(F("Mag Calibration: Wave device in an 8 figure until done!"));
	Serial.println(F("4 seconds to get ready followed by ~25 seconds of sampling)"));
	for (cnt = 4; cnt > 0; cnt--) {
		Serial.print(cnt)
		Serial.println(F("..........."));
		delay(1000);
	}
	Serial.println(F("GO!!"));
	
	for (uint16_t jj = 0; jj < 2000; jj++) {
		readRawMagData(tempRegister); // Read the mag data
		for (cnt = 0; cnt < 3; cnt++) {
			if ((tempRegister[cnt] > mag_max[cnt]) || (jj == 0))
				mag_max[cnt] = tempRegister[cnt];
			if ((tempRegister[cnt] < mag_min[cnt]) || (jj == 0))
				mag_min[cnt] = tempRegister[cnt];
		}
		delay(12); // At 100 Hz ODR, new mag data is available every 10 ms
	}

	// Get hard iron correction
	// Get x/y/z mag bias and save in raw
	if (mag_bias_raw != NULL)
		for (cnt = 0; cnt < 3; cnt++)
			mag_bias_raw[cnt] = (mag_max[cnt] + mag_min[cnt]) << 1;
	// Get x/y/z mag bias and save in mG
	if (mag_bias != NULL)
		for (cnt = 0; cnt < 3; cnt++)
			mag_bias[cnt] = (((float)(mag_max[cnt] + mag_min[cnt])) * magScale[cnt]) / 2.0;

	// Get soft iron correction estimate
	// Get x/y/z mag scale
	if (mag_scale != NULL) {
		for (cnt = 0; cnt < 3; cnt++) {
			mag_scale[cnt] = (mag_max[cnt] - mag_min[cnt]) / 2.0;
			avg_scale += mag_scale[cnt];
		}
		avg_scale /= 3.0;
		for (cnt = 0; cnt < 3; cnt++)
			mag_scale[cnt] = avg_scale / mag_scale[cnt];
	}

	// Restore initial values of registers
	setMagMode(prMode);
	setMagResolution(prRes);
	Serial.println(F("Mag Calibration done!"));
}
