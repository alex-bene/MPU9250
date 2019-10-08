/*
MPU9250_SelfTest.cpp

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
/*--------------------------------------------- SELF-TEST IMPLEMETATION ----------------------------------------------*/
/*--------------------------------------------------------------------------------------------------------------------*/

/** Gyroscope Self Test; Check Calibration with Respect to Factory Settings.
 * @returns -- true if the sensor passed the test, false elsewise
 * @returns (through passed pointer) -- percent deviation from factory trim values for each axis
 */
bool MPU9250::GyroSelfTest(float * destination) {
	uint8_t selfTest[3] = {0, 0, 0};
	int32_t avg[3] = {0, 0, 0}, STavg[3] = {0, 0, 0};
	bool pass = 1;
	float factoryTrim[3];

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
	setSensorsDisabled(0x38);	// Enable gyro only
	delay(200); // Wait for registers to set
	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, hold interrupt pin level HIGH
	// until interrupt cleared, clear on read of INT_STATUS, and enable
	// I2C_BYPASS_EN so additional chips can join the I2C bus and all can be
	// controlled by the Arduino as master.
	writeByte(MPU6500_I2Caddr, INT_PIN_CFG, 0x22);
	writeByte(MPU6500_I2Caddr, INT_ENABLE, 0x01); // Enable data ready (bit 0) interrupt
	delay(100);
	setRateDivider(0x00); // Set SMPLRT_DIV to 0 --> sample rate to 1 kHz
	writeByte(MPU6500_I2Caddr, CONFIG, 0x02); // Set range to minimum (2g)
	writeByte(MPU6500_I2Caddr, GYRO_CONFIG, 0x00); // Set DLPF to 99Hz set Fchoice to 0b1 (use DLPF)

	// Get sum of measurements from rest response of device
	while (cnt < 200){
		if (getInterruptStatus() & 0x01){
			// Read the six raw data registers sequentially into data array
			readBytes(MPU6500_I2Caddr, GYRO_XOUT_H, 6, &rawData[0]);
			// Turn the MSB and LSB into a signed 16-bit value
			avg[0] += ((int16_t)rawData[0] << 8) | rawData[1];
			avg[1] += ((int16_t)rawData[2] << 8) | rawData[3];
			avg[2] += ((int16_t)rawData[4] << 8) | rawData[5];
			cnt++;
		}
	}
	// Get average of 200 values and store as average current readings
	for (cnt = 0; cnt < 3; cnt++)
		avg[cnt] /= 200;
	// for (cnt = 0; cnt < 3; cnt++) avg[cnt] = (avg[cnt]/200) & 0xFF; //if we want only the LSB

	// Enable self test on all three axes, set range to minimum
	writeByte(MPU6500_I2Caddr, GYRO_CONFIG, 0xE0);
	delay(25); // Delay a while to let the device stabilize (oscillations)

	// Get sum of measurements from rest response of device
	cnt = 0;
	while (cnt < 200){
		if (getInterruptStatus() & 0x01){
			// Read the six raw data registers sequentially into data array
			readBytes(MPU6500_I2Caddr, GYRO_XOUT_H, 6, &rawData[0]);
			// Turn the MSB and LSB into a signed 16-bit value
			STavg[0] += ((int16_t)rawData[0] << 8) | rawData[1];
			STavg[1] += ((int16_t)rawData[2] << 8) | rawData[3];
			STavg[2] += ((int16_t)rawData[4] << 8) | rawData[5];
			cnt++;
		}
	}
	// Get average of 200 values and store as average self-test readings
	for (cnt = 0; cnt < 3; cnt++)
		STavg[cnt] /= 200;
	// for (cnt = 0; cnt < 3; cnt++) STavg[cnt] = (STavg[cnt]/200) & 0xFF; //if we want only the LSB

	// Disable self test on all three axes, set range to minimum
	writeByte(MPU6500_I2Caddr, GYRO_CONFIG, 0x00);
	delay(25); // Delay a while to let the device stabilize (oscillations)

	for (cnt = 0; cnt < 3; cnt++){
		// Retrieve device factory Self-Test Code from USR_Reg
		// X/Y/Z-axis self-test results
		selfTest[cnt] = readByte(MPU6500_I2Caddr, SELF_TEST_X_GYRO + cnt);

		// Retrieve factory self-test value from self-test code reads
		// See ANMPU-9250A-03, MPU-9250 Accelerometer, Gyroscope and Compass Self-Test Implementation
		// FT[X/Y/Z] factory trim calculation
		factoryTrim[cnt] = 2620 * pow(1.01, selfTest[cnt] - 1.0);

		// For each axis check if passed the test and also
		// pass the change from Factory Trim of Self - Test Response to 'destination'
		float st = (float)(STavg[cnt] - avg[cnt]);
		if (factoryTrim[cnt])
			pass = pass && ((st/factoryTrim[cnt]) > 0.5);
		else
			pass = pass && (abs(st * gyroScale) >= 60); // in dps
		// TODO: TO PASS OFFSET OF EACH AXIS MUST BE BELOW 20DPS

		/** Report results as a ratio of (STR - FT)/FT; the change from Factory Trim
		 * of the Self-Test Response
		 * To get percent, must multiply by 100
		 */
		destination[cnt] = 100.0*st / factoryTrim[cnt] - 100.0;
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

	return pass;
}


/** Accelerometer Self Test; Check Calibration with Respect to Factory Settings.
 * @returns -- true if the sensor passed the test, false elsewise
 * @returns (through passed pointer) -- percent deviation from factory trim values for each axis
 */
bool MPU9250::AccelSelfTest(float * destination) {
	uint8_t selfTest[3] = {0, 0, 0};
	int32_t avg[3] = {0, 0, 0}, STavg[3] = {0, 0, 0};
	bool pass = 1;
	float factoryTrim[3];

	// Store previous values of registers to restore at the end
	const float   prAccelScale    = accelScale;
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
	setSensorsDisabled(0x07); // Enable accelerometer only
	delay(200); // Wait for registers to set
	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, hold interrupt pin level HIGH
	// until interrupt cleared, clear on read of INT_STATUS, and enable
	// I2C_BYPASS_EN so additional chips can join the I2C bus and all can be
	// controlled by the Arduino as master.
	writeByte(MPU6500_I2Caddr, INT_PIN_CFG, 0x22);
	writeByte(MPU6500_I2Caddr, INT_ENABLE, 0x01); // Enable data ready (bit 0) interrupt
	delay(100);
	setRateDivider(0x00); // Set SMPLRT_DIV to 0 --> sample rate to 1 kHz
	writeByte(MPU6500_I2Caddr, ACCEL_CONFIG, 0x00); // Set range to minimum (2g)
	writeByte(MPU6500_I2Caddr, ACCEL_CONFIG2, 0x02); // Set DLPF to 99Hz set Fchoice to 0b1 (use DLPF)

	// Get sum of measurements from rest response of device
	while (cnt < 200){
		if (getInterruptStatus() & 0x01){
			// Read the six raw data registers sequentially into data array
			readBytes(MPU6500_I2Caddr, ACCEL_XOUT_H, 6, &rawData[0]);
			// Turn the MSB and LSB into a signed 16-bit value
			avg[0] += ((int16_t)rawData[0] << 8) | rawData[1];
			avg[1] += ((int16_t)rawData[2] << 8) | rawData[3];
			avg[2] += ((int16_t)rawData[4] << 8) | rawData[5];
			cnt++;
		}
	}
	// Get average of 200 values and store as average current readings
	for (cnt = 0; cnt < 3; cnt++)
		avg[cnt] /= 200;
	// for (cnt = 0; cnt < 3; cnt++) avg[cnt] = (avg[cnt]/200) & 0xFF; //if we want only the LSB

	// Enable self test on all three axes, set range to minimum
	writeByte(MPU6500_I2Caddr, ACCEL_CONFIG, 0xE0);
	delay(25); // Delay a while to let the device stabilize (oscillations)

	// Get sum of measurements from rest response of device
	cnt = 0;
	while (cnt < 200){
		if (getInterruptStatus() & 0x01){
			// Read the six raw data registers sequentially into data array
			readBytes(MPU6500_I2Caddr, ACCEL_XOUT_H, 6, &rawData[0]);
			// Turn the MSB and LSB into a signed 16-bit value
			STavg[0] += ((int16_t)rawData[0] << 8) | rawData[1];
			STavg[1] += ((int16_t)rawData[2] << 8) | rawData[3];
			STavg[2] += ((int16_t)rawData[4] << 8) | rawData[5];
			cnt++;
		}
	}
	// Get average of 200 values and store as average self-test readings
	for (cnt = 0; cnt < 3; cnt++)
		STavg[cnt] /= 200;
	// for (cnt = 0; cnt < 3; cnt++) STavg[cnt] = (STavg[cnt]/200) & 0xFF; //if we want only the LSB

	// Disable self test on all three axes, set range to minimum
	writeByte(MPU6500_I2Caddr, ACCEL_CONFIG,  0x00);
	delay(25); // Delay a while to let the device stabilize (oscillations)

	for (cnt = 0; cnt < 3; cnt++){
		// Retrieve device factory Self-Test Code from USR_Reg
		// X/Y/Z-axis self-test results
		selfTest[cnt] = readByte(MPU6500_I2Caddr, SELF_TEST_X_ACCEL + cnt);

		// Retrieve factory self-test value from self-test code reads
		// See ANMPU-9250A-03, MPU-9250 Accelerometer, Gyroscope and Compass Self-Test Implementation
		// FT[X/Y/Z] factory trim calculation
		factoryTrim[cnt] = 2620 * pow(1.01, selfTest[cnt] - 1.0);

		// For each axis check if passed the test and also
		// pass the change from Factory Trim of Self - Test Response to 'destination'
		float st = (float)(STavg[cnt] - avg[cnt]);
		if (factoryTrim[cnt])
			pass = pass && ((st/factoryTrim[cnt]) > 0.5) && ((st/factoryTrim[cnt]) < 1.5);
		else
			pass = pass && (abs(st * accelScale) >= 0.225) && (abs(st * accelScale) <= 0.675); // in gee

		/** Report results as a ratio of (STR - FT)/FT; the change from Factory Trim
		 * of the Self-Test Response
		 * To get percent, must multiply by 100
		 */
		destination[cnt] = 100.0 * st / factoryTrim[cnt] - 100.0;
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

	return pass;
}


/** Magnetometer Self Test; Check Calibration with Respect to Factory Settings.
 * @returns -- true if the sensor passed the test, false elsewise
 */
bool MPU9250::MagSelfTest() {
	float data[3] = {0, 0, 0};
	bool pass     = 1;

	// Store previous values of registers to restore at the end
	const float   prMagScale = magScale;
	const uint8_t prCNTL     = readByte(AK8963_ADDRESS, AK8963_CNTL1);
	const uint8_t prASTC     = readByte(AK8963_ADDRESS, AK8963_ASTC);

	writeByte(AK8963_ADDRESS, AK8963_CNTL1, 0x00); // Set the Magnetometer into power down mode and 14-bit output
	writeByte(AK8963_ADDRESS, AK8963_ASTC, 0x40); // Generate magnetic field for self-test
	writeByte(AK8963_ADDRESS, AK8963_CNTL1, 0x08); // Set the Magnetometer into self-test mode mode and 14-bit output
	setMagResolution(MAG_RESOLUTION_14BITS);
	do{
		cnt = 0;
		while (!(readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01)){cnt++;}
		readMagData(data); // Read Magnetometer measurement for this magnetic field
	} while (cnt == 0); // If it didn't wait to read the ST1 it had previous measurement so repeat
	
	writeByte(AK8963_ADDRESS, AK8963_ASTC, 0x00); // No Magnetic field for test
	writeByte(AK8963_ADDRESS, AK8963_CNTL1, 0x00); // Set the Magnetometer into power down mode and 14-bit output 

	for (cnt = 0; cnt < 3; cnt++){
		data[cnt] /= 10; // magScale makes the raw measurement into milliGauss, but we want microTesla (x10)
		data[cnt] *= (float)(readByte(AK8963_ADDRESS, AK8963_ASAX+cnt)) / 256.0 + 0.5; // Sensitivity Adjustment
		if (cnt < 2)
			pass = pass && (data[cnt] <= 50) && (data[cnt] >= -50); // 14bit condition to pass (200 in 16bit)
		else
			pass = pass && (data[cnt] <= -200) && (data[cnt] >= -800); // 14bit condition to pass ([-3200, -800] in 16bit)
	}

	// Restore initial values of registers
	magScale = prMagScale;
	writeByte(AK8963_ADDRESS, AK8963_CNTL1, prCNTL);
	writeByte(AK8963_ADDRESS, AK8963_ASTC, prASTC);

	return pass;
}