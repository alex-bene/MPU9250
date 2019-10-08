/*
MPU9250_OffsetConfiguration.cpp

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
/*----------------------------------------------- OFFSET CONFIGURATION -----------------------------------------------*/
/*--------------------------------------------------------------------------------------------------------------------*/

/*----------------------------------------------- ACCELEROMETER OFFSET -----------------------------------------------*/
/* Get accelerometer X/Y/Z Offset.
//* These registers are used to remove DC bias from the gyro sensor data output for X, Y and Z axes.
//* The values in these registers are subtracted from the gyro sensor values before going into the
//* sensor registers. Please refer to registers 67 to 72 for units.

 * *_OFFS_USR[14:7]	| Upper bits of the X accelerometer offset cancellation. +/- 16g Offset
                    | cancellation in all Full Scale modes, 15 bit 0.98-mg steps
 * -----------------+-----------------------------------------------------------
 * *_OFFS_USR[6:0]  | Lower bits of the X accelerometer offset cancellation. +/- 16g Offset
 *                  | cancellation in all Full Scale modes, 15 bit 0.98-mg steps
 * 
 * @return (through passed pointer) -- XA_OFFSET, YA_OFFSET, ZA_OFFSET
 * @see MPU6500_ADDRESS_AD*
 * @see XG_OFFSET_H
 */
void MPU9250::getAccelOffset(int16_t * destination) {
	uint8_t rawOffset[6];  // x/y/z accel offset register data stored here
	// Read the six raw data registers into data array
	readBytes(MPU6500_I2Caddr, XA_OFFSET_H, 6, &rawOffset[0]);

	destination[0] = ((int16_t)rawOffset[0] << 7) | rawOffset[1] >> 1;
	destination[1] = ((int16_t)rawOffset[2] << 7) | rawOffset[3] >> 1;
	destination[2] = ((int16_t)rawOffset[4] << 7) | rawOffset[5] >> 1;
}
/* Set accelerometer X/Y/Z Offset.
 * @see getAccelOffset()
 * @see XA_OFFSET_H, YA_OFFSET_H, ZA_OFFSET_H
 */
void MPU9250::setAccelOffset(int16_t * offset) {
	// one swift left because the lsb of the *A_OFFSET_L is reserved
	writeMPU6500doubleByte(XA_OFFSET_H, offset[0] << 1);
	writeMPU6500doubleByte(YA_OFFSET_H, offset[1] << 1);
	writeMPU6500doubleByte(ZA_OFFSET_H, offset[2] << 1);
}

/*--------------------------------------------------- GYRO OFFSET ----------------------------------------------------*/
/* Get gyroscope X/Y/Z Offset.
 * These registers are used to remove DC bias from the gyro sensor data output for X, Y and Z axes.
 * The values in these registers are subtracted from the gyro sensor values before going into the
 * sensor registers. Please refer to registers 67 to 72 for units.

 * *_OFFS_USR[15:8]	| High byte, Low byte in USR register (14h)
 *                  | OffsetLSB = *_OFFS_USR * 4 / 2^FS_SEL
 *                  | OffsetDPS = *_OFFS_USR * 4 / 2^FS_SEL / Gyro_Sensitivity
 *                  | Nominal FS_SEL = 0
 *                  | Conditions Gyro_Sensitivity = 2^16 LSB / 500dps
 *                  | Max 999.969 dps
 *                  | Min -1000 dps
 *                  | Step 0.0305 dps
 * -----------------+-----------------------------------------------------------
 * *_OFFS_USR[7:0]  | Low byte, High byte in USR register (13h)
 * 
 * @return (through passed pointer) -- XG_OFFSET, YG_OFFS, ZG_OFFS
 * @see MPU6500_ADDRESS_AD*
 * @see XG_OFFSET_H
 */
void MPU9250::getGyroOffset(int16_t * destination) {
	uint8_t rawOffset[6];  // x/y/z gyro offset register data stored here
	// Read the six raw data registers into data array
	readBytes(MPU6500_I2Caddr, XG_OFFSET_H, 6, &rawOffset[0]);

	destination[0] = ((int16_t)rawOffset[0] << 8) | rawOffset[1];
	destination[1] = ((int16_t)rawOffset[2] << 8) | rawOffset[3];
	destination[2] = ((int16_t)rawOffset[4] << 8) | rawOffset[5];
}
/* Set gyroscope X/Y/Z Offset.
 * @see getGyroOffset()
 * @see XG_OFFSET_H, YG_OFFSET_H, ZG_OFFSET_H
 */
void MPU9250::setGyroOffset(int16_t * offset) {
	writeMPU6500doubleByte(XG_OFFSET_H, offset[0]);
	writeMPU6500doubleByte(YG_OFFSET_H, offset[1]);
	writeMPU6500doubleByte(ZG_OFFSET_H, offset[2]);
}