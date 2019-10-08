/*
MPU9250_FullScaleRange.cpp

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
/*------------------------------------------ FULL SCALE RANGE CONFIGURATION ------------------------------------------*/
/*--------------------------------------------------------------------------------------------------------------------*/

/*------------------------------------------ ACCELEROMETER FULL SCALE RANGE ------------------------------------------*/

/** Get full-scale accelerometer range.
 * The ACCEL_FULL_SCALE parameter allows setting the full-scale range of the accelerometer
 * sensors, as described below.
 *
 * +/- 2g
 * +/- 4g
 * +/- 8g
 * +/- 16g
 *
 * @return Current full-scale accelerometer range setting
 */
uint8_t MPU9250::getFullScaleAccelRange() {
    return readMaskedByte(MPU6500_I2Caddr, ACCEL_CONFIG, ACCEL_FULL_SCALE_SEL_MASK);
}

/** Set full-scale accelerometer range.
 * @param range New full-scale accelerometer range setting
 * @see getFullScaleAccelRange()
 */
bool MPU9250::setFullScaleAccelRange(const uint8_t range) {
    if ((range >> 3) > 3)
		return 0;
    else {
			// Possible accelerometer scales (and their register bit settings) are:
			// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
			accelScale = (float)(0x02 << (range>>3)) / 32768.0f;
			writeMaskedByte(MPU6500_I2Caddr, ACCEL_CONFIG, ACCEL_FULL_SCALE_SEL_MASK, range);
			return 1;
	}
}

/*---------------------------------------------- GYRO FULL SCALE RANGE -----------------------------------------------*/

/** Get full-scale gyroscope range.
 * The GYRO_FULL_SCALE parameter allows setting the full-scale range of the gyro sensors,
 * as described below.
 *
 * 250 degrees/sec
 * 500 degrees/sec
 * 1000 degrees/sec
 * 2000 degrees/sec
 *
 * @return Current full-scale gyroscope range setting
 */
uint8_t MPU9250::getFullScaleGyroRange() {
    return readMaskedByte(MPU6500_I2Caddr, GYRO_CONFIG, GYRO_FULL_SCALE_SEL_MASK);
}

/** Set full-scale gyroscope range.
 * @param range -- New full-scale gyroscope range value
 * @see getFullScaleGyroRange()
 */
bool MPU9250::setFullScaleGyroRange(const uint8_t range) {
    if ((range >> 3) > 3)
		return 0;
	else {
		// Possible gyro scales (and their register bit settings) are:
		// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS (11).
		gyroScale = (float)(0xFA << (range >> 3)) / 32768.0f;
		writeMaskedByte(MPU6500_I2Caddr, GYRO_CONFIG, GYRO_FULL_SCALE_SEL_MASK, range);
		return 1;
	}
}