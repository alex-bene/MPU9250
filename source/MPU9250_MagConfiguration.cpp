/*
MPU9250_MagConfiguration.cpp

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

/*--------------------------------------------- MAGNETOMETER RESOLUTION ----------------------------------------------*/
/** Set Magnetometer Resolution.
 * @return -- False if now acceptable resolution, true elsewise
 * @param resolution -- New magnetometer resolution
 *     MAG_RESOLUTION_14BITS
 *     MAG_RESOLUTION_16BITS
 * @see getMagResolution()
 */
bool MPU9250::setMagResolution(const uint8_t resolution) {
	getMagSensitivity(magSensitivityAdj);
	if (resolution == MAG_RESOLUTION_14BITS)
		for (cnt = 0; cnt < 3; cnt++)
			magScale[cnt] = magSensitivityAdj[cnt] * 10.0f * 4912.0f / 8190.0f;
	else if (resolution == MAG_RESOLUTION_16BITS)
		for (cnt = 0; cnt < 3; cnt++)
			magScale[cnt] = magSensitivityAdj[cnt] * 10.0f * 4912.0f / 32760.0f;
	else
		return 0;
	writeMaskedByte(AK8963_ADDRESS, AK8963_CNTL1, MAG_RESOLUTION_SEL_MASK, resolution);
	return 1;
}

/** Get Magnetometer Resolution.
 * Possible magnetometer resolutions (and their register bit settings) are:
 * 14 bit resolution (0b00000) and 16 bit resolution (0b10000)
 *     MAG_RESOLUTION_14BITS
 *     MAG_RESOLUTION_16BITS
 * @return -- Magnetometer resolution
 */
uint8_t MPU9250::getMagResolution() {
	return readMaskedByte(AK8963_ADDRESS, AK8963_CNTL1, MAG_RESOLUTION_SEL_MASK);
}

/*----------------------------------------- MAGNETOMETER MODE CONFIGURATION ------------------------------------------*/
uint8_t MPU9250::getMagMode() {
	return readMaskedByte(AK8963_ADDRESS, AK8963_CNTL1, MAG_MODE_MASK);
}
bool MPU9250::setMagMode(const uint8_t mode) {
	if ((mode == 3) || (mode == 5) || (mode == 7) || ((mode > 7) && (mode != 15)))
		return 0;
	writeMaskedByte(AK8963_ADDRESS, AK8963_CNTL1, MAG_MODE_MASK, mode);
	return 1;
}


/*--------------------------------------------- MAGNETOMETER SENSITIVITY ---------------------------------------------*/
void MPU9250::getMagSensitivity(float * destination) {
	uint8_t prCNTL = readByte(AK8963_ADDRESS, AK8963_CNTL1); // Store previous byte to restore at end

	writeByte(AK8963_ADDRESS, AK8963_CNTL1, 0x00); // Power down magnetometer
	delay(50);
	writeByte(AK8963_ADDRESS, AK8963_CNTL1, 0x0F); // Enter Fuse ROM access mode
	delay(50);

	// Read the x-, y-, and z-axis calibration values
	readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);

	// Return x-axis sensitivity adjustment values, etc.
	for (cnt = 0; cnt < 3; cnt++)
		destination[cnt] =  (float)(rawData[cnt] - 128)/256. + 1.; // rawData[0]/256. + 0.5 <----------------------
	writeByte(AK8963_ADDRESS, AK8963_CNTL1, 0x00); // Power down magnetometer
	delay(50);

	writeByte(AK8963_ADDRESS, AK8963_CNTL1, prCNTL); // Restore previous byte
	delay(50);
}