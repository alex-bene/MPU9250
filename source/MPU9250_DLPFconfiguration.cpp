/*
MPU9250_DLPFconfiguration.cpp

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
/*------------------------------------------------ DLPF CONFIGURATION ------------------------------------------------*/
/*--------------------------------------------------------------------------------------------------------------------*/

/*----------------------------------------------- ENABLE/DISABLE DLPF ------------------------------------------------*/

/** Enable or Disable Accelerometer Digital Low-Pass Filter.
 * @param enable -- True to enable, false to disable
 * @see getAccelDLPFenable()
 * @see ACCEL_CONFIG2
 * @see MPU6500_ADDRESS_AD*
 */
void MPU9250::setAccelDLPFenable(const bool enable) {
	if (enable)
		writeMaskedByte(MPU6500_I2Caddr, ACCEL_CONFIG2, 0x08, 0x00);
	else
		writeMaskedByte(MPU6500_I2Caddr, ACCEL_CONFIG2, 0x08, 0x08);
}
/** Check if Accelerometer Digital Low-Pass Filter is Enabled or Disabled.
 * @returns -- True if it is enabled, false elsewise
 * @see setAccelDLPFenable()
 * @see ACCEL_CONFIG2
 * @see MPU6500_ADDRESS_AD*
 */
bool MPU9250::getAccelDLPFenable() {
	return (! readMaskedByte(MPU6500_I2Caddr, ACCEL_CONFIG2, 0x08));
}

/** Enable or Disable Gyroscope Digital Low-Pass Filter.
 * 
 * If the filter is disabled (by setting FCHOICE to anything different than 11)
 * then we have:
 * 
 * |---------+----------+----------------------------------------+-----------------------------|
 * |         |          |               Gyroscope                |      Temperature Sensor     |
 * | FCHOICE | DLPF_CFG | Bandwidth (Hz) | Delay (ms) | Fs (kHz) | Bandwidth (Hz) | Delay (ms) |
 * |---------+----------+----------------+------------+----------+----------------+------------|
 * | x  | 0  | x        | 8800           | 0.064      | 32       | 4000           | 0.04       |
 * | 0  | 1  | x        | 3600           | 0.11       | 32       | 4000           | 0.04       |
 * |---------+----------+----------------+------------+----------+----------------+------------|
 * 
 * @param enable -- True to enable, false to disable
 * @see getGyroDLPFenable()
 * @see GYRO_CONFIG
 * @see MPU6500_ADDRESS_AD*
 */
void MPU9250::setGyroDLPFenable(const bool enable, const uint8_t fchoice) {
	if (enable)
		writeMaskedByte(MPU6500_I2Caddr, GYRO_CONFIG, 0x03, 0x00);
	else
		writeMaskedByte(MPU6500_I2Caddr, GYRO_CONFIG, 0x03, ~fchoice);
}
/** Check if Gyroscope Digital Low-Pass Filter is Enabled or Disabled.
 * @returns -- FCHOICE. If equals 3 then enabled, else disabled (check setGyroDLPFenable() for more explanation)
 * @see setGyroDLPFenable()
 * @see GYRO_CONFIG
 * @see MPU6500_ADDRESS_AD*
 */
uint8_t MPU9250::getGyroDLPFenable() {
	return (~readMaskedByte(MPU6500_I2Caddr, GYRO_CONFIG, 0x03)) & 0x03;
}


/*----------------------------------------- ACCELEROMETER DLPF CONFIGURATION -----------------------------------------*/

/** Set Accelerometer Digital Low-Pass Filter Configuration Mode.
 * 
 * |------------+-------------+--------------+------------|
 * |            |    Output   |              |            |
 * | A_DLPF_CFG | 3dB BW (Hz) | Filter Block | Delay (ms) |
 * |------------+-------------+--------------+------------|
 * | 0          | 218.1       | DLPF         | 1.88       |
 * | 1          | 218.1       | DLPF         | 1.88       | //probably 198
 * | 2          | 99          | DLPF         | 2.88       |
 * | 3          | 44.8        | DLPF         | 4.88       |
 * | 4          | 21.2        | DLPF         | 8.87       |
 * | 5          | 10.2        | DLPF         | 16.83      |
 * | 6          | 5.05        | DLPF         | 32.48      |
 * | 7          | 420         | Dec2         | 1.38       |
 * |------------+-------------+--------------+------------|
 * 
 * Rate = 1 kHz
 * 
 * If the filter is disabled (by setting ACCEL_FCHOICE to 0 -- see setAccelDLPFenable())
 * then we have:
 * 
 * |------------+--------------------------+--------------+------------|
 * |            |           Output         |              |            |
 * | A_DLPF_CFG | 3dB BW (Hz) | Rate (kHz) | Filter Block | Delay (ms) |
 * |------------+-------------+------------+--------------+------------|
 * | x          | 1,046       | 4          | Dec1         | 0.503      |
 * |------------+-------------+------------+--------------+------------|
 * 
 * returns -- False if not acceptable mode, true elsewise
 * @param A_DLPF_CFG -- New accelerometer DLFP configuration mode
 * @see getAccelDLPFmode()
 * @see ACCEL_CONFIG2
 * @see ACCEL_DLPF_CFG_MASK
 * @see MPU6500_ADDRESS_AD*
 */
bool MPU9250::setAccelDLPFmode(const uint8_t A_DLPF_CFG) {
	if (A_DLPF_CFG > 7)
		return 0;
	writeMaskedByte(MPU6500_I2Caddr, ACCEL_CONFIG2, ACCEL_DLPF_CFG_MASK, A_DLPF_CFG);
	return 1;
}

/** Get Accelerometer Digital Low-Pass Filter Configuration Mode.
 * returns -- Accelerometer DLPF configuration mode
 * @see setAccelDLPFmode()
 * @see ACCEL_CONFIG2
 * @see ACCEL_DLPF_CFG_MASK
 * @see MPU6500_ADDRESS_AD*
 */
uint8_t MPU9250::getAccelDLPFmode() {
	return readMaskedByte(MPU6500_I2Caddr, ACCEL_CONFIG2, ACCEL_DLPF_CFG_MASK);
}


/*--------------------------------------------- GYRO DLPF CONFIGURATION ----------------------------------------------*/

/** Set Gyroscope Digital Low-Pass Filter Configuration Mode.
 * 
 * |----------+----------------------------------------+-----------------------------|
 * |          |               Gyroscope                |     Temperature Sensor      |
 * | DLPF_CFG | Bandwidth (Hz) | Delay (ms) | Fs (kHz) | Bandwidth (Hz) | Delay (ms) |
 * |----------+----------------+------------+----------+----------------+------------|
 * | 0        | 250            | 0.97       | 8        | 4000           | 0.04       |
 * | 1        | 184            | 2.9        | 1        | 188            | 1.9        |
 * | 2        | 92             | 3.9        | 1        | 98             | 2.8        |
 * | 3        | 41             | 5.9        | 1        | 42             | 4.8        |
 * | 4        | 20             | 9.9        | 1        | 20             | 8.3        |
 * | 5        | 10             | 17.85      | 1        | 10             | 13.4       |
 * | 6        | 5              | 33.48      | 1        | 5              | 18.6       |
 * | 7        | 3600           | 0.17       | 8        | 4000           | 0.04       |
 * |----------+----------------+------------+----------+----------------+------------|
 * 
 * If the filter is disabled (by setting FCHOICE to anything different than 11 -- see setGyroDLPFenable())
 * then we have:
 * 
 * |---------+----------+----------------------------------------+-----------------------------|
 * |         |          |                Gyroscope               |      Temperature Sensor     |
 * | FCHOICE | DLPF_CFG | Bandwidth (Hz) | Delay (ms) | Fs (kHz) | Bandwidth (Hz) | Delay (ms) |
 * |---------+----------+----------------+------------+----------+----------------+------------|
 * | x  | 0  | x        | 8800           | 0.064      | 32       | 4000           | 0.04       |
 * | 0  | 1  | x        | 3600           | 0.11       | 32       | 4000           | 0.04       |
 * |---------+----------+----------------+------------+----------+----------------+------------|
 * 
 * returns -- False if not acceptable mode, true elsewise
 * @param DLPF_CFG -- New gyroscope DLFP configuration mode
 * @see getGyroDLPFmode()
 * @see CONFIG
 * @see GYRO_DLPF_CFG_MASK
 * @see MPU6500_ADDRESS_AD*
 */
bool MPU9250::setGyroDLPFmode(const uint8_t DLPF_CFG) {
	if (DLPF_CFG > 7)
		return 0;
	writeMaskedByte(MPU6500_I2Caddr, CONFIG, GYRO_DLPF_CFG_MASK, DLPF_CFG);
	return 1;
}

/** Get digital low-pass filter configuration.
 * @returns --  Gyroscope DLPF configuration mode
 * @see setGyroDLPFmode()
 * @see CONFIG
 * @see GYRO_DLPF_CFG_MASK
 * @see MPU6500_ADDRESS_AD*
 */
uint8_t MPU9250::getGyroDLPFmode() {
	return readMaskedByte(MPU6500_I2Caddr, CONFIG, GYRO_DLPF_CFG_MASK);
}

