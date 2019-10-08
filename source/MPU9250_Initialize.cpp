/*
MPU9250_Initialize.cpp

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

/*------------------------------------------------ CLASS CONSTRUCTORS ------------------------------------------------*/

/** Class Constructor for I2C Interface
 * @param address -- Address of the MPU6500 chip
 *     MPU6500_ADDRESS_AD0 (Default)
 *     MPU6500_ADDRESS_AD1
 *     other
 * @param &wirePort -- Default = Wire
 * @param clock_frequency -- I2C protocol communication frequency (Default = 100000)
 */
MPU9250::MPU9250(const uint8_t address, TwoWire &wirePort,
                 const uint32_t clock_frequency) {
	MPU6500_I2Caddr = address;
	_wire = &wirePort;
	_spi = NULL;

	_interfaceSpeed = clock_frequency;

	_csPin = NOT_SPI;	// Used to tell the library that the sensor is using I2C

	_wire->begin();
	_wire->setClock(_interfaceSpeed);
}

/** Class Constructor for SPI Interface
 * @param csPin -- Chip select pin 
 * @param &spiInterface -- Default = SPI
 * @param spi_freq -- SPI protocol communication frequency (Default = SPI_DATA_RATE = 1000000)
 * @see AK8963_ADDRESS
 * @see USER_CTRL
 * @see I2C_SLV4_ADDR
 * @see I2C_SLV4_CTRL
 * @see I2C_MST_CTRL
 */
MPU9250::MPU9250(int8_t csPin, SPIClass &spiInterface, const uint32_t spi_freq) {
	_csPin = csPin;
	_spi = &spiInterface;
	_wire = NULL;

	_interfaceSpeed = spi_freq;

	_spi->begin();
	pinMode(_csPin, OUTPUT);
	deselect();

	//kickHardware
	// Write a null byte w/o CS assertion to get SPI hardware to idle high (mode 3)
	_spi->beginTransaction(SPISettings(SPI_DATA_RATE, MSBFIRST, SPI_MODE));
	_spi->transfer(0x00);	// Send null byte
	_spi->endTransaction();

	// Enable I2C master mode
	// Enable the I2C Master I/F module; pins ES_DA and ES_SCL are isolated from pins SDA/SDI and SCL/ SCLK.
	writeMaskedByte(MPU6500_I2Caddr, USER_CTRL, 0x20, 0x20);

	// Use slave number 4 for talking to the magnetometer
	// Set the SLV_4_ADDR register to the magnetometer's address
	writeByteSPI(I2C_SLV4_ADDR, ((1 << 7) | AK8963_ADDRESS));
	// Setup SLV_4 control as needed (but not set to do an operation yet)
	writeByteSPI(I2C_SLV4_CTRL, 0x00);
	// Enable the multi-master mode and set I2C master clock to 400 kHz //0b10000000 //0b00001101
	writeByteSPI(I2C_MST_CTRL, 0b10001101);
}

/*----------------------------------------------- CHIPS INITIALIZATION -----------------------------------------------*/

/** Initialize the MPU6500 chip.
 * @param gyroFullScaleRange -- defines the maximum measurements range for the Gyroscope in degrees/second
 *     GYRO_FULL_SCALE_250DPS (Default)
 *     GYRO_FULL_SCALE_500DPS
 *     GYRO_FULL_SCALE_1000DPS
 *     GYRO_FULL_SCALE_2000DPS
 * @param accelFullScaleRange -- defines the maximum measurements range for the Accelerometer in g
 *     ACCEL_FULL_SCALE_2G (Default)
 *     ACCEL_FULL_SCALE_4G
 *     ACCEL_FULL_SCALE_8G
 *     ACCEL_FULL_SCALE_16G
 * @param gyroDLPFbandwidth -- defines Gyroscope Digital Low-Pass Filter bandwidth
 *     GYRO_DLPF_BANDWIDTH_250HZ (Default)
 *     GYRO_DLPF_BANDWIDTH_184HZ
 *     GYRO_DLPF_BANDWIDTH_92HZ
 *     GYRO_DLPF_BANDWIDTH_41HZ
 *     GYRO_DLPF_BANDWIDTH_20HZ
 *     GYRO_DLPF_BANDWIDTH_10HZ
 *     GYRO_DLPF_BANDWIDTH_5HZ
 *     GYRO_DLPF_BANDWIDTH_3600HZ
 * @param accelDLPFbandwidth -- defines Accelerometer Digital Low-Pass Filter bandwidth
 *     ACCEL_DLPF_BANDWIDTH_218HZ (Default)
 *     ACCEL_DLPF_BANDWIDTH_99HZ
 *     ACCEL_DLPF_BANDWIDTH_45HZ
 *     ACCEL_DLPF_BANDWIDTH_21HZ
 *     ACCEL_DLPF_BANDWIDTH_10HZ
 *     ACCEL_DLPF_BANDWIDTH_5HZ
 *     ACCEL_DLPF_BANDWIDTH_420HZ
 * @param sampleRateDivider -- defines the value of the SMPLRT_DIV (Default = 4)
 *         Sample_Rate = Internal_Sample_Rate / (1 + SMPLRT_DIV)
 *     for Internal_Sample_Rate by default is set to the same Rate as the Gyroscope.
 *     @see setClockSource()
 * @param gyroEnableDLPF -- defines if the Gyroscope DLPF will be enabled (enabled by Default)
 * @param gyroDisabledDLPFfchoice -- defines the FCHOICE paramter if the Gyroscope DLPF is disabled (Default = 00)
 *     |---------+---------------------------------------------------+-----------------------------|
 *     |         |          |               Gyroscope                |     Temperature Sensor      |
 *     | FCHOICE | DLPF_CFG | Bandwidth (Hz) | Delay (ms) | Fs (kHz) | Bandwidth (Hz) | Delay (ms) |
 *     |---------+----------+----------------+------------+----------+----------------+------------|
 *     | x  | 0  | x        | 8800           | 0.064      | 32       | 4000           | 0.04       |
 *     | 0  | 1  | x        | 3600           | 0.11       | 32       | 4000           | 0.04       |
 *     |---------+----------+----------------+------------+----------+----------------+------------|
 * @param accelEnableDLPF -- defines if the Accelerometer DLPF will be enabled (enabled by Default)
 * @see setPowerManagement()
 * @see setSensorsDisabled()
 * @see setRateDivider()
 * @see setFullScaleAccelRange()
 * @see setFullScaleGyroRange()
 * @see setAccelDLPFmode()
 * @see setGyroDLPFmode()
 * @see setAccelDLPFenable()
 * @see setGyroDLPFenable()
 * @see CONFIG
 * @see INT_PIN_CFG
 * @see INT_ENABLE
 * @see MPU6500_ADDRESS_AD*
 */
void MPU9250::initMPU6500(const uint8_t gyroFullScaleRange, const uint8_t accelFullScaleRange,
						  const uint8_t gyroDLPFbandwidth,const uint8_t accelDLPFbandwidth,
						  const uint8_t sampleRateDivider, const bool gyroEnableDLPF,
						  const uint8_t gyroDisabledDLPFfchoise, const bool accelEnableDLPF){
	// Reset the internal registers and restores the default settings. Βit auto clears
	setPowerManagement(0x80);
	delay(200); // Wait for all registers to reset

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

	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV) --> only if gyro_out is 1khz
	setRateDivider(sampleRateDivider);
	setFullScaleAccelRange(accelFullScaleRange);
	setFullScaleGyroRange(gyroFullScaleRange);
	setAccelDLPFmode(accelDLPFbandwidth);
	setGyroDLPFmode(gyroDLPFbandwidth);
	setAccelDLPFenable(accelEnableDLPF);
	setGyroDLPFenable(gyroEnableDLPF, gyroDisabledDLPFfchoise);

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, hold interrupt pin level HIGH
	// until interrupt cleared, clear on read of INT_STATUS, and enable
	// I2C_BYPASS_EN so additional chips can join the I2C bus and all can be
	// controlled by the Arduino as master.
	writeByte(MPU6500_I2Caddr, INT_PIN_CFG, 0x22); // 0x02??
	// Enable data ready (bit 0) interrupt
	writeByte(MPU6500_I2Caddr, INT_ENABLE, 0x01);
	delay(100);
}

/** Initialize the AK8963 chip.
 * @param magResolution -- defines the resolution for the Magnetometer
 *     MAG_RESOLUTION_16BITS (Default)
 *     MAG_RESOLUTION_14BITS
 * @param magSampleRate -- defines Magnetometer Sample Rate
 *     MAG_SAMPLE_RATE_100HZ (Default)
 *     MAG_SAMPLE_RATE_8HZ
 * @see AK8963_CNTL2
 * @see AK8963_CNTL1
 * @see AK8963_ADDRESS
 */
void MPU9250::initAK8963(const uint8_t magResolution, const uint8_t magSampleRate){
	// Reset the internal registers. Βit auto clears
	writeByte(AK8963_ADDRESS, AK8963_CNTL2, 0x01);
	// Set magnetometer data resolution and sample ODR
	setMagResolution(magResolution);
	setMagMode(magSampleRate);
	delay(50);
}

/** Initialize the MPU9250 chip.
 * @param gyroFullScaleRange -- defines the maximum measurements range for the Gyroscope in degrees/second
 *     GYRO_FULL_SCALE_250DPS (Default)
 *     GYRO_FULL_SCALE_500DPS
 *     GYRO_FULL_SCALE_1000DPS
 *     GYRO_FULL_SCALE_2000DPS
 * @param accelFullScaleRange -- defines the maximum measurements range for the Accelerometer in g
 *     ACCEL_FULL_SCALE_2G (Default)
 *     ACCEL_FULL_SCALE_4G
 *     ACCEL_FULL_SCALE_8G
 *     ACCEL_FULL_SCALE_16G
 * @param magResolution -- defines the resolution for the Magnetometer
 *     MAG_RESOLUTION_16BITS (Default)
 *     MAG_RESOLUTION_14BITS
 * @param magSampleRate -- defines Magnetometer Sample Rate
 *     MAG_SAMPLE_RATE_100HZ (Default)
 *     MAG_SAMPLE_RATE_8HZ
 * @param gyroDLPFbandwidth -- defines Gyroscope Digital Low-Pass Filter bandwidth
 *     GYRO_DLPF_BANDWIDTH_250HZ (Default)
 *     GYRO_DLPF_BANDWIDTH_184HZ
 *     GYRO_DLPF_BANDWIDTH_92HZ
 *     GYRO_DLPF_BANDWIDTH_41HZ
 *     GYRO_DLPF_BANDWIDTH_20HZ
 *     GYRO_DLPF_BANDWIDTH_10HZ
 *     GYRO_DLPF_BANDWIDTH_5HZ
 *     GYRO_DLPF_BANDWIDTH_3600HZ
 * @param accelDLPFbandwidth -- defines Accelerometer Digital Low-Pass Filter bandwidth
 *     ACCEL_DLPF_BANDWIDTH_218HZ (Default)
 *     ACCEL_DLPF_BANDWIDTH_99HZ
 *     ACCEL_DLPF_BANDWIDTH_45HZ
 *     ACCEL_DLPF_BANDWIDTH_21HZ
 *     ACCEL_DLPF_BANDWIDTH_10HZ
 *     ACCEL_DLPF_BANDWIDTH_5HZ
 *     ACCEL_DLPF_BANDWIDTH_420HZ
 * @param sampleRateDivider -- defines the value of the SMPLRT_DIV (Default = 4)
 *         Sample_Rate = Internal_Sample_Rate / (1 + SMPLRT_DIV)
 *     for Internal_Sample_Rate by default is set to the same Rate as the Gyroscope.
 *     @see setClockSource()
 * @param gyroEnableDLPF -- defines if the Gyroscope DLPF will be enabled (enabled by Default)
 * @param gyroDisabledDLPFfchoice -- defines the FCHOICE paramter if the Gyroscope DLPF is disabled (Default = 00)
 *     |---------+---------------------------------------------------+-----------------------------|
 *     |         |          |                Gyroscope               |      Temperature Sensor     |
 *     | FCHOICE | DLPF_CFG | Bandwidth (Hz) | Delay (ms) | Fs (kHz) | Bandwidth (Hz) | Delay (ms) |
 *     |---------+----------+----------------+------------+----------+----------------+------------|
 *     | x  | 0  | x        | 8800           | 0.064      | 32       | 4000           | 0.04       |
 *     | 0  | 1  | x        | 3600           | 0.11       | 32       | 4000           | 0.04       |
 *     |---------+----------+----------------+------------+----------+----------------+------------|
 * @param accelEnableDLPF -- defines if the Accelerometer DLPF will be enabled (enabled by Default)
 * @see initMPU6500()
 * @see initAK8963()
 */
void MPU9250::initMPU9250(const uint8_t gyroFullScaleRange, const uint8_t accelFullScaleRange,
						  const uint8_t magResolution, const uint8_t magSampleRate,
						  const uint8_t gyroDLPFbandwidth, const uint8_t accelDLPFbandwidth,
						  const uint8_t sampleRateDivider, const bool gyroEnableDLPF,
						  const uint8_t gyroDisabledDLPFfchoise, const bool accelEnableDLPF){
	initMPU6500(gyroFullScaleRange, accelFullScaleRange, gyroDLPFbandwidth, accelDLPFbandwidth, sampleRateDivider,
	            gyroEnableDLPF, gyroDisabledDLPFfchoise, accelEnableDLPF);
	initAK8963(magResolution, magSampleRate);
}