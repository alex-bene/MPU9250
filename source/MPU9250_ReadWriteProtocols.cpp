/*
MPU9250_ReadWriteProtocols.cpp

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
/*------------------------------------------- MPU9250 READ/WRITE PROTOCOLS -------------------------------------------*/
/*--------------------------------------------------------------------------------------------------------------------*/

/*----------------------------------------------- ONE BYTE READ/WRITE ------------------------------------------------*/
/** Writes a byte to given register on device. Calls necessary SPI or I2C
 * implementation. This was configured in the constructor.
 */
uint8_t MPU9250::writeByte(const uint8_t deviceAddress, const uint8_t registerAddress, const uint8_t data) {
	if (_csPin != NOT_SPI) {
		if (deviceAddress == AK8963_ADDRESS)
			return writeMagByteSPI(registerAddress, data);
		else
			return writeByteSPI(registerAddress, data);
	} else
		return writeByteWire(deviceAddress,registerAddress, data);
}
/** Read a byte from given register on device. Calls necessary SPI or I2C
 * implementation. This was configured in the constructor.
 */
uint8_t MPU9250::readByte(const uint8_t deviceAddress, const uint8_t registerAddress) {
	if (_csPin != NOT_SPI){
		if (deviceAddress == AK8963_ADDRESS)
			return readMagByteSPI(registerAddress);
		else
			return readByteSPI(registerAddress);
	} else
		return readByteWire(deviceAddress, registerAddress);
}

/*---------------------------------------------- MASKED BYTE READ/WRITE ----------------------------------------------*/
void MPU9250::writeMaskedByte(const uint8_t deviceAddress, const uint8_t register_addr, const uint8_t mask,
                              const uint8_t value) {
	uint8_t masked_value = (mask & value);
	uint8_t regvalue = readByte(deviceAddress, register_addr);
	regvalue = regvalue & ~mask;	// Zero Mask bits
	regvalue = regvalue | masked_value;	//Set Mask value
	writeByte(deviceAddress, register_addr, regvalue);
}
uint8_t MPU9250::readMaskedByte(const uint8_t deviceAddress, const uint8_t register_addr, const uint8_t mask) {
	uint8_t data = readByte(deviceAddress, register_addr);
	return (data & mask);
}

/*------------------------------------------ MPU6500 DOUBLE BYTE READ/WRITE ------------------------------------------*/
void MPU9250::writeMPU6500doubleByte(const uint8_t register_addr, const int16_t data) {
	writeByte(MPU6500_I2Caddr, register_addr, (data >> 8) & 0xFF);
	writeByte(MPU6500_I2Caddr, register_addr + 1, data & 0xFF);
}
int16_t MPU9250::readMPU6500doubleByte(const uint8_t register_addr) {
	return (((int16_t)readByte(MPU6500_I2Caddr, register_addr)) << 8 | readByte(MPU6500_I2Caddr, register_addr + 1));
}

/*----------------------------------------------- MULTIPLE BYTES READ ------------------------------------------------*/
// Read 1 or more bytes from given register on device. Calls necessary SPI or I2C
// implementation. This was configured in the constructor.
void MPU9250::readBytes(const uint8_t deviceAddress, const uint8_t registerAddress, const uint8_t count, uint8_t * dest){
	if (_csPin != NOT_SPI){
		if (deviceAddress == AK8963_ADDRESS)
			for (uint8_t i = 0; i < count; i++)
				dest[i] = readMagByteSPI(registerAddress);
		else
			readBytesSPI(registerAddress, count, dest);	// Read using SPI
	} else
		readBytesWire(deviceAddress, registerAddress, count, dest);	// Read via I2C
}



/*--------------------------------------------------------------------------------------------------------------------*/
/*--------------------------------------------- SPI READ/WRITE PROTOCOLS ---------------------------------------------*/
/*--------------------------------------------------------------------------------------------------------------------*/

// Select slave IC by asserting CS pin - SPI
void MPU9250::select() {
	digitalWrite(_csPin, LOW);
}

// Select slave IC by deasserting CS pin - SPI
void MPU9250::deselect() {
	digitalWrite(_csPin, HIGH);
}

/*----------------------------------------------- ONE BYTE READ/WRITE ------------------------------------------------*/
// Writes a byte to the given register address using SPI
uint8_t MPU9250::writeByteSPI(const uint8_t registerAddress, const uint8_t writeData) {
	uint8_t returnVal;
	_spi->beginTransaction(SPISettings(_interfaceSpeed, MSBFIRST, SPI_MODE));
	select();

	_spi->transfer(registerAddress);
	returnVal = _spi->transfer(writeData);

	deselect();
	_spi->endTransaction();
	return returnVal;
}

// Read a byte from the given register address using SPI
uint8_t MPU9250::readByteSPI(const uint8_t registerAddress) {
	return writeByteSPI(registerAddress | READ_FLAG, 0xFF /*0xFF is arbitrary*/);
}

// One Byte SPI Write for Magnetometer
uint8_t MPU9250::writeMagByteSPI(const uint8_t registerAddress, const uint8_t data) {
	writeByteSPI(I2C_SLV4_REG, registerAddress);
	writeByteSPI(I2C_SLV4_DO, data);
	// Command the read into I2C_SLV4_DI register, cause an interrupt when complete
	writeByteSPI(I2C_SLV4_CTRL, 0b10000001);

	uint8_t I2C_MASTER_STATUS = readByteSPI(I2C_MST_STATUS);
	uint32_t count = 0;

	// Checks against the I2C_SLV4_DONE bit in the I2C master status register
	while (((I2C_MASTER_STATUS & 0b01000000) == 0) && (count++ < 10000))
		I2C_MASTER_STATUS = readByteSPI(I2C_MST_STATUS);
	if (count > 10000)
		Serial.println(F("Timed out"));
	return 1;
}

// One Byte SPI Read for Magnetometer
uint8_t MPU9250::readMagByteSPI(const uint8_t registerAddress) {
	writeByteSPI(I2C_SLV4_REG, registerAddress);
	// Command the read into I2C_SLV4_DI register, cause an interrupt when complete
	writeByteSPI(I2C_SLV4_CTRL, 0b10000001);

	// Wait for the data to be ready
	uint8_t I2C_MASTER_STATUS = readByteSPI(I2C_MST_STATUS);

	uint32_t count = 0;
	// Checks against the I2C_SLV4_DONE bit in the I2C master status register
	while (((I2C_MASTER_STATUS & 0b01000000) == 0) && (count++ < 100000))
		I2C_MASTER_STATUS = readByteSPI(I2C_MST_STATUS);
	if (count > 10000)
		Serial.println(F("Timed out"));
	return readByteSPI(I2C_SLV4_DI);	// Read the data that is in the SLV4_DI register 
}

/*----------------------------------------------- MULTIPLE BYTES READ ------------------------------------------------*/
// Read 1 or more bytes from given register and device using SPI
void MPU9250::readBytesSPI(const uint8_t registerAddress, const uint8_t count, uint8_t * dest) {
	_spi->beginTransaction(SPISettings(SPI_DATA_RATE, MSBFIRST, SPI_MODE));
	select();
	
	_spi->transfer(registerAddress | READ_FLAG);
	for (uint8_t i = 0; i < count; i++) dest[i] = _spi->transfer(0x00);
	_spi->endTransaction();
	
	deselect();
	delayMicroseconds(50);
}



/*--------------------------------------------------------------------------------------------------------------------*/
/*--------------------------------------------- I2C READ/WRITE PROTOCOLS ---------------------------------------------*/
/*--------------------------------------------------------------------------------------------------------------------*/

/*----------------------------------------------- ONE BYTE READ/WRITE ------------------------------------------------*/
// Write a byte to the given register address from device using I2C
uint8_t MPU9250::writeByteWire(const uint8_t deviceAddress, const uint8_t registerAddress, const uint8_t data) {
	_wire->setClock(_interfaceSpeed);	// Reset to the desired speed, in case other devices required a slowdown
	_wire->beginTransmission(deviceAddress);	// Initialize the Tx buffer
	_wire->write(registerAddress);	// Put slave register address in Tx buffer
	_wire->write(data);	// Put data in Tx buffer
	_wire->endTransmission();	// Send the Tx buffer
	return 1;
}

// Read a byte from the given register address from device using I2C
uint8_t MPU9250::readByteWire(const uint8_t deviceAddress, const uint8_t registerAddress) {
	uint8_t data;	// `data` will store the register data

	// Initialize the Tx buffer
	_wire->beginTransmission(deviceAddress);
	// Put slave register address in Tx buffer
	_wire->write(registerAddress);
	// Send the Tx buffer, but send a restart to keep connection alive
	_wire->endTransmission(false);
	// Read one byte from slave register address
	_wire->requestFrom(deviceAddress, (uint8_t) 1);
	// Fill Rx buffer with result
	data = _wire->read();
	// Return data read from slave register
	return data;
}

/*----------------------------------------------- MULTIPLE BYTES READ ------------------------------------------------*/
// Read 1 or more bytes from given register and device using I2C
void MPU9250::readBytesWire(const uint8_t deviceAddress, const uint8_t registerAddress, const uint8_t count, uint8_t * dest) {
	// Initialize the Tx buffer
	_wire->beginTransmission(deviceAddress);
	// Put slave register address in Tx buffer
	_wire->write(registerAddress);
	// Send the Tx buffer, but send a restart to keep connection alive
	_wire->endTransmission(false);

	uint8_t i = 0;
	// Read bytes from slave register address
	_wire->requestFrom(deviceAddress, count);
	while (_wire->available()) dest[i++] = _wire->read();	// Put read results in the Rx buffer
}