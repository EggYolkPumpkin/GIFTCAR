/*
 *  @author KKest
 *		@created 10.01.2022
 *
 * Library to control an rpLidar S2
 *
 *  @author 潘骏羿
 *		@created 24.10.2025
 *
 * C1雷达+ESP32
 * 用于溥渊未来技术学院新生杯
 */
#include "rpLidar.h"
#include "Arduino.h"

rpLidar::rpLidar(HardwareSerial *_mySerial, uint32_t baud)
{
	serial = _mySerial;
	serial->begin(baud);
}

stDeviceInfo_t rpLidar::getDeviceInfo()
{
	clearSerialBuffer();
	stDeviceInfo_t info;
	rp_descriptor_t descr;
	serial->write((uint8_t *)&req_message[rq_info], 2); // send Device Info request
	if (!checkForTimeout(400, 27))						// wait for Response
	{
		serial->readBytes((uint8_t *)&descr, 7);
		serial->readBytes((uint8_t *)&info, 20);
	}
	return info;
}

stDeviceStatus_t rpLidar::getDeviceHealth()
{
	clearSerialBuffer(); // remove old data in SerialBuffer
	rp_descriptor_t descr;
	stDeviceStatus_t deviceStatus;
	serial->write((uint8_t *)&req_message[rq_health], 2); // send device health request
	if (!checkForTimeout(400, 10))						  // wait for response
	{
		serial->readBytes((uint8_t *)&descr, 7);
		serial->readBytes((uint8_t *)&deviceStatus, 3);
	}
	return deviceStatus;
}

uint16_t rpLidar::scanStandard()
{
	start();
	return readMeasurePoints();
}

void rpLidar::resetDevice()
{
	serial->write((uint8_t *)&req_message[rq_reset], 2); // send reset request
	delay(800);											 // wait for reboot
	clearSerialBuffer();								 // remove old data in SerialBuffer
	status = false;
}

void rpLidar::stopDevice()
{
	serial->write((uint8_t *)&req_message[rq_stop], 2);
}

bool rpLidar::start()
{
	serial->write((uint8_t *)&req_message[rq_scan], 2); // standard scan request

	rp_descriptor_t descr;

	if (!checkForTimeout(100, 7)) // wait for response
	{
		serial->readBytes((uint8_t *)&descr, 7);
		printf("descr: ");
		for (int i = 0; i < 7; i++)
			printf("%x ", descr[i]);
		printf("\r\n");
		scanMode = standard;
		status = true;
		return compareDescriptor(descr, resp_descriptor[startScan]);
	}
	return false;
}

uint16_t rpLidar::readMeasurePoints()
{
	uint16_t count = 0;
	count = awaitStandardScan();
	return count;
}

uint16_t rpLidar::awaitStandardScan()
{
	serial->flush();
	uint16_t count = 0;
	stScanDataPoint_t point;
	auto timeout = millis() + 100;
	while (serial->available() >= 5 && millis() < timeout)
	{
		serial->readBytes((uint8_t *)&point, 5);
		//这三个数据看通讯协议
		float angle = (point.angle_high * 128 + point.angle_low / 2) / 64.0;
		float distance = (point.distance_high * 256 + point.distance_low) / 4.0;
		int quality = point.quality / 4;
		if (point.quality & 0x01 == (point.quality & 0x02) / 2)
		{
			/*
			Serial.print(point.quality, BIN);
			Serial.print(",");
			Serial.print(point.angle_low, BIN);
			Serial.print(",");
			Serial.print(point.angle_high, BIN);
			Serial.print(",");
			Serial.print(point.distance_low, BIN);
			Serial.print(",");
			Serial.print(point.distance_high, BIN);
			Serial.println(",");
			//*/
			clearSerialBuffer();
			return count;
		}
		if (quality < 15 || distance == 0)
			continue;

		// Serial.printf("A%.1f\tD%.1f\tQ%d\n",angle,distance,quality);
		DataBuffer[count] = point;
		count++; // new point
	}
	return count;
}

void rpLidar::setAngleOfInterest(uint16_t _left, uint16_t _right)
{
	// setter
	interestAngleLeft = _left;
	interestAngleRight = _right;
}

bool rpLidar::isDataBetweenBorders(stScanDataPoint_t _point)
{
	float angle = calcAngle(_point.angle_low, _point.angle_high);
	if ((angle >= interestAngleLeft) && (angle <= interestAngleRight))
	{
		return true;
	}
	return false;
}

bool rpLidar::isDataBetweenBorders(float _angle)
{
	if ((_angle > interestAngleLeft) && (_angle < interestAngleRight))
	{
		return true;
	}
	return false;
}

bool rpLidar::isDataValid(stScanDataPoint_t _point)
{
	if (calcDistance(_point.distance_low, _point.distance_high) > 0)
	{
		return true;
	}
	return false;
}

bool rpLidar::isDataValid(uint16_t _distance)
{
	if (_distance > 0)
	{
		return true;
	}
	return false;
}

bool rpLidar::isRunning()
{
	return status;
}

uint8_t rpLidar::isScanMode()
{
	return scanMode;
}

float rpLidar::calcAngle(uint8_t _lowByte, uint8_t _highByte)
{
	uint16_t winkel = _highByte << 7;
	winkel |= _lowByte >> 1;
	return winkel / 64.0;
}

float rpLidar::calcCapsuledAngle(uint16_t _Wi, uint16_t _Wi2, uint8_t _k)
{
	float angle1 = _Wi / 64.00;
	float angle2 = _Wi2 / 64.00;
	float result;
	if (angle1 <= angle2)
	{
		result = angle1 + ((angle2 - angle1) / 40) * _k;
	}
	else
	{
		result = angle1 + ((360 + angle2 - angle1) / 40) * _k;
	}
	if (result > 360.0)
	{
		result = result - 360.0;
	}
	return result;
}

float rpLidar::calcDistance(uint8_t _lowByte, uint8_t _highByte)
{
	uint16_t distance = (_highByte) << 8;
	distance |= _lowByte;
	return distance / 4.0;
}

//-----------------------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------------------------//
//											 Debug Functions
//-----------------------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------------------------//

void rpLidar::DebugPrintMeasurePoints(int16_t _count)
{
	Serial.println(_count - 1);
	if (_count <= 0)
		return;

	for (uint16_t i = 0; i < _count - 1; i++)
	{
		if (isDataBetweenBorders(DataBuffer[i]) && isDataValid(DataBuffer[i]))
		{
			Serial.print(i);
			Serial.print("\t|\t");
			Serial.print((DataBuffer[i].quality) >> 2, DEC);
			Serial.print("\t|\t");
			Serial.print(calcAngle(DataBuffer[i].angle_low, DataBuffer[i].angle_high));
			Serial.print("\t|\t");
			Serial.print(calcDistance(DataBuffer[i].distance_low, DataBuffer[i].distance_high));
			Serial.println();
		}
	}
}

void rpLidar::DebugPrintDeviceErrorStatus(stDeviceStatus_t _status)
{
	Serial.println("\n--Device Health--");
	Serial.print("Status:");
	Serial.println(_status.status);
	Serial.print("Error Low:");
	Serial.print(_status.errorCode_low);
	Serial.print("Error High:");
	Serial.println(_status.errorCode_high);
	Serial.println('\n');
}

void rpLidar::DebugPrintDeviceInfo(stDeviceInfo_t _info)
{
	Serial.println("\n--Device Info--");
	Serial.print("Model:");
	Serial.println(_info.model);
	Serial.print("Firmware:");
	Serial.print(_info.firmware_major);
	Serial.print(".");
	Serial.println(_info.firmware_minor);
	Serial.print("Hardware:");
	Serial.println(_info.hardware);
	Serial.print("Serial Number:");
	for (uint16_t i = 0; i < 16; i++)
	{
		Serial.print(_info.serialnumber[i], HEX);
	}
	Serial.println('\n');
}

void rpLidar::DebugPrintDescriptor(rp_descriptor_t _descriptor)
{
	Serial.print("Descriptor : ");
	for (uint8_t i = 0; i < 7; i++)
	{
		Serial.print(_descriptor[i], HEX);
		Serial.print("|");
	}
	Serial.println();
}

void rpLidar::DebugPrintBufferAsHex()
{
	for (uint16_t i = 0; i < sizeof(DataBuffer) / sizeof(scanDataPoint); i++)
	{
		Serial.print("0x");
		Serial.print(DataBuffer[i].quality, HEX);
		Serial.print(",");
		Serial.print("0x");
		Serial.print(DataBuffer[i].angle_low, HEX);
		Serial.print(",");
		Serial.print("0x");
		Serial.print(DataBuffer[i].angle_high, HEX);
		Serial.print(",");
		Serial.print("0x");
		Serial.print(DataBuffer[i].distance_low, HEX);
		Serial.print(",");
		Serial.print("0x");
		Serial.print(DataBuffer[i].distance_high, HEX);
		Serial.println(",");
	}
	Serial.println();
}

//-----------------------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------------------------//
//											End Debug Functions
//-----------------------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------------------------//

bool rpLidar::compareDescriptor(rp_descriptor_t _descr1, rp_descriptor_t _descr2)
{
	for (size_t i = 0; i < sizeof(rp_descriptor_t); i++)
	{
		if (_descr1[i] != _descr2[i])
		{
			return false;
		}
	}
	return true;
}

void rpLidar::clearSerialBuffer()
{
	while (serial->available()) // read as long the hardware buffer is not empty
	{
		serial->read();
	}
}

bool rpLidar::checkCRC(stExpressDataPacket_t _package, uint8_t _crc)
{
	uint8_t crc = 0;
	crc = (uint8_t)_package.angle & 0x00FF;
	crc ^= (uint8_t)(_package.angle >> 8);
	for (int i = 0; i < 40; i++)
	{
		crc ^= (uint8_t)_package.cabin[i];
		crc ^= (uint8_t)(_package.cabin[i] >> 8);
	}
	if (_crc == crc)
	{
		return true;
	}
	return false;
}

bool rpLidar::checkForTimeout(uint32_t _time, size_t _size)
{
	float startTime = millis();
	while (!(serial->available() >= _size))
	{
		if (millis() > (startTime + _time))
		{
			Serial.println("Lidar Timeout");
			return true;
		}
	}
	return false;
}

double rpLidar::calcAngle(stExpressDataPacket_t *_packets, uint16_t _k)
{
	double angle1 = (_packets->angle & 0x7FFF) / 64.00;
	_packets++;
	double angle2 = (_packets->angle & 0x7FFF) / 64.00;
	double result;
	if (angle1 <= angle2)
	{
		result = angle1 + ((angle2 - angle1) / 40) * _k;
	}
	else
	{
		result = angle1 + ((360 + angle2 - angle1) / 40) * _k;
	}
	if (result > 360.0)
	{
		result = result - 360.0;
	}
	return result;
}