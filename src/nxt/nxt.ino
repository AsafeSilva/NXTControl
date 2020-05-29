#include <SoftwareSerial.h>
#include "NXTControl.h"

SoftwareSerial serial(2, 3);

NXTControl nxt(serial);

InputValues inputData;

void setup() {
	serial.begin(38400);
	Serial.begin(9600);

	nxt.PlayTone(1000, 100);
	delay(100);
	nxt.PlayTone(1400, 100);

	nxt.SetInputMode(S2, LOWSPEED_9V, RAW_MODE);

	//////////
	
	byte commandToSend[] = {0x08, 0x00, DIRECT_COMMAND,
						  COMMAND_LS_WRITE,
						  port,
						  0x03,	// TX length
						  0x01, // RX length
						  0x02, 0x41, 0x02}; // Continuous mensurement command

	serial.write(commandToSend, sizeof(commandToSend));

	////////////
}

void loop() {

	byte commandToSend[] = {0x03, 0x00, DIRECT_COMMAND_RESPONSE,
							COMMAND_LS_GET_STATUS,
							S2};

	serial.write(commandToSend, sizeof(commandToSend));

	byte returnPackage[6];
	serial.readBytes(returnPackage, 6);

	// Serial.print("status: ");	Serial.println(returnPackage[4]);
	// Serial.print("bytes: ");	Serial.println(returnPackage[5]);
	// Serial.println("--------------");

	////////////////////

	byte _commandToSend[] = {0x, 0x00, DIRECT_COMMAND_RESPONSE,
							COMMAND_LS_READ,
							S2};

	serial.write(_commandToSend, sizeof(_commandToSend));

	byte returnPackage[6];
	serial.readBytes(returnPackage, 6);




	// nxt.GetInputValues(S2, inputData);

	// Serial.print("status: "); Serial.print(inputData.statusByte);		Serial.println();
	// Serial.print("port: "); Serial.print(inputData.port);		Serial.println();
	// Serial.print("isValid: "); Serial.print(inputData.isValid);		Serial.println();
	// Serial.print("isCalibrated: "); Serial.print(inputData.isCalibrated);		Serial.println();
	// Serial.print("sensorType: "); Serial.print(inputData.sensorType);		Serial.println();
	// Serial.print("sensorMode: "); Serial.print(inputData.sensorMode);		Serial.println();
	// Serial.print("rawValue: "); Serial.print(inputData.rawValue);		Serial.println();
	// Serial.print("normalizedValue: "); Serial.print(inputData.normalizedValue);		Serial.println();
	// Serial.print("scaledValue: "); Serial.print(inputData.scaledValue);		Serial.println();
	// Serial.print("calibratedValue: "); Serial.print(inputData.calibratedValue);		Serial.println();
	// Serial.println("------------------");

	delay(600);

}
