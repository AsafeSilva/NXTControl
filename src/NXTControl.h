/**
 *
 * NXTControl.h
 *
 * Biblioteca para controle do NXT via Arduino
 *
 * Criado em 2016
 * Por Asafe Silva (github: @AsafeSilva | e-mail: asafesilva01@gmail.com)
 *
 */

#ifndef NXTControl_h
#define NXTControl_h

#include <Arduino.h>

#include "HardwareSerial.h"


#define DIRECT_COMMAND 0x80
#define DIRECT_COMMAND_RESPONSE 0x00

// Direct commands
#define COMMAND_START_PROGRAM 0x00
#define COMMAND_STOP_PROGRAM 0x01
#define COMMAND_PLAY_SOUND_FILE 0x02
#define COMMAND_PLAY_TONE 0x03
#define COMMAND_SET_OUTPUT_STATE 0x04
#define COMMAND_SET_INPUT_MODE 0x05
#define COMMAND_GET_OUTPUT_STATE 0x06
#define COMMAND_GET_INPUT_VALUES 0x07
#define COMMAND_RESET_INPUT_SCALED_VALUE 0x08
#define COMMAND_MESSAGE_WRITE 0x09
#define COMMAND_RESET_MOTOR_POSITION 0x0A
#define COMMAND_GET_BATTERY_LEVEL 0x0B
#define COMMAND_STOP_SOUND_PLAYBACK 0x0C
#define COMMAND_KEEP_ALIVE 0x0D
#define COMMAND_LS_GET_STATUS 0x0E
#define COMMAND_LS_WRITE 0x0F
#define COMMAND_LS_READ 0x10
#define COMMAND_GET_CURRENT_PROGRAM_NAME 0x11
#define COMMAND_MESSAGE_READ 0x13

// Valide enumaration for "Output Port"
#define OUT_A 	0x00
#define OUT_B 	0x01
#define OUT_C   0x02
#define OUT_AB  0x03
#define OUT_AC  0x04
#define OUT_BC  0x05
#define OUT_ABC 0x06

// Valide enumaration for "Output Mode"
#define MODE_MOTORON 	0x01
#define MODE_BRAKE		0x02
#define MODE_REGULATED 	0x04

// Valide enumaration for Output "Regulation Mode"
#define REGMODE_IDLE	0x00
#define REGMODE_SPEED	0x01
#define REGMODE_SYNC	0x02

// Valide enumaration for Output "Run State"
#define RUNSTATE_IDLE		0x00
#define RUNSTATE_RAMPUP 	0x10
#define RUNSTATE_RUNNING	0x20
#define RUNSTATE_RAMPDOWN	0x40

// Valide enumaration for Input "Port"
#define S1 0
#define S2 1
#define S3 2
#define S4 3

// Valide enumaration for Input "Sensor Type"
#define NO_SENSOR 0x00
#define SWITCH 0x01
#define TEMPERATURE 0x02
#define REFLECTION 0x03
#define ANGLE 0x04
#define LIGHT_ACTIVE 0x05
#define LICHT_INACTIVE 0x06
#define SOUND_DB 0x07
#define SOUND_DBA 0x08
#define CUSTOM 0x09
#define LOWSPEED 0x0A
#define LOWSPEED_9V 0x0B
#define NO_OF_SENSOR_TYPES 0x0C

// Valide enumaration for Input "Sensor Mode"
#define RAW_MODE 0x00
#define BOOLEAN_MODE 0x20
#define TRANSITION_COUNTER_MODE 0x40
#define PERIOD_COUNTER_MODE 0x60
#define PCT_FULL_SCALE_MODE 0x80
#define CELSIUS_MODE 0xA0
#define FAHRENHEIT_MODE 0xC0
#define ANGLE_STEPS_MODE 0xE0
#define SLOPE_MASK 0x1F
#define MODE_MASK 0xE0

// Package returned by GetOutputState function
struct OutputState{
	byte statusByte;
	byte port;
	sbyte power;
	byte mode;
	byte regulationMode;
	sbyte turnRatio;
	byte runState;
	unsigned long tachoLimit;
	long tachoCount;
	long blockTachoCount;
	long rotationCount;
};

// Package returned by GetInputValues function
struct InputValues{
	byte statusByte;
	byte port;
	bool isValid;
	bool isCalibrated;
	byte sensorType;
	byte sensorMode;
	unsigned int rawValue;
	unsigned int normalizedValue;
	int scaledValue;
	int calibratedValue;
};

#define WAIT_TIME	45

typedef int8_t sbyte;

#define byteRead(x,n) ((uint8_t) ((x >> 8*n) & 0xFF))


class NXTControl{
public:
	NXTControl();
	NXTControl(Stream &serial);

	void StartProgram(String name);
	void StopProgram();

	void PlayTone(unsigned int frequency, unsigned int duration);

	void SetOutputState(byte port, sbyte power, byte mode,
		byte regulationMode, sbyte turnRatio, byte runState,
		unsigned long tachoLimit = 0);

	void OnFwd(byte port, sbyte power);

	void OnRev(byte port, sbyte power);

	void OnFwdReg(byte port, sbyte power, byte regMode);

	void OnRevReg(byte port, sbyte power, byte regMode);

	void Off(byte port);

	void SetInputMode(byte port, byte sensorType, byte sensorMode);
	
	bool GetOutputState(byte port, OutputState &params);

	bool GetInputValues(byte port, InputValues &params);

	// void ResetInputScaledValue(byte port);

	// void MessageWrite(byte inboxNumber, int messageSize, byte* messageData);

	void ResetMotorPosition(byte port, bool isRelative);

	void RotateMotor(byte port, sbyte power, int degrees);

	// int GetBateryLevel();

	// byte* MessageRead(byte remoteInboxNumber, byte localInboxNumber, bool remove);

private:
	
	Stream *_serial;	

};

#endif