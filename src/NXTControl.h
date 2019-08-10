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


#define DIRECT_COMMAND 0x80
#define DIRECT_COMMAND_RESPONSE 0x00

#define COMMAND_START_PROG 0x00
#define COMMAND_STOP_PROG 0x01
#define COMMAND_PLAY_SOUND_FILE 0x02
#define COMMAND_PLAY_TONE 0x03
#define COMMAND_SET_OUTPUT 0x04
#define COMMAND_SET_INPUT 0x05
#define COMMAND_GET_OUTPUT 0x06
#define COMMAND_GET_INPUT 0x07
#define COMMAND_RESET_INPUT_SCALED_VALUE 0x08
#define COMMAND_MESSAGE_WRITE 0x09
#define COMMAND_RESET_MOTOR 0x0A
#define COMMAND_GET_BATTERY_LEVEL 0x0B
#define COMMAND_STOP_SOUND_PLAYBACK 0x0C
#define COMMAND_KEEP_ALIVE 0x0D
#define COMMAND_LS_GET_STATUS 0x0E
#define COMMAND_LS_WRITE 0x0F
#define COMMAND_LS_READ 0x10
#define COMMAND_GET_CURRENT_PROGRAM_NAME 0x11
#define COMMAND_MESSAGE_READ 0x13

// == COMMANDS SET_OUTPUT_STATE == //
#define OUT_A 	0x00
#define OUT_B 	0x01
#define OUT_C   0x02
#define OUT_AB  0x03
#define OUT_AC  0x04
#define OUT_BC  0x05
#define OUT_ABC 0x06

#define MODE_MOTORON 	0x01
#define MODE_BRAKE		0x02
#define MODE_REGULATED 	0x04

#define REGMODE_IDLE	0x00
#define REGMODE_SPEED	0x01
#define REGMODE_SYNC	0x02

#define RUNSTATE_IDLE		0x00
#define RUNSTATE_RAMPUP 	0x10
#define RUNSTATE_RUNNING	0x20
#define RUNSTATE_RAMPDOWN	0x40

#define WAIT_TIME	45

struct OUTPUT_STATE{
	byte Port 				 = 0;
	byte Mode 				 = 0;
	byte RegMode 			 = 0;
	byte RunState 			 = 0;
	int Power 				 = 0;
	int TurnRatio 			 = 0;
	long TachoCount 		 = 0;
	long BlockTachoCount	 = 0;
	long RotationCount		 = 0;
	unsigned long TachoLimit = 0;
};

class NXTControl{
public:
	NXTControl();
	NXTControl(Stream &serial);

	void PlayTone(unsigned int freq, unsigned int duration);
	void OnFwd(byte port, int power);
	void OnRev(byte port, int power);
	void OnFwdReg(byte port, int power, byte regMode, 
			 byte turnRatio, byte runState, byte tachoLimit);
	void OnRevReg(byte port, int power, byte regMode, 
			 byte turnRatio, byte runState, byte tachoLimit);

	void Off(int port);

	bool GetOutputState(byte port, OUTPUT_STATE &params);
	void ResetMotorPosition(byte port, bool isRelative);

	void RotateMotor(byte port, int power, int degrees);

	void StartProgram(String name);
	void StopProgram();

private:
	void send(byte port, int power, byte regMode, 
		 	  byte turnRatio, byte runState, byte tachoLimit);

	Stream *_serial;	
};

#endif