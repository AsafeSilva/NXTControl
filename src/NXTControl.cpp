#include <Arduino.h>

#include "NXTControl.h"

#include "HardwareSerial.h"


NXTControl::NXTControl(){
	_serial = &Serial;
}

NXTControl::NXTControl(Stream &serial){
	_serial = &serial;
}

void NXTControl::send(byte port, int power, byte regMode, 
		 byte turnRatio, byte runState, byte tachoLimit){

	byte sendCommand[] = 
	{
		0x0C,
		0x00,
		DIRECT_COMMAND,
		COMMAND_SET_OUTPUT,
		port,
		power,
		0x07,
		regMode,
		turnRatio,
		runState,
		0x00, 0x00, 0x00, 0x00
	};


	_serial->write(sendCommand, sizeof(sendCommand));					 
					 
}

void NXTControl::PlayTone(unsigned int freq, unsigned int duration){
	byte sendCommand[] =
	{
		0x06,
		0x00,
		DIRECT_COMMAND,
		COMMAND_PLAY_TONE,
		lowByte(freq),
		highByte(freq),
		lowByte(duration),
		highByte(duration)	
	};


	_serial->write(sendCommand, sizeof(sendCommand));
}

void NXTControl::OnFwd(byte port, int power){
	if(port > OUT_C){
		if(port == OUT_AB){
			send(OUT_A, power, REGMODE_IDLE, 0, RUNSTATE_RUNNING, 0);
			delay(WAIT_TIME);
			send(OUT_B, power, REGMODE_IDLE, 0, RUNSTATE_RUNNING, 0);
		}
		if(port == OUT_AC){
			send(OUT_A, power, REGMODE_IDLE, 0, RUNSTATE_RUNNING, 0);
			delay(WAIT_TIME);
			send(OUT_C, power, REGMODE_IDLE, 0, RUNSTATE_RUNNING, 0);		
		}
		if(port == OUT_BC){
			send(OUT_B, power, REGMODE_IDLE, 0, RUNSTATE_RUNNING, 0);
			delay(WAIT_TIME);
			send(OUT_C, power, REGMODE_IDLE, 0, RUNSTATE_RUNNING, 0);			
		}
		if(port == OUT_ABC){
			send(OUT_A, power, REGMODE_IDLE, 0, RUNSTATE_RUNNING, 0);
			send(OUT_B, power, REGMODE_IDLE, 0, RUNSTATE_RUNNING, 0);		
			send(OUT_C, power, REGMODE_IDLE, 0, RUNSTATE_RUNNING, 0);		
		}
	}else{
		send(port, power, REGMODE_IDLE, 0, RUNSTATE_RUNNING, 0);
	}	

}

void NXTControl::OnRev(byte port, int power){
	OnFwd(port, -power);
}

void NXTControl::OnFwdReg(byte port, int power, byte regMode, 
		 byte turnRatio, byte runState, byte tachoLimit){

	send(port, power, regMode, turnRatio, runState, tachoLimit);

}

void NXTControl::OnRevReg(byte port, int power, byte regMode, 
		 byte turnRatio, byte runState, byte tachoLimit){

	send(port, -power, regMode, turnRatio, runState, tachoLimit);
	
}

void NXTControl::Off(int port){
	OnFwd(port, 0);
}

bool NXTControl::GetOutputState(byte port, OUTPUT_STATE &params){

	if(port > OUT_C) return false;

	byte sendCommand[] = {0x03, 0x00, DIRECT_COMMAND_REPONSE,
						  COMMAND_GET_OUTPUT,
						  port};

	_serial->write(sendCommand, sizeof(sendCommand));
	
	unsigned long time = millis();
	while(!_serial->available()){
		// espera receber os valores
		if(millis() - time > 2000)
			break;
	}

	byte receiveCommand[30];
	byte cont = 0;	
	while(_serial->available()){
		receiveCommand[cont] = _serial->read();
		cont++;
	}

	params.Port 			= receiveCommand[5];
	params.Power 			= receiveCommand[6];
	params.Mode 			= receiveCommand[7];
	params.RegMode 			= receiveCommand[8];
	params.TurnRatio 		= receiveCommand[9];
	params.RunState 		= receiveCommand[10];

	params.TachoLimit 		= receiveCommand[11] |
							  (receiveCommand[12] << 8) |
							  (receiveCommand[13] << 16) |
							  (receiveCommand[14] << 32);
	params.TachoCount 		= receiveCommand[15] |
							  (receiveCommand[16] << 8) |
							  (receiveCommand[17] << 16) |
							  (receiveCommand[18] << 32);
	params.BlockTachoCount 	= receiveCommand[19] |
							  (receiveCommand[20] << 8) |
							  (receiveCommand[21] << 16) |
							  (receiveCommand[22] << 32);
	params.RotationCount	= receiveCommand[23] |
							  (receiveCommand[24] << 8) |
							  (receiveCommand[25] << 16) |
							  (receiveCommand[26] << 32);

	if(receiveCommand[2] == 0)
		return true;

	return receiveCommand[2];	
}

void NXTControl::ResetMotorPosition(byte port, bool isRelative = true){
	byte sendCommand[] = {0x04,
						  0x00,
						  DIRECT_COMMAND,
						  COMMAND_RESET_MOTOR,
						  port,
						  isRelative
						 };

	if(port > OUT_C){
		switch (port) {
		    case OUT_AB:
				sendCommand[4] = 	OUT_A;		 
				_serial->write(sendCommand, sizeof(sendCommand));					 
				sendCommand[4] = 	OUT_B;		 
				_serial->write(sendCommand, sizeof(sendCommand));					 
		    	break;
		    case OUT_AC:
				sendCommand[4] = 	OUT_A;		 
				_serial->write(sendCommand, sizeof(sendCommand));					 
				sendCommand[4] = 	OUT_C;		 
				_serial->write(sendCommand, sizeof(sendCommand));	
		      	break;
		    case OUT_BC:
				sendCommand[4] = 	OUT_B;		 
				_serial->write(sendCommand, sizeof(sendCommand));					 
				sendCommand[4] = 	OUT_C;		 
				_serial->write(sendCommand, sizeof(sendCommand));	
		      	break;		      
		    default:;
		}
	}else{
		_serial->write(sendCommand, sizeof(sendCommand));					 
	}					 

}

void NXTControl::RotateMotor(byte port, int power, int degrees){

	OUTPUT_STATE params;

	if(port > OUT_C){
		switch (port) {
		    case OUT_AB:
		      nxt.ResetMotorPosition(OUT_A, true);
		      delay(WAIT_TIME);
		      nxt.GetOutput(OUT_A, params);
		      delay(WAIT_TIME);

		      nxt.OnFwd(port, power);
		      delay(WAIT_TIME);

		      while(params.BlockTachoCount < degrees){
		          nxt.GetOutput(OUT_A, params);
		          delay(WAIT_TIME);
		      }
		      break;
		    case OUT_AC:
		      nxt.ResetMotorPosition(OUT_A, true);
		      delay(WAIT_TIME);
		      nxt.GetOutput(OUT_A, params);
		      delay(WAIT_TIME);

		      nxt.OnFwd(port, power);
		      delay(WAIT_TIME);

		      while(params.BlockTachoCount < degrees){
		          nxt.GetOutput(OUT_A, params);
		          delay(WAIT_TIME);
		      }
		      break;
		    case OUT_BC:
		      nxt.ResetMotorPosition(OUT_B, true);
		      delay(WAIT_TIME);
		      nxt.GetOutput(OUT_B, params);
		      delay(WAIT_TIME);

		      nxt.OnFwd(port, power);
		      delay(WAIT_TIME);

		      while(params.BlockTachoCount < degrees){
		          nxt.GetOutput(OUT_B, params);
		          delay(WAIT_TIME);
		      }
		      break;		      
		    default:;
		}
	}else{
		nxt.ResetMotorPosition(port, true);
		delay(WAIT_TIME);
		nxt.GetOutput(port, params);
		delay(WAIT_TIME);

		nxt.OnFwd(port, power);
		delay(WAIT_TIME);

		while(params.BlockTachoCount < degrees){
		    nxt.GetOutput(port, params);
		    delay(WAIT_TIME);
		}
	}

		nxt.Off(port);
}

void NXTControl::StartProgram(String name){

	byte size = name.length() + 5;
	byte sendCommand[size];

	sendCommand[0] = size-2;
	sendCommand[1] = 0x00;
	sendCommand[2] = DIRECT_COMMAND;
	sendCommand[3] = COMMAND_START_PROG;

	name += "\0";

	for(byte i=4; i<name.length()+5; i++){
	    sendCommand[i] = name.charAt(i-4);
	}

	_serial->write(sendCommand, sizeof(sendCommand));
}

void NXTControl::StopProgram(){
	byte sendCommand[] = {
			0x02,
			0x00,
			DIRECT_COMMAND,
			COMMAND_STOP_PROG
		};

	_serial->write(sendCommand, sizeof(sendCommand));
}