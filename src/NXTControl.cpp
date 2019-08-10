#include "NXTControl.h"


NXTControl::NXTControl(){
	_serial = &Serial;
}

NXTControl::NXTControl(Stream &serial){
	_serial = &serial;
}

void NXTControl::StartProgram(String name){

	byte size = name.length() + 5;
	byte commandToSend[size];

	commandToSend[0] = size-2;
	commandToSend[1] = 0x00;
	commandToSend[2] = DIRECT_COMMAND;
	commandToSend[3] = COMMAND_START_PROGRAM;

	name += "\0";

	for(byte i=4; i<name.length()+5; i++){
	    commandToSend[i] = name.charAt(i-4);
	}

	_serial->write(commandToSend, sizeof(commandToSend));
}

void NXTControl::StopProgram(){
	byte commandToSend[] = {
			0x02,
			0x00,
			DIRECT_COMMAND,
			COMMAND_STOP_PROGRAM
		};

	_serial->write(commandToSend, sizeof(commandToSend));
}

void NXTControl::PlayTone(unsigned int frequency, unsigned int duration){
	byte commandToSend[] =
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

	_serial->write(commandToSend, sizeof(commandToSend));
}

void NXTControl::SetOutputState(byte port, sbyte power, byte mode,
		byte regulationMode, sbyte turnRatio, byte runState,
		unsigned long tachoLimit){

	power = constrain(power, -100, 100);

	byte commandToSend[] = 
	{
		0x0C,
		0x00,
		DIRECT_COMMAND,
		COMMAND_SET_OUTPUT_STATE,
		port,
		power,
		mode,
		regulationMode,
		turnRatio,
		runState,
		byteRead(tachoLimit, 0),
		byteRead(tachoLimit, 1),
		byteRead(tachoLimit, 2),
		byteRead(tachoLimit, 3)
	};

	if(port > OUT_C){
		switch (port){
			case OUT_AB:
				commandToSend[4] = OUT_A;
				_serial->write(commandToSend, sizeof(commandToSend));					 
				commandToSend[4] = OUT_B;
				_serial->write(commandToSend, sizeof(commandToSend));					 
				break;
			case OUT_AC:
				commandToSend[4] = OUT_A;
				_serial->write(commandToSend, sizeof(commandToSend));					 
				commandToSend[4] = OUT_C;
				_serial->write(commandToSend, sizeof(commandToSend));					 
				break;
			case OUT_BC:
				commandToSend[4] = OUT_B;
				_serial->write(commandToSend, sizeof(commandToSend));					 
				commandToSend[4] = OUT_C;
				_serial->write(commandToSend, sizeof(commandToSend));					 
				break;
			case OUT_ABC:
				commandToSend[4] = 0xFF;
				_serial->write(commandToSend, sizeof(commandToSend));					 
				break;
		}
	}else{
		_serial->write(commandToSend, sizeof(commandToSend));					 
	}
					 
}

void NXTControl::OnFwd(byte port, sbyte power){
	SetOutputState(port, power, MODE_MOTOR_ON, REGMODE_IDLE, 0, RUNSTATE_RUNNING, 0);
}

void NXTControl::OnRev(byte port, sbyte power){
	SetOutputState(port, -power, MODE_MOTOR_ON, REGMODE_IDLE, 0, RUNSTATE_RUNNING, 0);
}

void NXTControl::OnFwdReg(byte port, sbyte power, byte regMode){
	SetOutputState(port, power, MODE_MOTOR_ON, regMode, 0, RUNSTATE_RUNNING, 0);
}

void NXTControl::OnRevReg(byte port, sbyte power, byte regMode){
	SetOutputState(port, -power, MODE_MOTOR_ON, regMode, 0, RUNSTATE_RUNNING, 0);
}

void NXTControl::Off(byte port){
	SetOutputState(port, 0, MODE_BRAKE, regMode, 0, RUNSTATE_RUNNING, 0);
}

void NXTControl::SetInputMode(byte port, byte sensorType, byte sensorMode){
	byte commandToSend[] = 
	{
		0x05,
		0x00,
		DIRECT_COMMAND,
		COMMAND_SET_INPUT_MODE,
		port,
		sensorType,
		sensorMode
	};

	_serial->write(commandToSend, sizeof(commandToSend));
}


bool NXTControl::GetOutputState(byte port, OutputState &params){

	if(port > OUT_C) return false;

	byte commandToSend[] = {0x03, 0x00, DIRECT_COMMAND_RESPONSE,
						  COMMAND_GET_OUTPUT_STATE,
						  port};

	_serial->write(commandToSend, sizeof(commandToSend));
	
	unsigned long time = millis();
	while(!_serial->available()){
		// expect to receive the values
		if(millis() - time > 100)
			break;
	}

	byte returnPackage[27];
	_serial->readBytes(returnPackage, 27);

	params.statusByte		= returnPackage[4]
	params.port 			= returnPackage[5];
	params.power 			= returnPackage[6];
	params.mode 			= returnPackage[7];
	params.regulationMode	= returnPackage[8];
	params.turnRatio 		= returnPackage[9];
	params.runState 		= returnPackage[10];
	params.tachoLimit 		= returnPackage[11] |
							  (returnPackage[12] << 8) |
							  (returnPackage[13] << 16) |
							  (returnPackage[14] << 32);
	params.tachoCount 		= returnPackage[15] |
							  (returnPackage[16] << 8) |
							  (returnPackage[17] << 16) |
							  (returnPackage[18] << 32);
	params.blockTachoCount 	= returnPackage[19] |
							  (returnPackage[20] << 8) |
							  (returnPackage[21] << 16) |
							  (returnPackage[22] << 32);
	params.rotationCount	= returnPackage[23] |
							  (returnPackage[24] << 8) |
							  (returnPackage[25] << 16) |
							  (returnPackage[26] << 32);

	if(params.statusByte != 0)
		return false;

	return true;
}

bool NXTControl::GetInputValues(byte port, InputValues &params){

	if(port > S4) return false;

	byte commandToSend[] = {0x03, 0x00, DIRECT_COMMAND_RESPONSE,
						  COMMAND_GET_INPUT_VALUES,
						  port};

	_serial->write(commandToSend, sizeof(commandToSend));
	
	unsigned long time = millis();
	while(!_serial->available()){
		// expect to receive the values
		if(millis() - time > 100)
			break;
	}

	byte returnPackage[18];
	_serial->readBytes(returnPackage, 18);

	statusByte;
	port;
	isValid;
	isCalibrated;
	sensorType;
	sensorMode;
	rawValue;
	normalizedValue;
	scaledValue;
	calibratedValue;

	params.statusByte		= returnPackage[4]
	params.port 			= returnPackage[5];
	params.isValid 			= returnPackage[6];
	params.isCalibrated		= returnPackage[7];
	params.sensorType		= returnPackage[8];
	params.sensorMode 		= returnPackage[9];
	params.rawValue			= returnPackage[10] |
							  (returnPackage[11] << 8);
	params.normalizedValue	= returnPackage[12] |
							  (returnPackage[13] << 8);
	params.scaledValue		= returnPackage[14] |
							  (returnPackage[15] << 8);
	params.calibratedValue	= returnPackage[16] |
							  (returnPackage[17] << 8);

	if(params.statusByte != 0)
		return false;

	return true;
}

void NXTControl::ResetMotorPosition(byte port, bool isRelative = true){
	byte commandToSend[] = {0x04,
						  0x00,
						  DIRECT_COMMAND,
						  COMMAND_RESET_MOTOR_POSITION,
						  port,
						  isRelative
						 };

	if(port > OUT_C){
		switch (port) {
		    case OUT_AB:
				commandToSend[4] = 	OUT_A;		 
				_serial->write(commandToSend, sizeof(commandToSend));					 
				commandToSend[4] = 	OUT_B;		 
				_serial->write(commandToSend, sizeof(commandToSend));					 
		    	break;
		    case OUT_AC:
				commandToSend[4] = 	OUT_A;		 
				_serial->write(commandToSend, sizeof(commandToSend));					 
				commandToSend[4] = 	OUT_C;		 
				_serial->write(commandToSend, sizeof(commandToSend));	
		      	break;
		    case OUT_BC:
				commandToSend[4] = 	OUT_B;		 
				_serial->write(commandToSend, sizeof(commandToSend));					 
				commandToSend[4] = 	OUT_C;		 
				_serial->write(commandToSend, sizeof(commandToSend));	
		      	break;		      
		    case OUT_ABC:
				commandToSend[4] = 0xFF;
				_serial->write(commandToSend, sizeof(commandToSend));					 
				break;
		}
	}else{
		_serial->write(commandToSend, sizeof(commandToSend));					 
	}					 

}

void NXTControl::RotateMotor(byte port, int power, int degrees){

	OutputState params;

	if(port > OUT_C){
		switch (port) {
		    case OUT_AB:
				ResetMotorPosition(OUT_A, true);
				delay(WAIT_TIME);
				GetOutputState(OUT_A, params);
				delay(WAIT_TIME);

				OnFwd(port, power);
				delay(WAIT_TIME);

				while(params.BlockTachoCount < degrees){
				  GetOutputState(OUT_A, params);
				  delay(WAIT_TIME);
				}
				break;
		    case OUT_AC:
				ResetMotorPosition(OUT_A, true);
				delay(WAIT_TIME);
				GetOutputState(OUT_A, params);
				delay(WAIT_TIME);

				OnFwd(port, power);
				delay(WAIT_TIME);

				while(params.BlockTachoCount < degrees){
				  GetOutputState(OUT_A, params);
				  delay(WAIT_TIME);
				}
				break;
		    case OUT_BC:
				ResetMotorPosition(OUT_B, true);
				delay(WAIT_TIME);
				GetOutputState(OUT_B, params);
				delay(WAIT_TIME);

				OnFwd(port, power);
				delay(WAIT_TIME);

				while(params.BlockTachoCount < degrees){
				  GetOutputState(OUT_B, params);
				  delay(WAIT_TIME);
				}
				break;		      
		    case OUT_ABC:
				ResetMotorPosition(OUT_A, true);
				delay(WAIT_TIME);
				GetOutputState(OUT_A, params);
				delay(WAIT_TIME);

				OnFwd(port, power);
				delay(WAIT_TIME);

				while(params.BlockTachoCount < degrees){
				  GetOutputState(OUT_A, params);
				  delay(WAIT_TIME);
				}
		    	break;
		}
	}else{
		ResetMotorPosition(port, true);
		delay(WAIT_TIME);
		GetOutputState(port, params);
		delay(WAIT_TIME);

		OnFwd(port, power);
		delay(WAIT_TIME);

		while(params.BlockTachoCount < degrees){
		    GetOutputState(port, params);
		    delay(WAIT_TIME);
		}
	}

	Off(port);
}
