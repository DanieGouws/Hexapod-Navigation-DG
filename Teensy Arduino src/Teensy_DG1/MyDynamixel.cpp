#include "Arduino.h"
#include "MyDynamixel.h"
#include <cmath>

MyDynamixel::MyDynamixel(HardwareSerial& port, int rate, int Dir_pin): _port(port)
{
	dir_pin_ = Dir_pin;
	_port.begin(rate);
	_port.transmitterEnable(Dir_pin);
//  pinMode(dir_pin_, OUTPUT);  // Set direction pin as output
//  digitalWrite(dir_pin_, LOW);
	_port.setTimeout(100);
}

void MyDynamixel::WriteServos (uint8_t id, enum CONTROL_TABLE reg, uint8_t * data, uint8_t dataLength)
{
	uint8_t returnPack[6];
	uint8_t lengt = dataLength + 3;
	uint8_t packet[lengt + 4];

	packet[0] = packet[1] = 0xFF;
	packet[2] = id;
	packet[3] = lengt;
	packet[4] = WRITE_DATA;
	packet[5] = reg;

	for (int j = 0; j < dataLength; j++)
	{
		packet[6 + j] = data[j];
	}


	packet[lengt + 3] = 0;
	for (int i = 2; i < lengt + 3; i++)
	{
		packet[lengt + 3] += packet[i];
	}
	packet[lengt + 3] = ~packet[lengt + 3];

//  digitalWrite(dir_pin_, HIGH); // Enable TX mode
//  delayMicroseconds(1);  // Small delay for stability
	
	_port.write(packet, lengt + 4);
	_port.flush(); //wait for packet to be send

//  delayMicroseconds(1);  // Small delay before switching back
//  digitalWrite(dir_pin_, LOW); // Switch to RX mode

	delayMicroseconds(300); //fixed return delay time
	if(_port.available()>0)
	{
		uint8_t x=_port.readBytes(returnPack,6);
    x = x;   //remove the unused variable warning.
		// for(int i = 0; i < 6+dataLength; i++)
		// {
			// Serial.print(returnPack[i],HEX);
		// }
		//Serial.println(x);
	}
	while(_port.read() >= 0){}
}

void MyDynamixel::ReadServos (uint8_t id, enum CONTROL_TABLE reg, uint8_t dataLength, uint8_t* returnPack)
{
	//uint8_t returnPack[6+dataLength];
	uint8_t lengt = 4;
	uint8_t readpacket[lengt + 4];

	readpacket[0] = readpacket[1] = 0xFF;
	readpacket[2] = id;
	readpacket[3] = lengt;
	readpacket[4] = READ_DATA;
	readpacket[5] = reg;

	readpacket[6] = dataLength;

	readpacket[lengt + 3] = 0;
	for (int i = 2; i < lengt + 3; i++) 
	{
		readpacket[lengt + 3] += readpacket[i];
	}
	readpacket[lengt + 3] = ~readpacket[lengt + 3];

	_port.write(readpacket, lengt + 4);

	_port.flush(); //wait for packet to be send
	delayMicroseconds(300); //fixed return delay time	
	if(_port.available()>0)
	{
		uint8_t x=_port.readBytes(returnPack,6+dataLength);
    x = x;   //remove the unused variable warning.
		// for(int i = 0; i < 6+dataLength; i++)
		// {
		 // Serial.println(returnPack[i],HEX);
		// }
		// Serial.print(x);
	}
	while(_port.read() >= 0){}
	//return returnPack;
}

void MyDynamixel::MoveServos (uint8_t id, double position, double speed)
{
	double pos = position/0.29;
	if(pos < 0)
	{
		pos = 0;
	}
	else if(pos > 1023)
	{
		pos = 1023;
	}
	goalMovementBits[0] = (uint16_t)pos & 0xff;
	goalMovementBits[1] = (uint16_t)pos >> 8;
		
	if(speed == -10)
	{
		WriteServos(id,GOAL_POSITION_L, goalMovementBits, 2);
	}
	else
	{
		double spd = speed/0.111;
		goalMovementBits[2] = (uint16_t)(spd < 0 ? 1 : spd) & 0xff;
		goalMovementBits[3] = (uint16_t)(spd < 0 ? 1 : spd) >> 8;
		WriteServos(id,GOAL_POSITION_L, goalMovementBits, 4);
	}
}

// Present position of servo in radians, center as 0
double MyDynamixel::PresentPos (uint8_t id)
{
	double position=0;
	uint16_t pos=0;
	uint8_t retPac[8]={0,0,0,0,0,0,0,0};
	
	ReadServos (id, PRESENT_POSITION_L, 2, retPac);
	pos = (uint16_t)((retPac[6]& 0xFF) << 8) | (uint16_t)(retPac[5]& 0xFF);
	position = ((double)(pos - 512)) * SERVO_STEP;
	
	return position;
}

//double MyDynamixel::ReadLimit (uint8_t id)
//{
//  double position=0;
//  uint16_t pos=0;
//  uint8_t retPac[8]={0,0,0,0,0,0,0,0};
//  
//  ReadServos (id, A, 2, retPac);
//  pos = (uint16_t)((retPac[6]& 0xFF) << 8) | (uint16_t)(retPac[5]& 0xFF);
//  position = ((double)(pos - 512)) * SERVO_STEP;
//  
//  return position;
//}

void MyDynamixel::SyncWrite (uint8_t* id, uint8_t numMotors, enum CONTROL_TABLE reg, uint8_t * data, uint8_t dataLength)
{
	uint8_t length = (dataLength+1)*numMotors+4;
	uint8_t BroadcastID = 0xFE;
	uint8_t instruction = SYNC_WRITE;
	
	uint8_t packet[length+4];
	
	packet[0] = packet[1] = 0xFF;
	packet[2] = BroadcastID;
	packet[3] = length;
	packet[4] = instruction;
	packet[5] = reg;
	packet[6] = dataLength;
	
	int j = 0;
	int k = 0;
	for(int i = 0; i<numMotors*(dataLength+1); i++)
	{	
		if (i%(dataLength+1) == 0)
		{
			packet[7+i] = id[j];
			j = j + 1;
		}
		else
		{
			packet[7+i] = data[k];
			k = k + 1;
		}

	}

	packet[length + 3] = 0;
	for (int i = 2; i < length + 3; i++)
	{
		packet[length + 3] += packet[i];
	}
	packet[length + 3] = ~packet[length + 3];

	_port.write(packet, length + 4);
	
	_port.flush(); //wait for packet to be send

}

void MyDynamixel::SyncDisableTorque(uint8_t* id, uint8_t numMotors)
{
	uint8_t dataLength = numMotors;
	uint8_t data[dataLength];

	for(int i = 0; i<dataLength; i++)
	{
		data[i] = 0;
	}
	SyncWrite(id, numMotors, TORQUE_ENABLE, data, 1);
}

void MyDynamixel::SyncMove (uint8_t* id, double* position, double* speed, uint8_t numMotors)
{
	uint8_t dataLength = numMotors*4;
	uint8_t data[dataLength];

	for(int i = 0,j = 0; i<dataLength && j<numMotors; i+=4,j++)
	{
		double pos = position[j]/SERVO_STEP + 512;
		if(pos < 0)
		{
			pos = 0;
		}
		else if(pos > 1023)
		{
			pos = 1023;
		}
    

		// Pack 16 bit data into 8 bit data array
		data[i] = (uint16_t)pos & 0xff;
		data[i+1] = (uint16_t)pos >> 8;

		double spd = speed[j]/0.111;
		data[i+2] = (uint16_t)(spd < 0 ? 1 : spd) & 0xff;
		data[i+3] = (uint16_t)(spd < 0 ? 1 : spd) >> 8;
	}
	
	SyncWrite(id,numMotors,GOAL_POSITION_L, data, 4);
	
}
