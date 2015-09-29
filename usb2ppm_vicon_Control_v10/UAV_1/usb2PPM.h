#include <windows.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include "DataStructure.h"


#ifndef SERIALCLASS_H_INCLUDED
#define SERIALCLASS_H_INCLUDED
#define ARDUINO_WAIT_TIME 2000

using namespace std;

extern char* rtrn;

char* encode_PPM(int cmd_ch1, int cmd_ch2, int cmd_ch3, int cmd_ch4);

class Serial
{
private:
	//Serial comm handler
	HANDLE hSerial;
	//Connection status
	bool connected;
	//Get various information about the connection
	COMSTAT status;

	DWORD errors;//Keep track of last error

public:
	//Initialize Serial communication with the given COM port
	Serial(void);
	~Serial();//Close the connection
public:
	int Ini(char *portName);
	//Read data in a buffer, if nbChar is greater than the
	//maximum number of bytes available, it will return only the
	//bytes available. The function return -1 when nothing could
	//be read, the number of bytes actually read.
	int ReadData(char *buffer, unsigned int nbChar);
	//Writes data from a buffer through the Serial connection
	//return true on success.
	bool WriteData(char *buffer, unsigned int nbChar);
	//Check if we are actually connected
	bool IsConnected();
private:
	HANDLE hWriteThread;
};

#endif // SERIALCLASS_H_INCLUDED