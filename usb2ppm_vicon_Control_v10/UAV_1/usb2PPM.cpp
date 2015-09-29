#include "StdAfx.h"
#include "usb2PPM.h"
#include <algorithm>

DWORD WINAPI WriteThreadFunc(LPVOID);

extern CTRL_Input g_XPCommand;
extern HANDLE g_XCommandMutex;
char* rtrn = NULL;

Serial::Serial(void)
{
	hWriteThread = INVALID_HANDLE_VALUE;
}

Serial::~Serial()
{
	//Check if we are connected before trying to disconnect
	if (this->connected)
	{
		//We're no longer connected
		this->connected = false;
		//Close the serial handler
		CloseHandle(this->hSerial);
	}
}

int Serial::Ini(char *portName)
{
	int iResult =0;
	//We're not yet connected
	this->connected = false;
	//Try to connect to the given port throuh CreateFile
	this->hSerial = CreateFile(portName,
		GENERIC_READ | GENERIC_WRITE,
		0,
		NULL,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL,
		NULL);
	//Check if the connection was successfull
	if (this->hSerial == INVALID_HANDLE_VALUE)
	{
		//If not success full display an Error
		if (GetLastError() == ERROR_FILE_NOT_FOUND){
			//Print Error if neccessary
			printf("ERROR: Handle was not attached. Reason: %s not available.\n", portName);
		}
		else
		{
			printf("ERROR!!!");
			iResult = 1;
			return iResult;
		}
	}
	else
	{
		//If connected we try to set the comm parameters
		DCB dcbSerialParams = { 0 };

		//Try to get the current
		if (!GetCommState(this->hSerial, &dcbSerialParams))
		{
			//If impossible, show an error
			printf("failed to get current serial parameters!");
		}
		else
		{
			dcbSerialParams.BaudRate = CBR_115200; //Define serial connection parameters for the arduino board
			dcbSerialParams.ByteSize = 8;
			dcbSerialParams.StopBits = ONESTOPBIT;
			dcbSerialParams.Parity = NOPARITY;
			//Setting the DTR to Control_Enable ensures that the Arduino is properly
			//reset upon establishing a connection
			dcbSerialParams.fDtrControl = DTR_CONTROL_ENABLE;

			//Set the parameters and check for their proper application
			if (!SetCommState(hSerial, &dcbSerialParams))
			{
				printf("ALERT: Could not set Serial Port parameters");
			}
			else
			{
				//If everything went fine we're connected
				this->connected = true;
				//Flush any remaining characters in the buffers 
				PurgeComm(this->hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR);
				//We wait 2s as the arduino board will be reseting
				Sleep(ARDUINO_WAIT_TIME);
			}
		}
		hWriteThread = CreateThread(NULL, 0, WriteThreadFunc, this, 0, NULL);	
	}
	//hWriteThread = CreateThread(NULL, 0, WriteThreadFunc, this, 0, NULL);
	return 0;
}


int Serial::ReadData(char *buffer, unsigned int nbChar)
{
	//Number of bytes we'll have read
	DWORD bytesRead;
	//Number of bytes we'll really ask to read
	unsigned int toRead;

	//Use the ClearCommError function to get status info on the Serial port
	ClearCommError(this->hSerial, &this->errors, &this->status);

	//Check if there is something to read
	if (this->status.cbInQue>0)
	{
		//If there is we check if there is enough data to read the required number
		//of characters, if not we'll read only the available characters to prevent
		//locking of the application.
		if (this->status.cbInQue>nbChar)
		{
			toRead = nbChar;
		}
		else
		{
			toRead = this->status.cbInQue;
		}

		//Try to read the require number of chars, and return the number of read bytes on success
		if (ReadFile(this->hSerial, buffer, toRead, &bytesRead, NULL) && bytesRead != 0)
		{
			return bytesRead;
		}

	}

	//If nothing has been read, or that an error was detected return -1
	return -1;

}


bool Serial::WriteData(char *buffer, unsigned int nbChar)
{
	DWORD bytesSend;

	//Try to write the buffer on the Serial port
	if (!WriteFile(this->hSerial, (void *)buffer, nbChar, &bytesSend, 0))
	{
		//In case it don't work get comm error and return false
		ClearCommError(this->hSerial, &this->errors, &this->status);

		return false;
	}
	else
		return true;
}

bool Serial::IsConnected()
{
	//Simply return the connection status
	return this->connected;
}


char* encode_PPM(short cmd_ch1, short cmd_ch2, short cmd_ch3, short cmd_ch4){
	
	int cmd_ch[4] = { 0, 0, 0, 0}; //Default values for each channels
	char output_string[26]="\0";
	char* output_string_p = output_string;
	rtrn = (char*)malloc(26);

	char const_ch[4] = { 'x','y', 'z', 'k' };
	///////////////////////////////////////////
	cmd_ch[0] = (int) cmd_ch1;
	cmd_ch[1] = (int) cmd_ch2;
	cmd_ch[2] = (int) cmd_ch3;
	cmd_ch[3] = (int) cmd_ch4;
	//////////////////////////////////////////
	int j = 0;
	
	//for loop for each channel value processing
	for (int i = 0; i < 4; i++){
		//Set maximum values
		if (cmd_ch[i] > 4000){
			cmd_ch[i] = 4000;
		}
		else if (cmd_ch[i] < 1){
			cmd_ch[i] = 0;
		}		
		char cmd_ch_s[5]="";
		char* cmd_ch_sp = cmd_ch_s;

		_itoa_s(cmd_ch[i], cmd_ch_s, 10);

		//convert to single string for communication with Arduino

		*(output_string_p + j) = const_ch[i];
		j++;
		while (*(output_string_p + j) = *(cmd_ch_sp++))
		{
			j++;
		}
		
	}
	//printf("string :%s\n", output_string);
	strcpy_s(rtrn,25, output_string);
	return rtrn;
}

DWORD WINAPI WriteThreadFunc(LPVOID lpParam)
{
	int iResult = 0;	
	
	Serial *sp = (Serial *)lpParam;

	CTRL_Input cur_XPCommand = {0};
	/*
	unsigned short crc = 0;
	short *ptr = NULL;
	unsigned short length = 0;

	unsigned short *ptr_p = NULL;
	unsigned short packets = 0x0400;
	*/

	//printf("Serial Interface Initiation\n");
	//Serial* SP = new Serial("\\\\.\\COM24");    // adjust as needed

	char incomingData[27] = "";			// don't forget to pre-allocate memory
	char outgoingData[27] = "";


	int dataLength = 26;
	int readResult = 0;
	int i=0;
	int ch1=650;
	int cnt = 0;
	int cnt_hz = 0;

	while(true)
	{
		// Copy the global XCommand
		WaitForSingleObject(g_XCommandMutex, INFINITE);
		memcpy(&cur_XPCommand, &g_XPCommand, sizeof(CTRL_INPUT));
		ReleaseMutex(g_XCommandMutex);
		
		//printf("Bytes read: (-1 means no data available) %i\n", readResult);
		//Ch1 : ROLL Ch2: Thrust Ch3: Pitch Ch4: Yaw
		strcpy_s(outgoingData, 26, encode_PPM(cur_XPCommand.roll, cur_XPCommand.thrust, cur_XPCommand.pitch, cur_XPCommand.yaw));
		//strcpy_s(outgoingData, 26, encode_PPM(512.0, 512.0, 512.0, cur_XPCommand.yaw));
		//printf("Incoming data : %s\n", incomingData);
		//SP->WriteData(outgoingData, dataLength);
		sp->WriteData(outgoingData, dataLength);
		//std::string test(incomingData);
		readResult = sp->ReadData(incomingData, dataLength);
		
		ch1 ++;
		

		free(rtrn);
		cnt++;
		printf("data : %s \n", outgoingData);
		//printf("data : %s \n", incomingData);
		
		if (cnt == 50){
			//Read data for handshake
			readResult = sp->ReadData(incomingData, dataLength);
			cnt_hz++;
			cnt = 0; 
		}
		if (cnt ==1023){
			cnt = 0;
		}
		
		Sleep(20);
	}
	return iResult;
}
