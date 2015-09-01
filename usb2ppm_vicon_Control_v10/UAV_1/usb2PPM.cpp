#include "usb2PPM.h"
#include "SerialClass.h"
#include "StdAfx.h"
#include <algorithm>


extern CTRL_Input g_XPCommand;
//extern XPState g_XPState;

//extern CTRL_Input g_XPCommand2;
//extern XPState g_XPState2;

extern HANDLE g_XCommandMutex;
//extern HANDLE g_XStateMutex;
//extern HANDLE g_XCommandMutex2;
//extern HANDLE g_XStateMutex2;

Serial::Serial(char *portName)
{
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
			//Define serial connection parameters for the arduino board
			dcbSerialParams.BaudRate = CBR_115200;
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
	}

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


char* encode_PPM(int cmd_ch1, int cmd_ch2, int cmd_ch3, int cmd_ch4){
	
	int cmd_ch[4] = { 0, 0, 0, 0}; //Default values for each channels
	char output_string[26]="\0";
	char* output_string_p = output_string;
	rtrn = (char*)malloc(26);

	char const_ch[4] = { 'x','y', 'z', 'k' };

	cmd_ch[0] = cmd_ch1;
	cmd_ch[1] = cmd_ch2;
	cmd_ch[2] = cmd_ch3;
	cmd_ch[3] = cmd_ch4;
	
	int j = 0;
	
	//for loop for each channel value processing
	for (int i = 0; i < 4; i++){
		//Set maximum values
		if (cmd_ch[i] > 1023){
			cmd_ch[i] = 1023;
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
	
	SerialCom *pSCom = (SerialCom *)lpParam;

	CTRL_Input cur_XPCommand = {0};

	unsigned short crc = 0;
	short *ptr = NULL;
	unsigned short length = 0;

	unsigned short *ptr_p = NULL;
	unsigned short packets = 0x0400;
	
	while(true)
	{

		// Copy the global XCommand
		WaitForSingleObject(g_XCommandMutex, INFINITE);
		memcpy(&cur_XPCommand, &g_XPCommand, sizeof(g_XPCommand));
		ReleaseMutex(g_XCommandMutex);
		
		DWORD dwNumBytesWritten;   
		DWORD dwHaveNumWritten =0 ; //已经写入多少

		PelicanComPacket xpacket;
		ptr = &(cur_XPCommand.pitch);
		memcpy(xpacket.startMark,">*>di",5);
		memcpy(&xpacket.command, ptr, sizeof(CTRL_Input));
		WriteFile(pSCom->hSerial,&xpacket,sizeof(xpacket),&dwNumBytesWritten,NULL);
		PurgeComm(pSCom->hSerial, PURGE_TXCLEAR );

		PurgeComm(pSCom->hSerial, PURGE_TXCLEAR );
		Sleep(40);
	}
	return iResult;
}
