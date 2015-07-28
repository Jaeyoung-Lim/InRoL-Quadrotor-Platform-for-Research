#include "StdAfx.h"
#include "SerialCom.h"
#include <algorithm>

extern CTRL_Input g_XPCommand;
extern XPState g_XPState;

extern CTRL_Input g_XPCommand2;
extern XPState g_XPState2;

extern HANDLE g_XCommandMutex;
extern HANDLE g_XStateMutex;
extern HANDLE g_XCommandMutex2;
extern HANDLE g_XStateMutex2;


DWORD WINAPI ReadThreadFunc(LPVOID);
DWORD WINAPI WriteThreadFunc(LPVOID);
DWORD WINAPI ReadThreadFunc2(LPVOID);
DWORD WINAPI WriteThreadFunc2(LPVOID);


unsigned short crc_update(unsigned short crc,unsigned char data);
unsigned short crc16(void* data, unsigned short cnt);

SerialCom::SerialCom(void)
{	
	hSerial =  INVALID_HANDLE_VALUE;
	//hSerial2 =  INVALID_HANDLE_VALUE;
}

SerialCom::~SerialCom(void)
{
	this->Close();
}

int SerialCom::Ini(void)
{
	int iResult = 0;
	
// Connect the Serial Port
	//1st UAV
	if(hSerial != INVALID_HANDLE_VALUE)	{	return iResult;	}
	hSerial = CreateFile("COM3",
		GENERIC_READ | GENERIC_WRITE,
		0,
		NULL,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL,
		NULL);
	std::cout << "A" <<std::endl;
	if(hSerial==INVALID_HANDLE_VALUE){
		if(GetLastError()==ERROR_FILE_NOT_FOUND)
		{
			printf("Error at open COM port.\n");
			iResult = 1;
			return iResult;
		}
	}
	
	DCB dcbSerialParams = {0};
	dcbSerialParams.DCBlength=sizeof(dcbSerialParams);
	if (!GetCommState(hSerial, &dcbSerialParams)) {
		//error getting state
	}
	dcbSerialParams.BaudRate=57600;
	dcbSerialParams.ByteSize=8;
	dcbSerialParams.StopBits=ONESTOPBIT;
	dcbSerialParams.Parity=NOPARITY;
	
	if(!SetCommState(hSerial, &dcbSerialParams))
		printf("Error at setting the Com port");


	COMMTIMEOUTS timeouts={0};
	timeouts.ReadIntervalTimeout=MAXDWORD;
	timeouts.ReadTotalTimeoutConstant=0;
	timeouts.ReadTotalTimeoutMultiplier=0;
	timeouts.WriteTotalTimeoutConstant=50;
	timeouts.WriteTotalTimeoutMultiplier=10;
	if(!SetCommTimeouts(hSerial, &timeouts))
		printf("Error at setting ComTimeOut");

	SetCommMask (hSerial, EV_RXCHAR );  
	SetupComm(hSerial,512,512);    
	PurgeComm(hSerial,PURGE_TXCLEAR|PURGE_RXCLEAR);   

	hReadCloseEvent = CreateEvent(NULL,TRUE,FALSE,NULL);    
	
	hReadThread = CreateThread(NULL,0,ReadThreadFunc,this,0,NULL);    
	hWriteThread = CreateThread(NULL,0,WriteThreadFunc,this,0,NULL);  

		
	//////////////////////////////////////////////////////////////////

	printf("Serial Port Ini Seccess!\n");

	return iResult;
}

int SerialCom::Close(void)
{
	int iResult = 0;
	if(hWriteThread != INVALID_HANDLE_VALUE)		CloseHandle(hWriteThread);
	if(hReadThread != INVALID_HANDLE_VALUE) 		CloseHandle(hReadThread);
	if(hWriteCloseEvent != INVALID_HANDLE_VALUE)	CloseHandle(hWriteCloseEvent);
	if(hReadCloseEvent != INVALID_HANDLE_VALUE)		CloseHandle(hReadCloseEvent);
	if(hSerial != INVALID_HANDLE_VALUE)				CloseHandle(hSerial);
	
	return iResult;
}

DWORD WINAPI ReadThreadFunc(LPVOID lpParam)
{
	int iResult = 0;

	SerialCom *pSCom = (SerialCom *)lpParam;

	XPState cur_XPState = {0};

	DWORD evtMask;
	BYTE *readBuf = NULL;
	DWORD actualReadLen = 0;
	DWORD willReadLen;

	DWORD dwReadErrors;
	COMSTAT cmState;

	PurgeComm(pSCom->hSerial, PURGE_RXCLEAR );
	SetCommMask (pSCom->hSerial, EV_RXCHAR | EV_CTS | EV_DSR );

	CTRL_Input cur_XPCommand = {0};

	unsigned short crc = 0;
	short *ptr = NULL;
	unsigned short length = 0;

	unsigned short *ptr_p = NULL;
	int i = 0;

	while(true)
	{
		//PurgeComm(pSCom->hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR );

		DWORD dwNumBytesWritten;   
		DWORD dwHaveNumWritten =0 ; //已经写入多少
		BYTE beginByte;

		PelicanPollPacket xPpacket;
		//memcpy(xPpacket.startMark,">*>p",4);	
		xPpacket.startMark[0] = '>';
		xPpacket.startMark[1] = '*';
		xPpacket.startMark[2] = '>';
		xPpacket.startMark[3] = 'p';
		xPpacket.packets=0x0004;
		WriteFile(pSCom->hSerial,&xPpacket,sizeof(xPpacket),&dwNumBytesWritten,NULL);		
		
		ReadFile(pSCom->hSerial, &beginByte, 1, &actualReadLen,0);		
		//printf("%c\n", beginByte); // check the first character : should be '>'. if not, check the sleep. Because the main loop is too fast, garbage might be included.
		if(beginByte == '>')
		{
			ReadFile(pSCom->hSerial, &beginByte, 1, &actualReadLen,0);
			if(beginByte == '*')
			{
				ReadFile(pSCom->hSerial, &beginByte, 1, &actualReadLen,0);
				if(beginByte == '>')
				{
					ReadFile(pSCom->hSerial, &beginByte, 1, &actualReadLen,0);
					if(beginByte == 0x5c)
					{
						ReadFile(pSCom->hSerial, &beginByte, 1, &actualReadLen,0); //0x00
						ReadFile(pSCom->hSerial, &beginByte, 1, &actualReadLen,0);
						if(beginByte == 0x03)
						{
							unsigned char data[92];
							ReadFile(pSCom->hSerial, &data, 92, &actualReadLen,0);
							XPState *cur_XPState = (XPState *)data;
						
							unsigned char crc[2];
							unsigned char stop[3];
							ReadFile(pSCom->hSerial, &crc, 2, &actualReadLen,0);
							ReadFile(pSCom->hSerial, &stop, 3, &actualReadLen,0);

							WaitForSingleObject(g_XStateMutex,INFINITE);
							memcpy(&g_XPState,cur_XPState,sizeof(XPState));
							ReleaseMutex(g_XStateMutex);
						}
			
					}

				}
			}
		}

		PurgeComm(pSCom->hSerial, PURGE_RXCLEAR );
		Sleep(40);
	} // the end of while loop

				
	return iResult;
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

//cyclic redundant check
unsigned short crc_update(unsigned short crc,unsigned char data)
{
	data ^= (crc & 0xff);
	data ^= data << 4;
	return ((((unsigned short )data << 8) | ((crc>>8)&0xff))
		^ (unsigned char )(data >> 4) ^ ((unsigned short )data << 3));
}
//calculate the CRC16 of an array of bytes with the length cnt
unsigned short crc16(void* data, unsigned short cnt)
{
	unsigned short crc=0xff;
	unsigned char * ptr=(unsigned char *) data;
	int i;
	for (i=0;i<cnt;i++)
	{
		crc=crc_update(crc,*ptr);
		ptr++;
	}
	return crc;
}
