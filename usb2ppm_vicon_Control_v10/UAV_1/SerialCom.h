#pragma once

#include <Windows.h>
#include "DataStructure.h"
#include <assert.h>
#include <deque>

class SerialCom
{
public:
	SerialCom(void);
	~SerialCom(void);
public:
	int Ini(void);
	int Close(void);
public:
	HANDLE hSerial;
	HANDLE hSerial2;
private:
	HANDLE hWriteThread, hReadThread;
	HANDLE hWriteCloseEvent, hReadCloseEvent;
	HANDLE hWriteThread2, hReadThread2;
	HANDLE hWriteCloseEvent2, hReadCloseEvent2;
public:
	std::deque<XCommand_dq_pack> dq_XCommand;

};
