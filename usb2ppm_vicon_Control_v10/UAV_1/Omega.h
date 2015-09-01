#pragma once

#include <windows.h>
#include <iostream>
#include <deque>
#include "DataStructure.h"

class Omega
{
public:
	Omega(void);
	~Omega(void);
public:
	int Ini(void);
	int Close(void);
private:
	HANDLE hOmegaThread;
	HANDLE hOmegaCloseEvent;
	//HANDLE hCameraPollingThread;
public:
	float ii;
	float jj;
public:
	friend DWORD WINAPI OmegaThreadFunc (LPVOID lpParam);
};



