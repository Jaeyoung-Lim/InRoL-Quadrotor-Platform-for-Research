#include <Windows.h>
#include "DataStructure.h"

class UDP
{
public:
	UDP(void);
	~UDP(void);
private:
	HANDLE hPtamPollingThread;
public:
	int Ini(void);
	int Close(void);
//private:
//	Client MyClient;
private:
//	int updateInterval;
	long long cpuFreq;

public:
	friend DWORD WINAPI PtamPollingThreadFunc (LPVOID lpParam);

};
