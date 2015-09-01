#include "stdafx.h"
#include "UDP.h"
#include <algorithm>
#include "Matrix33.h"
#include "Vector3.h"

#pragma comment (lib,"ws2_32.lib")

extern HANDLE g_PStateMutex;
extern PState g_PState;
extern int signal;
extern Vector3 x_v0;
extern Matrix33 R_v0;

extern SOCKET   ServerSocket;
extern SOCKADDR_IN  server_addr;
extern SOCKADDR_IN  client_addr;


DWORD WINAPI PtamPollingThreadFunc (LPVOID lpParam);
long long iniTick;

UDP::UDP(void)
{	
	printf("UDP start\n");
	hPtamPollingThread = INVALID_HANDLE_VALUE;

	LARGE_INTEGER freq;
	QueryPerformanceFrequency(&freq);
	cpuFreq = freq.QuadPart;

}

UDP::~UDP(void)
{
	this->Close();
}

int UDP::Ini(void)
{
	int iResult = 0;

	WSADATA   wsaData;

	if (WSAStartup(0x202, &wsaData) == SOCKET_ERROR)
	{
		printf("WinSock Error.n");
		WSACleanup();
		iResult = 1;
		return iResult;
	}

	memset(&server_addr, 0, sizeof(server_addr));
	memset(&client_addr, 0, sizeof(client_addr));

	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.s_addr = inet_addr("147.46.175.54");	// IP
	server_addr.sin_port = htons(6962);							// port

	//####################################### create #######################################
	ServerSocket = socket(AF_INET, SOCK_DGRAM, 0); // make udp socket

	if (ServerSocket == INVALID_SOCKET)
	{
		printf("Can't build socket.");
		closesocket(ServerSocket);
		WSACleanup();
		exit(0);
		iResult = 1;
		return iResult;
	}

	//##################################### bind ###########################################
	if (bind(ServerSocket, (struct sockaddr*)&server_addr, //BIND
		sizeof(server_addr)) == SOCKET_ERROR)
	{
		printf("Can't bind.");
		closesocket(ServerSocket);
		WSACleanup();
		exit(0);
		iResult = 1;
		return iResult;
	}

	printf("waiting...\n");

	// Here Start the Polling loop
	hPtamPollingThread = CreateThread(NULL, 0, PtamPollingThreadFunc, this, 0, NULL);

	return iResult;
}

int UDP::Close(void)
{
	int iResult = 0;
	//	closesocket(ServerSocket);//close socket
	WSACleanup();

	if (hPtamPollingThread != INVALID_HANDLE_VALUE)
		CloseHandle(hPtamPollingThread);

	return iResult;
}

DWORD WINAPI PtamPollingThreadFunc (LPVOID lpParam)  // The rate is about 25Hz
{
	UDP *pUDP = (UDP *)lpParam;
	PState last_PState;

	int   Recv_size;
	int   Send_size;
	int   SeqNo = 0;
	int   prev_SeqNo = 0;

	int   client_addr_size=sizeof(client_addr);

	while(true)
	{
		PState cur_PState;
		UState cur_UState;

		static double xlast[3] = {0};
		static double vlast[3] = {0};
		static double tlast = 0;
		static int x_p0[3] = { 0 };
		static Matrix33 R_p0;
		static int ptam_start=0;

		// get UState from ptam
		Recv_size = recvfrom(ServerSocket, (char *)&cur_UState, sizeof(cur_UState), 0, (struct sockaddr*) &client_addr, &client_addr_size);
		if (Recv_size<0){ printf("recvfrom() error! \n"); }
//		printf("Cient IP < %s >\n", inet_ntoa(client_addr.sin_addr));

		cur_UState.signal=signal;

		// send signal to ptam
		Send_size=sendto(ServerSocket, (char *)&cur_UState, sizeof(cur_UState), 0, (struct sockaddr*) &client_addr, client_addr_size);
		if (Send_size<0){ printf("sendto() error! \n"); }

		//Time checker
		static LARGE_INTEGER l;
		static LARGE_INTEGER llast = {0};
		static bool firstRun = true;
		QueryPerformanceCounter(&l);
		if (firstRun == true) // Ini Last time
		{
			iniTick = l.QuadPart;
			llast.QuadPart = l.QuadPart;
			firstRun = false;
		}
		cur_PState.seqNo = l.QuadPart;
		double freq = (double)pUDP->cpuFreq;

		double T = (double)((l.QuadPart) - (llast.QuadPart)) / freq;
		llast.QuadPart = l.QuadPart;
		double time = (double)(l.QuadPart - iniTick) / freq;

		//1,000,000 times rotation matrix
		cur_PState.RotationMatrix[0] = (float)cur_UState.r11 / 1000000;
		cur_PState.RotationMatrix[1] = (float)cur_UState.r12 / 1000000;
		cur_PState.RotationMatrix[2] = (float)cur_UState.r13 / 1000000;
		cur_PState.RotationMatrix[3] = (float)cur_UState.r21 / 1000000;
		cur_PState.RotationMatrix[4] = -(float)cur_UState.r22 / 1000000;
		cur_PState.RotationMatrix[5] = -(float)cur_UState.r23 / 1000000;
		cur_PState.RotationMatrix[6] = (float)cur_UState.r31 / 1000000;
		cur_PState.RotationMatrix[7] = -(float)cur_UState.r32 / 1000000;
		cur_PState.RotationMatrix[8] = -(float)cur_UState.r33 / 1000000;
		if (ptam_start==0&&cur_UState.z>0)
		{
			x_p0[0]=-cur_UState.x;
			x_p0[1]=cur_UState.y;
			x_p0[2]=-cur_UState.z;
			Matrix33 Rp_dum(cur_PState.RotationMatrix);
			R_p0= Rp_dum;
		}
		ptam_start=cur_UState.z;

		//	std::cout<<cur_UState.z<<std::endl;
		//	std::cout<<x_p0[0]<<" "<<x_p0[1]<<" "<<x_p0[2]<<" "<<std::endl;
		// I just changed two things(y -> -y && R_p0.Trans()*R_v0-> R_v0*R_p0.Trans()

		// mm to m
		Vector3 xp_dum( (double)(-cur_UState.x-x_p0[0])/1000, (double)(cur_UState.y-x_p0[1])/1000, (double)(-cur_UState.z-x_p0[2])/1000);
		//	std::cout<<xp_dum.x<<" "<<xp_dum.y<<" "<<xp_dum.z<<std::endl;
		//	xp_dum=x_v0+R_p0.Trans()*R_v0*xp_dum;//(changed)
		xp_dum=x_v0+xp_dum;
//		xp_dum=x_v0+R_v0*R_p0.Trans()*xp_dum;
		//	std::cout<<xp_dum.x<<" "<<xp_dum.y<<" "<<xp_dum.z<<std::endl;

		cur_PState.Translation[0] = xp_dum.x;
		cur_PState.Translation[1] = xp_dum.y;
		cur_PState.Translation[2] = xp_dum.z;


		//	std::cout<<cur_PState.Translation[0]<<" "<<cur_PState.Translation[1]<<" "<<cur_PState.Translation[2]<<std::endl;
		//	std::cout<<cur_UState.signal<<std::endl;

		//velocity
		if ((cur_PState.Translation[0] - xlast[0])*(cur_PState.Translation[0] - xlast[0]) + (cur_PState.Translation[1] - xlast[1])*(cur_PState.Translation[1] - xlast[1]) + (cur_PState.Translation[2] - xlast[2])*(cur_PState.Translation[2] - xlast[2]) > 0)
		{
			//update velocity with LLP
			double Wlast = 0.2;//0.2 seems to be best 15.02.27
			vlast[0] = (1.0 - Wlast)*(cur_PState.Translation[0] - xlast[0]) / (time - tlast) + Wlast*vlast[0];
			vlast[1] = (1.0 - Wlast)*(cur_PState.Translation[1] - xlast[1]) / (time - tlast) + Wlast*vlast[1];
			vlast[2] = (1.0 - Wlast)*(cur_PState.Translation[2] - xlast[2]) / (time - tlast) + Wlast*vlast[2];

			tlast = time;
			memcpy(xlast, cur_PState.Translation, sizeof(xlast));
		}
		memcpy(cur_PState.Velocity, vlast, sizeof(vlast));

		WaitForSingleObject(g_PStateMutex, INFINITE);
		memcpy(&g_PState, &cur_PState, sizeof(PState));
		ReleaseMutex(g_PStateMutex);
	}

}