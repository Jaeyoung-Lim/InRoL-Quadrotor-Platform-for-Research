////////////////////////////////////////////////////////////
// Extract trajectory works                             //
// BLUE                                                 //
////////////////////////////////////////////////////////////

#include "stdafx.h"
#include <iostream>
#include <stdio.h>
#include "DataStructure.h"
#include "Vector3.h"
#include "Desired.h"
#include "Matrix33.h"
#include <Windows.h>
#include <string>
#include "SerialCom.h"
#include "Client.h"
#include "Tracker.h"
#include "Omega.h"
#include <fstream>
#include <vector>
#include <conio.h>
#include "UDP.h"

using namespace std;


/////////Global DATA////////////
//States...Xside
//XCommand g_XCommand; // hummingbird
//XState g_XState; // hummingbird
CTRL_Input g_XPCommand;
//CTRL_Input g_XPCommand2;
//XPState g_XPState;
//XPState g_XPState2;

XConfig g_XConfig;
XStatus g_XStatus;
//States...Tracker side
TState g_TState;
//TState g_TState2;

//States...Ptam side
/*
PState g_PState;
int signal;
Vector3 x_ps;
Vector3 x_v0;
Matrix33 R_v0;
*/
//States...Haptic side
/*
HCommand g_HCommand;
HState g_HState;
*/

//All in Client.h
//Mutex...
HANDLE g_XCommandMutex = NULL;
HANDLE g_XStateMutex = NULL;
//HANDLE g_XCommandMutex2 = NULL;
//HANDLE g_XStateMutex2 = NULL;
//HANDLE g_XConfigMutex;
//HANDLE g_XStatusMutex;

//HANDLE g_HCommandMutex;
//HANDLE g_HStateMutex;
HANDLE g_TStateMutex;
//HANDLE g_TStateMutex2;

//HANDLE g_PStateMutex;

//motion scale
long long initialTick;
double eta = 3.0+2.0+2.0-1.0; //v = haptic_x*eta
Vector3 e_3(0.0, 0.0, 1.0);
int index = 1;// hovering
int mode = 2;// 1: teleoperation, 2: trajectory tracking, 3: velocity field control
bool PTAM_FLY = false;
double time0=0.0;
Vector3 Force;
Vector3 Force_env;
Vector3 acc_l;
double TDelay=0.0;
unsigned long long C_loss=0;
unsigned long long C_data=0;

double PI = 3.1415926535897932384626433832795028841971;
double m = 0.8+0.1, m2 = 0.8+0.1;//in kg, measured by balance 19oz*28.35
double g = 9.80665; // 1350 is the thrust that the flyer take off;
static double lambda = m*g, lambda2 = m2*g;
static double w1=0.0, w2=0.0, w21=0.0, w22=0.0;

ofstream outFile1("E:\\DHLee/data/STATE.txt");//path should be modified
ofstream outFile2("E:\\DHLee/data/ERROR.txt");//path should be modified
ofstream outFile3("E:\\DHLee/data/FORCE.txt");//path should be modified
ofstream outFile4("E:\\DHLee/data/SPEED.txt");//path should be modified
ofstream outFile5("E:\\DHLee/data/RAW.txt");//path should be modified

SOCKET   ServerSocket;
SOCKADDR_IN  server_addr;
SOCKADDR_IN  client_addr;


////////MainThreadDeclear////
int HXControl(void);
int HXTest(void);

int _tmain(int argc, _TCHAR* argv[]) //This Control the All the states' Communication
{
	int ret = 0;
	// About the global Datas
	SECURITY_ATTRIBUTES    sec_attr;
	
	sec_attr.nLength              = sizeof( SECURITY_ATTRIBUTES );
	sec_attr.lpSecurityDescriptor = NULL;
	sec_attr.bInheritHandle       = TRUE;
	// MutexCreated
	g_XCommandMutex = CreateMutex(&sec_attr, NULL, _T("mutex for Xcommand"));
	//g_XCommandMutex2 = CreateMutex(&sec_attr, NULL, _T("mutex for Xcommand2"));
	g_XStateMutex = CreateMutex(&sec_attr, NULL, _T("mutex for XState"));
	//g_XStateMutex2 = CreateMutex(&sec_attr, NULL, _T("mutex for XState2"));

	//g_HCommandMutex = CreateMutex(&sec_attr, NULL, _T("mutex for HCommand"));
	//g_HStateMutex = CreateMutex(&sec_attr, NULL, _T("mutex for HState"));

	g_TStateMutex = CreateMutex(&sec_attr, NULL, _T("mutex for TState"));
	//g_TStateMutex2 = CreateMutex(&sec_attr, NULL, _T("mutex for TState2"));

	//g_PStateMutex = CreateMutex(&sec_attr, NULL, _T("mutex for PState"));


	// UDP Side Established
	UDP udp;
	ret += udp.Ini();

	// Serial Port Side Established
	SerialCom SCom;
	ret += SCom.Ini();


	// Haptic Device Side Established
	Omega ome;
	if (mode==1)
		ret += ome.Ini();
	
	// Tracker Polling Established
	Tracker vicon; //updateRate 100
	ret += vicon.Ini();


	// Ini the rotation matrix
	g_TState.RotationMatrix[0] =1.0;
	g_TState.RotationMatrix[4] =1.0; 
	g_TState.RotationMatrix[8] =1.0; 

	Sleep(1000);
///////////////////////////////////////////////Here is CONTROLLER////////////////////////////////////////////////////
	
	while(true)
	{
		HXControl();
		Sleep(1); // Synchronizing : '1' for haptic, '10' for vicon
	}
	getchar();
	outFile1.close();
	outFile2.close();
	outFile3.close();
	outFile4.close();
	outFile5.close();
	return ret;
}

int HXControl(void)
{
	int iResult = 0;
	
	
	//XPState cur_XPState;
	//HState cur_HState;
	TState cur_TState;
	//TState cur_TState2;
	//PState cur_PState;
	
	CTRL_Input cur_XPCommand;
	//CTRL_Input cur_XPCommand2;w1

	//HCommand cur_HCommand;

////////////////////////////////Main Loop T/////////////////////////// //////
	static LARGE_INTEGER freq;
	QueryPerformanceFrequency(&freq);
	long long cpuFreq = freq.QuadPart;

	static LARGE_INTEGER l;
	static LARGE_INTEGER llast = {0};
	static bool firstRun = true;
	QueryPerformanceCounter(&l);
	if(firstRun == true) // Ini Last time
	{
		initialTick = l.QuadPart;
		llast.QuadPart = l.QuadPart;
		firstRun = false;
		return iResult;
	}
	double T = (double)((l.QuadPart)-(llast.QuadPart))/cpuFreq;
	llast.QuadPart = l.QuadPart;
	double time = (double)(l.QuadPart - initialTick)/cpuFreq;
	
////////////////////////////////////////Here begin the state polling from Haptic(virtual mass and object), Flyer, Tracker
	//Get all the States
	/*
	WaitForSingleObject(g_XStateMutex,INFINITE);
	memcpy(&cur_XPState, &g_XPState, sizeof(XPState));
	ReleaseMutex(g_XStateMutex);

	WaitForSingleObject(g_HStateMutex,INFINITE);
	memcpy(&cur_HState, &g_HState, sizeof(HState));
	ReleaseMutex(g_HStateMutex);
	*/

	// get parameters from Vicon
	WaitForSingleObject(g_TStateMutex,INFINITE);
	memcpy(&cur_TState, &g_TState, sizeof(TState));
	ReleaseMutex(g_TStateMutex);

	/*
	WaitForSingleObject(g_TStateMutex2,INFINITE);
	memcpy(&cur_TState2, &g_TState2, sizeof(TState));
	ReleaseMutex(g_TStateMutex2);

	WaitForSingleObject(g_PStateMutex,INFINITE);
	memcpy(&cur_PState, &g_PState, sizeof(PState));
	ReleaseMutex(g_PStateMutex);
	*/
	// Get position, velocity,Rotation matrix from Tracker

	Vector3 x(cur_TState.Translation[0]/1000.0, cur_TState.Translation[1]/1000.0, cur_TState.Translation[2]/1000.0); //in m
	Vector3 dx(cur_TState.Velocity[0]/1000.0, cur_TState.Velocity[1]/1000.0, cur_TState.Velocity[2]/1000.0); //in m/s
	Vector3 ddx;
	Matrix33 R(cur_TState.RotationMatrix);// R is from Body frame to Global Frame

	/*
	Vector3 x2(cur_PState.Translation[0], cur_PState.Translation[1], cur_PState.Translation[2]); //in m
	Vector3 dx2(cur_PState.Velocity[0], cur_PState.Velocity[1], cur_PState.Velocity[2]); //in m/s
	Vector3 ddx2;
	Matrix33 R2;// R is from Body frame to Global Frame //changed!! we will use rotation from LLP
	*/
	//Make signal for PTAM
	/*
	if(signal==1)
	{
		if((x-x_ps).Magnitude()>0.10)
		{
			x_v0=x;
			R_v0=R;
			signal=2;
			std::cout<<"PTAM starts at"<<x_v0.x<<" "<<x_v0.y<<" "<<x_v0.z<<std::endl;
		}
		else	signal=1;
	}
	*/
	//Get pose from Flyer
	/*
	double po = cur_XPState.angle_nick/1000.0;
	double ro = cur_XPState.angle_roll/1000.0;
	double wo = cur_XPState.angle_yaw/1000.0+0.0;//check!!!!!!!!!!!!!!!!!!
	*/
//	std::cout << ro << "	" << po << "	" << wo << std::endl;


	po = po*PI/180.0;
	ro = ro*PI/180.0;
	wo = wo*PI/180.0;
	if(wo>PI)
		wo=wo-2*PI;

	double Rr[9] = {1, 0, 0,
		0, cos(ro), -sin(ro),
		0, sin(ro), cos(ro)};
	double Rp[9] = {cos(po), 0, sin(po),
		0, 1, 0,
		-sin(po), 0, cos(po)};
	double Ry[9] = {cos(wo), -sin(wo), 0,
		sin(wo), cos(wo), 0,
		0, 0, 1};
	Matrix33 RR(Rr);
	Matrix33 RP(Rp);
	Matrix33 RY(Ry);

	R2=RY*RP*RP;
	R2=R;
	/*
	//Get Acceleration from Flyer
	double acc_x = cur_XPState.acc_x/10000.0*g;
	double acc_y = cur_XPState.acc_y/10000.0*g;
	double acc_z = cur_XPState.acc_z/10000.0*g;
	Vector3 ACC_IMU(acc_x, acc_y, acc_z);
	ACC_IMU = R2*ACC_IMU;
	//	LOW PASS FILTER
	Vector3 L_acc = ACC_IMU;
	static Vector3 acc_fil;// = L_acc;
	double fil_alph = 0.05;
	acc_fil = L_acc*fil_alph+acc_fil*(1-fil_alph);

	*/
	//Get Acceleration from Dynamics
	static double m_hat = 0.55+0.25; // initial estimated mass value
	static double m_hat2 = 0.55+0.25; // initial estimated mass value
	Vector3 Dyn=R*e_3*lambda*(-1.0)/m_hat+e_3*g; // dynamics	
	Vector3 Dyn2=R2*e_3*lambda2*(-1.0)/m_hat2+e_3*g; // dynamics	

	//!!! Choose the rotation matrix!!!//
	R=R2;// IT'S OK TO USE POSE FROM FLYER ALWAYS!!!!

	//!!! Choose the acceleration!!!//
	ddx=Dyn;
	ddx2=Dyn2;
//	ddx=acc_fil;// IT'S OK TO USE ACC FROM FLYER ALWAYS!!!!
//	ddx2=acc_fil;



	/*
	//raw data zone
	double a_x = cur_XPState.acc_x_calib / 10000.0*g;
	double a_y = cur_XPState.acc_y_calib / 10000.0*g;
	double a_z = cur_XPState.acc_z_calib / 10000.0*g;
	double a_t = cur_XPState.acc_absolute_value / 10000.0*g;

	int w_x = cur_XPState.angvel_nick;
	int w_y = cur_XPState.angvel_roll;
	int w_z = cur_XPState.angvel_yaw;

	double h    = cur_XPState.height / 1000.0;
	double dh   = cur_XPState.dheight / 1000.0;
	double h_r  = cur_XPState.height_reference / 1000.0;
	double dh_r = cur_XPState.dheight_reference / 1000.0;

	int speed_x = cur_XPState.speed_x;
	int speed_y = cur_XPState.speed_y;
	int speed_z = cur_XPState.speed_z;

	double yaw_c = cur_XPState.mag_heading / 1000.0;
	int Hx = cur_XPState.Hx;
	int Hy = cur_XPState.Hy;
	int Hz = cur_XPState.Hz;
	*/


	Vector3 xd, xd2;
	Vector3 dxd, dxd2;
	Vector3 ddxd, ddxd2;
	Vector3 dddxd, dddxd2;	
	
	//Desired trajectories are in Desired.cpp
	Desired traj(mode, index, time, time0);
	xd = traj.x;	dxd = traj.v;	ddxd = traj.a;	dddxd = traj.da;
	xd2 = xd;	dxd2 = dxd;	ddxd2 = ddxd;	dddxd2 = dddxd;

	double k = 4.0;//3.0;  lag wormming<<<<0.5<<<<occilating //k=4.0 seems to be best for IMU-Vicon 15.02.27
	double b = sqrt(0.1*m*k);//			  sqrt(4*m*k);//sqrt(0.1*m*k)seems to be best for IMU-Vicon 15.02.27
	double alpha = 100.0;//						 50<<<<steady//100.0 seems to be best for IMU-Vicon 15.02.27
	double epsi = b/(12*m);//b/(8*m);responsive<<<8<<<steady
	double r1 = 140.0, r2=0.001;

	//1st UAV
	Vector3 e=x-xd;
	Vector3 de=dx-dxd;

	double dm_hat = r2*((de*g+e*epsi*g).Dot(e_3)+(de+e*epsi).Dot(ddxd)*(-1.0));
	Vector3 dv = dddxd*m_hat + ddxd*dm_hat + (ddx-ddxd)*b*(-1.0) + (dx - dxd)*k*(-1.0);
	Vector3 v = ddxd*m_hat + (dx-dxd)*b*(-1.0) + (x-xd)*k*(-1.0);
	Vector3 Control = (dv+e_3*g*dm_hat*(-1.0)) + (v + e_3*g*m_hat*(-1.0))*alpha-((dx-dxd) + (x-xd)*epsi)*r1;
		
	Vector3 Rtv = R.Trans()*Control*(-1.0);
	w1 = -Rtv.y/lambda;
	w2 = Rtv.x/lambda;
	double dlambda = Rtv.z - alpha*lambda;
	lambda = max(lambda + T*dlambda,1.0); //avoid being zero
	m_hat = m_hat+dm_hat*T;
	/*
	//2nd UAV
	Vector3 e2=x2-xd2;
	Vector3 de2=dx2-dxd2;

	double dm_hat2 = r2*((de2*g+e2*epsi*g).Dot(e_3)+(de2+e2*epsi).Dot(ddxd2)*(-1.0));
	Vector3 dv2 = dddxd2*m_hat2 + ddxd2*dm_hat2 + (ddx2-ddxd2)*b*(-1.0) + (dx2 - dxd2)*k*(-1.0);
	Vector3 v2 = ddxd2*m_hat2 + (dx2-dxd2)*b*(-1.0) + (x2-xd2)*k*(-1.0);
	Vector3 Control2 = (dv2+e_3*g*dm_hat2*(-1.0)) + (v2 + e_3*g*m_hat2*(-1.0))*alpha-((dx2-dxd2) + (x2-xd2)*epsi)*r1;

	Vector3 Rtv2 = R2.Trans()*Control2*(-1.0);
	w21 = -Rtv2.y/lambda2;
	w22 = Rtv2.x/lambda2;
	double dlambda2 = Rtv2.z - alpha*lambda2;
	lambda2 = max(lambda2 + T*dlambda2,1.0); //avoid being zero
	m_hat2 = m_hat2+dm_hat2*T;
	*/		

	double thrustValue;
	double polya = 0.000001301; 
	double polyb = 0.0011; 
	double polyc = 0.3604 - lambda;
	double newton2value = (-polyb+sqrt(polyb*polyb-4*polya*polyc))/(2*polya);
	thrustValue = newton2value;//193.3*lambda+511.5;//
	
	/*
	double thrustValue2;
	double polya2 = 0.000001301; 
	double polyb2 = 0.0011; 
	double polyc2 = 0.3604 - lambda2;
	double newton2value2 = (-polyb2+sqrt(polyb2*polyb2-4*polya2*polyc2))/(2*polya2);
	thrustValue2 = newton2value2;//193.3*lambda+511.5;//
	*/

	//keyboard setting!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	if( _kbhit() )
		{
			char input = _getch();
			if (input == 'q')
			{
				cout << "\nExit";
				Sleep(50);
				exit(9);
			}

			else if( input == 'a')
			{
				mode=2;	index=1;
			}
			else if( input == 'b')
			{
				mode=2;	index=2;
				time0 = time;
			}
			else if( input == 'c')
			{
				mode=2;	index=3;
			}
			else if( input == 'd')
			{
				mode=2;	index=4;
				time0=time;
			}
			else if( input == 'e')
			{
				mode=2;	index=5;
				time0=time;
			}
			else if( input == 'p')
			{
				signal=1;
				x_ps=x;
			}
			else if( input == 'o')
			{
				PTAM_FLY=!PTAM_FLY;
				m_hat2=m_hat;
			}
			else if( input == 's')
			{
				mode=2;	index=3;
			}

			else if( input == '1')
			{
				mode=3;	index=1;
			}
			else if( input == '2')
			{
				mode=3;	index=2;
			}
			else if( input == '3')
			{
				mode=3;	index=3;
			}
			else if( input == '4')
			{
				mode=3;	index=4;
			}
			else if( input == '5')
			{
				mode=3;	index=5;
			}
			else if( input == '6')
			{
				mode=3;	index=6;
			}
			else if( input == '7')
			{
				mode=3;	index=7;
			}
			else if( input == '8')
			{
				mode=3;	index=8;
			}
			else if( input == '9')
			{
				mode=3;	index=9;
			}
	
	
	}

	double compen_w1 = 30.0*0.0;//20.0+5.0+10.0+5.0;//-20;// for roll       //Blue : 0   //
	double compen_w2 = -300.0*0.0;//-70.0-30.0-30.0;// for pitch  //Blue : -30 //
	double compen_w3 = -2000.0*(cur_TState.RotationEuler[2]-0.778);//0.0;+100;// for yaw		//Blue : 100 //
//	std::cout<< cur_TState.RotationEuler[2] << std::endl;

	double compen_w21 = 30.0*0.0;//20.0+5.0+10.0+5.0;//-20;// for roll       //Blue : 0   //
	double compen_w22 = -300.0*0.0;//-70.0-30.0-30.0;// for pitch  //Blue : -30 //
	double compen_w23 = 0.0;//+100;// for yaw		//Blue : 100 //
	
	double yawspeed = 2048 + compen_w3;// + 3*yawangle;
	double yawspeed2 = 2048 + compen_w3;// + 3*yawangle;

//*************************************************Translate all to Flyer's Value**********************************************************
		
		cur_XPCommand.thrust = max(min(thrustValue,2500),0); 
		cur_XPCommand.pitch = min(max(compen_w2 +11.7*w2,-2048),2048); // minus is positive direction   
		cur_XPCommand.roll = min(max(compen_w1 +11.4*w1,-2048),2048); // minus is positive direction   
		cur_XPCommand.yaw= yawspeed-2048;// min(max(2048 + compen_w3 -11.4*w3,0),4096);//yawspeed;  // positve
		cur_XPCommand.ctrl=1+2+4+8;

		cur_XPCommand2.thrust = max(min(thrustValue2,2500),0); 
		cur_XPCommand2.pitch = min(max(compen_w22 +11.7*w22,-2048),2048); // minus is positive direction   
		cur_XPCommand2.roll = min(max(compen_w21 +11.4*w21,-2048),2048); // minus is positive direction   
		cur_XPCommand2.yaw= yawspeed2-2048;// min(max(2048 + compen_w3 -11.4*w3,0),4096);//yawspeed;  // positve
		cur_XPCommand2.ctrl=1+2+4+8;

// trial : change command position data based on vicon -> ptam
		if(PTAM_FLY)
		{
			cur_XPCommand.thrust = max(min(thrustValue2,2400),0); 
			cur_XPCommand.pitch = min(max(compen_w22 +11.7*w22,-2048),2048); // minus is positive direction   
			cur_XPCommand.roll = min(max(compen_w21 +11.4*w21,-2048),2048); // minus is positive direction   
			cur_XPCommand.yaw= yawspeed2-2048;// min(max(2048 + compen_w3 -11.4*w3,0),4096);//yawspeed;  // positve
			std::cout<<"PTAM MODE"<<std::endl;
		}


		cur_XPCommand.chksum=cur_XPCommand.pitch+cur_XPCommand.roll+cur_XPCommand.yaw+cur_XPCommand.thrust+cur_XPCommand.ctrl+0xAAAA;
		cur_XPCommand2.chksum=cur_XPCommand2.pitch+cur_XPCommand2.roll+cur_XPCommand2.yaw+cur_XPCommand2.thrust+cur_XPCommand2.ctrl+0xAAAA;



//*************************************** Here POST the command to haptic device and flyer

	WaitForSingleObject(g_XCommandMutex,INFINITE);
	memcpy(&g_XPCommand, &cur_XPCommand, sizeof(CTRL_Input));
	ReleaseMutex(g_XCommandMutex);

	/*
	WaitForSingleObject(g_XCommandMutex2,INFINITE);
	memcpy(&g_XPCommand2, &cur_XPCommand2, sizeof(CTRL_Input));
	ReleaseMutex(g_XCommandMutex2);

	cur_HCommand.xset = dx.x;
	cur_HCommand.yset = dx.y;
	cur_HCommand.zset = dx.z;	
	
	cur_HCommand.seqNo = cur_TState.seqNo;
	*/
	// Send Haptic Command
	/*
	WaitForSingleObject(g_HCommandMutex,INFINITE);
	memcpy(&g_HCommand, &cur_HCommand, sizeof(HCommand));
	ReleaseMutex(g_HCommandMutex);
	*/
	//**********************************************CONFIRM ZONE**********************************************************//
	//logging

	return iResult;
}