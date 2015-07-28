
#include "stdafx.h"
#include "Omega.h"
#include "DataStructure.h"
#include "dhdc.h"
#include "PSPM.h"
#include "Obstacle.h"
#include <algorithm> // for sort

extern HCommand g_HCommand;
extern HState g_HState;
extern HANDLE g_HCommandMutex;
extern HANDLE g_HStateMutex;
extern Vector3 Force;
extern Vector3 Force_env;
extern double eta;
long long initialTick_hd;

double del_L = 0.00/2.0; // dividing 2 is due to round-trip
double del_U = 0.00/2.0; // dividing 2 is due to round-trip
extern double TDelay;
extern unsigned long long C_loss;
extern unsigned long long C_data;

DWORD WINAPI OmegaThreadFunc (LPVOID lpParam);

Omega::Omega(void)
{	
	//hCameraPollingThread = INVALID_HANDLE_VALUE;
}

Omega::~Omega(void)
{
	dhdEnableForce (DHD_OFF);
	dhdClose ();
	this->Close();
}

int Omega::Ini(void)
{
	int iResult = 0;

	int major, minor, release, revision;
	dhdGetSDKVersion (&major, &minor, &release, &revision);
	printf ("\n");
	printf ("Force Dimension - Viscosity Example %d.%d.%d.%d\n", major, minor, release, revision);
	printf ("(C) 2011 Force Dimension\n");
	printf ("All Rights Reserved.\n\n");

  // required to change asynchronous operation mode
	dhdEnableExpertMode ();

  // open the first available device
	if (dhdOpen () < 0) {
		printf ("error: cannot open device (%s)\n", dhdErrorGetLastStr());
		dhdSleep (2.0);
		return -1;
	}

	dhdEnableForce (DHD_ON);

	hOmegaCloseEvent = CreateEvent(NULL,TRUE,FALSE,NULL); 
	hOmegaThread = CreateThread(NULL, 0, OmegaThreadFunc, this, 0, NULL);

	return iResult;
}

int Omega::Close(void)
{
	int iResult = 0;
	dhdEnableForce (DHD_OFF);
	dhdClose ();

	if(hOmegaThread != INVALID_HANDLE_VALUE)
	CloseHandle(hOmegaThread);
	if(hOmegaCloseEvent != INVALID_HANDLE_VALUE)
	CloseHandle(hOmegaCloseEvent);
	return iResult;
}

bool Less(Data item1, Data item2)
{
	return item1.time_to_go < item2.time_to_go;
}

DWORD WINAPI OmegaThreadFunc (LPVOID lpParam)  
{
	int iResult = 0;
    int done = 0;
	double fx, fy, fz;

	HState cur_HState;
	HCommand cur_HCommand; // definition
	Data MtoS; // to make delay for data form master to slave
	Data StoM; // to make delay for data form slave to master

	MtoS.i[0] = 0.0; MtoS.i[1] = 0.0; MtoS.i[2] = 0.0;
	MtoS.acquisition_time = 0.0;
	MtoS.Loss = 0;
	MtoS.time_to_go = 0.0;

	StoM.i[0] = 0.0; StoM.i[1] = 0.0; StoM.i[2] = 0.0;
	StoM.acquisition_time = 0.0;
	StoM.Loss = 0;
	StoM.time_to_go = 0.0;

	std::deque<Data> dq_Data1;
	std::deque<Data> dq_Data2;

	Packet_send packet_send1;
	packet_send1.data.i[0]=0.0; packet_send1.data.i[1]=0.0; packet_send1.data.i[2]=0.0;
	packet_send1.data.acquisition_time=0.0;
	packet_send1.data.time_to_go=0.0;
	packet_send1.data.Loss = 0;

	Packet_send packet_send2;
	packet_send2.data.i[0]=0.0; packet_send2.data.i[1]=0.0; packet_send2.data.i[2]=0.0;
	packet_send2.data.acquisition_time=0.0;
	packet_send2.data.time_to_go=0.0;
	packet_send2.data.Loss = 0;

	unsigned long long loss_check=0;
	unsigned long long data_check=0;

	while(!done)
	{

	// time acquisition
	//////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////

		LARGE_INTEGER freq;
		QueryPerformanceFrequency(&freq);
		long long cpuFreq = freq.QuadPart;

		static LARGE_INTEGER l;
		static LARGE_INTEGER l_cur;
		static LARGE_INTEGER llast = {0};
		static bool firstRun = true;
		QueryPerformanceCounter(&l);

		if(firstRun == true) // Ini Last time
		{
			llast.QuadPart = l.QuadPart;
			firstRun = false;
			initialTick_hd=l.QuadPart;
		}

		double T = (double)((l.QuadPart)-(llast.QuadPart))/cpuFreq;
		llast.QuadPart = l.QuadPart;
		double time = (double)(l.QuadPart - initialTick_hd)/cpuFreq;
	
	// data from the slave to the master
	//////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////
		
		WaitForSingleObject(g_HCommandMutex,INFINITE);
		memcpy(&cur_HCommand, &g_HCommand, sizeof(HCommand)); // cur_Hcommand
		ReleaseMutex(g_HCommandMutex);
		
		static Vector3 F_environ(0.0,0.0,0.0);

		// Addition
		cur_HCommand.xset = (cur_HCommand.xset + F_environ.x)/eta;
		cur_HCommand.yset = (cur_HCommand.yset + F_environ.y)/eta;
		cur_HCommand.zset = (cur_HCommand.zset + F_environ.z)/eta;
		
		/////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!///////////////////////
		/////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!///////////////////////
		/////////////!!!!!!!!!!!!!!!!!!!DELAY HERE!!!!!!!!!!!!!!///////////////////////
		/////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!///////////////////////
		/////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!///////////////////////		

		// sending data: "cur_HCommand.xset" with delay (y=1/eta(dx+dpi/dp))
		QueryPerformanceCounter(&l);
		QueryPerformanceFrequency(&freq);
		StoM.acquisition_time=(double)(l.QuadPart-initialTick_hd)/freq.QuadPart;
		
		StoM.i[0]=cur_HCommand.xset; StoM.i[1]=cur_HCommand.yset; StoM.i[2]=cur_HCommand.zset;

		double P_rand1 = (double)rand()/RAND_MAX;
		double P_delay1 = (del_U-del_L)*P_rand1+del_L;
		StoM.time_to_go = StoM.acquisition_time+P_delay1;
		
		if (P_rand1 < 0.025)
			StoM.Loss = 1;
		else
			StoM.Loss = 0;

		dq_Data1.push_back(StoM);
		std::sort(dq_Data1.begin(), dq_Data1.end(), Less);

		QueryPerformanceCounter(&l_cur);
		double current_time1 = (double)(l_cur.QuadPart-initialTick_hd)/freq.QuadPart;
		data_check++;
		if (dq_Data1[0].time_to_go < current_time1)
		{

			if(dq_Data1[0].acquisition_time > packet_send1.data.acquisition_time)
			{
				if (dq_Data1[0].Loss==1)
				{
					dq_Data1.pop_front();
					loss_check++;
				}
				else
				{
					packet_send1.data.i[0]=dq_Data1[0].i[0];
					packet_send1.data.i[1]=dq_Data1[0].i[1];
					packet_send1.data.i[2]=dq_Data1[0].i[2];

					packet_send1.data.acquisition_time=dq_Data1[0].acquisition_time;
					dq_Data1.pop_front();
				}
			}
			else
			{
				dq_Data1.pop_front();
				loss_check++;
			}
		}

		/////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!///////////////////////
		/////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!///////////////////////
		/////////////!!!!!!!!!!!!!!!!!!!DELAY HERE!!!!!!!!!!!!!!///////////////////////
		/////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!///////////////////////
		/////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!///////////////////////
		
		cur_HCommand.xset=packet_send1.data.i[0];
		cur_HCommand.yset=packet_send1.data.i[1];
		cur_HCommand.zset=packet_send1.data.i[2];
		cur_HCommand.seqNo=packet_send1.data.acquisition_time;
		
		double k_x_0 = 25.0;
		double k_x_1 = 20.0+10.0+10.0;
		double b_x = 5.0;
		double k_y_0 = k_x_0;
		double k_y_1 = k_x_1;
		double b_y = b_x;
		double k_z_0 = k_x_0;
		double k_z_1 = k_x_1;
		double b_z = b_x;

		static PSPM pspm_x(k_x_0, b_x, 1.0, 5.0, true, 1);
		static PSPM pspm_y(k_y_0, b_y, 1.0, 5.0, true, 1);
		static PSPM pspm_z(k_z_0, b_z, 1.0, 5.0, true, 1);	

		// Get the haptic information
		dhdGetPosition(&cur_HState.y, &cur_HState.x, &cur_HState.z); // Current haptic position
		cur_HState.z=-cur_HState.z;
		dhdGetLinearVelocity(&cur_HState.yv, &cur_HState.xv, &cur_HState.zv); //Current haptic velocity
		cur_HState.zv=-cur_HState.zv;
		
		// PSPM
		pspm_x.Modulate(cur_HState.x, cur_HCommand.xset, cur_HCommand.seqNo, 0.0);
		pspm_y.Modulate(cur_HState.y, cur_HCommand.yset, cur_HCommand.seqNo, 0.0);
		pspm_z.Modulate(cur_HState.z, cur_HCommand.zset, cur_HCommand.seqNo, 0.0);	
	
		//cur_HCommand.Fx = -b_x*cur_HState.xv - k_x_1*cur_HState.x - k_x_0*(cur_HState.x - cur_HCommand.xset); // Non-PSPM
		//cur_HCommand.Fy = -b_x*cur_HState.yv - k_x_1*cur_HState.y - k_x_0*(cur_HState.y - cur_HCommand.yset); // Non-PSPM
		//cur_HCommand.Fz = -b_x*cur_HState.zv - k_x_1*cur_HState.z - k_x_0*(cur_HState.z - cur_HCommand.zset); // Non-PSPM
		
		cur_HCommand.Fx = -b_x*cur_HState.xv - k_x_1*cur_HState.x - k_x_0*(cur_HState.x - pspm_x.yMod); // PSPM
		cur_HCommand.Fy = -b_x*cur_HState.yv - k_x_1*cur_HState.y - k_x_0*(cur_HState.y - pspm_y.yMod); // PSPM
		cur_HCommand.Fz = -b_x*cur_HState.zv - k_x_1*cur_HState.z - k_x_0*(cur_HState.z - pspm_z.yMod); // PSPM
		
		fx = min(max(cur_HCommand.Fx,-10),10); 
		fy = min(max(cur_HCommand.Fy,-10),10); 
		fz = min(max(cur_HCommand.Fz,-10),10); // consider the direction of your system
		Force.x=fx; Force.y=fy; Force.z=fz;
		
		
		if (dhdSetForce(fy, fx, -fz) < DHD_NO_ERROR) 
		{
			printf ("error: cannot set force (%s)\n", dhdErrorGetLastStr());
			done = 1;
		}
		
		
	// data from the master to the slave
	//////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////
		
		// Should we get the haptic position information again here?
		Vector3 qk(cur_HState.x, cur_HState.y, cur_HState.z);
	
		static Vector3 ddq_t(0.0,0.0,0.0);
		static Vector3 dq_t(0.0,0.0,0.0);
		static Vector3 q_t(0.0,0.0,0.0);
		
		/////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!///////////////////////
		/////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!///////////////////////
		/////////////!!!!!!!!!!!!!!!!!!!DELAY HERE!!!!!!!!!!!!!!///////////////////////
		/////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!///////////////////////
		/////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!///////////////////////		

		// sending data: "q_k" with delay
		QueryPerformanceCounter(&l);
		QueryPerformanceFrequency(&freq);
		MtoS.acquisition_time=(double)(l.QuadPart-initialTick_hd)/freq.QuadPart;

		MtoS.i[0]=qk.x; MtoS.i[1]=qk.y; MtoS.i[2]=qk.z;

		double P_rand2 = (double)rand()/RAND_MAX;
		double P_delay2 = (del_U-del_L)*P_rand2+del_L;
		MtoS.time_to_go = MtoS.acquisition_time+P_delay2;

		if (P_rand2 < 0.025)
			MtoS.Loss = 1;
		else
			MtoS.Loss = 0;

		dq_Data2.push_back(MtoS);
		std::sort(dq_Data2.begin(), dq_Data2.end(), Less);

		QueryPerformanceCounter(&l_cur);
		double current_time2 = (double)(l_cur.QuadPart-initialTick_hd)/freq.QuadPart;
		data_check++;
		if (dq_Data2[0].time_to_go < current_time2)
		{

			if(dq_Data2[0].acquisition_time > packet_send2.data.acquisition_time)
			{
				if (dq_Data2[0].Loss==1)
				{
					dq_Data2.pop_front();
					loss_check++;
				}
				else
				{
					packet_send2.data.i[0]=dq_Data2[0].i[0];
					packet_send2.data.i[1]=dq_Data2[0].i[1];
					packet_send2.data.i[2]=dq_Data2[0].i[2];

					packet_send2.data.acquisition_time=dq_Data2[0].acquisition_time;
					dq_Data2.pop_front();
				}
			}
			else
			{
				dq_Data2.pop_front();
				loss_check++;
			}
		}

		/////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!///////////////////////
		/////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!///////////////////////
		/////////////!!!!!!!!!!!!!!!!!!!DELAY HERE!!!!!!!!!!!!!!///////////////////////
		/////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!///////////////////////
		/////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!///////////////////////
		
		qk.x=packet_send2.data.i[0];
		qk.y=packet_send2.data.i[1];
		qk.z=packet_send2.data.i[2];
		
		double filter_k = 1400;
		double filter_b = 2.5*sqrt(filter_k);
		ddq_t = qk*filter_k - q_t*filter_k - dq_t*filter_b;
		dq_t = dq_t + ddq_t*T;
		q_t = q_t + dq_t*T;
		
// 		double ob_k = 0.01*3.0;
// 		double ob_mu = 0.1;
// 		double ob_d = 0.3;
// 
// 		double obs_x1=0.5, obs_x2=-0.5;
// 		double obs_y1=0.5, obs_y2=-0.5;
// 		double obs_z=-0.3;
// 		double obs_z_dis = -0.05;
// 
// 		static Obstacle obline1(Vector3(obs_x1, obs_y1, obs_z+obs_z_dis*0.0),Vector3(obs_x2, obs_y2, obs_z+obs_z_dis*0.0),10); //create obstacle
// 		static Obstacle obline2(Vector3(obs_x1, obs_y1, obs_z+obs_z_dis*1.0),Vector3(obs_x2, obs_y2, obs_z+obs_z_dis*1.0),10); //create obstacle
// 		static Obstacle obline3(Vector3(obs_x1, obs_y1, obs_z+obs_z_dis*2.0),Vector3(obs_x2, obs_y2, obs_z+obs_z_dis*2.0),10); //create obstacle
// 		static Obstacle obline4(Vector3(obs_x1, obs_y1, obs_z+obs_z_dis*3.0),Vector3(obs_x2, obs_y2, obs_z+obs_z_dis*3.0),10); //create obstacle
// 		static Obstacle obline5(Vector3(obs_x1, obs_y1, obs_z+obs_z_dis*4.0),Vector3(obs_x2, obs_y2, obs_z+obs_z_dis*4.0),10); //create obstacle
// 		static Obstacle obline6(Vector3(obs_x1, obs_y1, obs_z+obs_z_dis*5.0),Vector3(obs_x2, obs_y2, obs_z+obs_z_dis*5.0),10); //create obstacle
// 		static Obstacle obline7(Vector3(obs_x1, obs_y1, obs_z+obs_z_dis*6.0),Vector3(obs_x2, obs_y2, obs_z+obs_z_dis*6.0),10); //create obstacle
// 		static Obstacle obline8(Vector3(obs_x1, obs_y1, obs_z+obs_z_dis*7.0),Vector3(obs_x2, obs_y2, obs_z+obs_z_dis*7.0),10); //create obstacle
// 		static Obstacle obline9(Vector3(obs_x1, obs_y1, obs_z+obs_z_dis*8.0),Vector3(obs_x2, obs_y2, obs_z+obs_z_dis*8.0),10); //create obstacle
// 		static Obstacle obline10(Vector3(obs_x1, obs_y1, obs_z+obs_z_dis*9.0),Vector3(obs_x2, obs_y2, obs_z+obs_z_dis*9.0),10); //create obstacle
// 		static Obstacle obline11(Vector3(obs_x1, obs_y1, obs_z+obs_z_dis*10.0),Vector3(obs_x2, obs_y2, obs_z+obs_z_dis*10.0),10); //create obstacle
// 		static Obstacle obline12(Vector3(obs_x1, obs_y1, obs_z+obs_z_dis*11.0),Vector3(obs_x2, obs_y2, obs_z+obs_z_dis*11.0),10); //create obstacle
// 		static Obstacle obline13(Vector3(obs_x1, obs_y1, obs_z+obs_z_dis*12.0),Vector3(obs_x2, obs_y2, obs_z+obs_z_dis*12.0),10); //create obstacle
// 		static Obstacle obline14(Vector3(obs_x1, obs_y1, obs_z+obs_z_dis*13.0),Vector3(obs_x2, obs_y2, obs_z+obs_z_dis*13.0),10); //create obstacle
// 		static Obstacle obline15(Vector3(obs_x1, obs_y1, obs_z+obs_z_dis*14.0),Vector3(obs_x2, obs_y2, obs_z+obs_z_dis*14.0),10); //create obstacle
// 		static Obstacle obline16(Vector3(obs_x1, obs_y1, obs_z+obs_z_dis*15.0),Vector3(obs_x2, obs_y2, obs_z+obs_z_dis*15.0),10); //create obstacle
// 		static Obstacle obline17(Vector3(obs_x1, obs_y1, obs_z+obs_z_dis*16.0),Vector3(obs_x2, obs_y2, obs_z+obs_z_dis*16.0),10); //create obstacle
// 		static Obstacle obline18(Vector3(obs_x1, obs_y1, obs_z+obs_z_dis*17.0),Vector3(obs_x2, obs_y2, obs_z+obs_z_dis*17.0),10); //create obstacle
// 		static Obstacle obline19(Vector3(obs_x1, obs_y1, obs_z+obs_z_dis*18.0),Vector3(obs_x2, obs_y2, obs_z+obs_z_dis*18.0),10); //create obstacle
// 		static Obstacle obline20(Vector3(obs_x1, obs_y1, obs_z+obs_z_dis*19.0),Vector3(obs_x2, obs_y2, obs_z+obs_z_dis*19.0),10); //create obstacle
// 		static Obstacle obline21(Vector3(obs_x1, obs_y1, obs_z+obs_z_dis*20.0),Vector3(obs_x2, obs_y2, obs_z+obs_z_dis*20.0),10); //create obstacle

		static Vector3 p(0.0,0.0,-0.2);
		static Vector3 dp(0.0,0.0,0.0);
		static Vector3 ddp(0.0,0.0,0.0);
		static Vector3 dddp(0.0,0.0,0.0);

// 		Vector3 ob_u = obline1.Uobstacle(p,ob_k,ob_mu,ob_d)+obline2.Uobstacle(p,ob_k,ob_mu,ob_d)+obline3.Uobstacle(p,ob_k,ob_mu,ob_d)+obline4.Uobstacle(p,ob_k,ob_mu,ob_d)+obline5.Uobstacle(p,ob_k,ob_mu,ob_d)+obline6.Uobstacle(p,ob_k,ob_mu,ob_d)+obline7.Uobstacle(p,ob_k,ob_mu,ob_d)+obline8.Uobstacle(p,ob_k,ob_mu,ob_d)+obline9.Uobstacle(p,ob_k,ob_mu,ob_d)+obline10.Uobstacle(p,ob_k,ob_mu,ob_d)+obline11.Uobstacle(p,ob_k,ob_mu,ob_d)+obline12.Uobstacle(p,ob_k,ob_mu,ob_d)+obline13.Uobstacle(p,ob_k,ob_mu,ob_d)+obline14.Uobstacle(p,ob_k,ob_mu,ob_d)+obline15.Uobstacle(p,ob_k,ob_mu,ob_d)+obline16.Uobstacle(p,ob_k,ob_mu,ob_d)+obline17.Uobstacle(p,ob_k,ob_mu,ob_d)+obline18.Uobstacle(p,ob_k,ob_mu,ob_d)+obline19.Uobstacle(p,ob_k,ob_mu,ob_d)+obline20.Uobstacle(p,ob_k,ob_mu,ob_d)+obline21.Uobstacle(p,ob_k,ob_mu,ob_d);
// 		Vector3 phi_p = ob_u*(-1);
// 
// 		F_environ = ob_u;
// 		Matrix33 H = obline1.H(p,ob_k,ob_mu,ob_d)+obline2.H(p,ob_k,ob_mu,ob_d)+obline3.H(p,ob_k,ob_mu,ob_d)+obline4.H(p,ob_k,ob_mu,ob_d)+obline5.H(p,ob_k,ob_mu,ob_d)+obline6.H(p,ob_k,ob_mu,ob_d)+obline7.H(p,ob_k,ob_mu,ob_d)+obline8.H(p,ob_k,ob_mu,ob_d)+obline9.H(p,ob_k,ob_mu,ob_d)+obline10.H(p,ob_k,ob_mu,ob_d)+obline11.H(p,ob_k,ob_mu,ob_d)+obline12.H(p,ob_k,ob_mu,ob_d)+obline13.H(p,ob_k,ob_mu,ob_d)+obline14.H(p,ob_k,ob_mu,ob_d)+obline15.H(p,ob_k,ob_mu,ob_d)+obline16.H(p,ob_k,ob_mu,ob_d)+obline17.H(p,ob_k,ob_mu,ob_d)+obline18.H(p,ob_k,ob_mu,ob_d)+obline19.H(p,ob_k,ob_mu,ob_d)+obline20.H(p,ob_k,ob_mu,ob_d)+obline21.H(p,ob_k,ob_mu,ob_d);
// 		Matrix33 H_t = obline1.H_t(p,dp,ob_k,ob_mu,ob_d)+obline2.H_t(p,dp,ob_k,ob_mu,ob_d)+obline3.H_t(p,dp,ob_k,ob_mu,ob_d)+obline4.H_t(p,dp,ob_k,ob_mu,ob_d)+obline5.H_t(p,dp,ob_k,ob_mu,ob_d)+obline6.H_t(p,dp,ob_k,ob_mu,ob_d)+obline7.H_t(p,dp,ob_k,ob_mu,ob_d)+obline8.H_t(p,dp,ob_k,ob_mu,ob_d)+obline9.H_t(p,dp,ob_k,ob_mu,ob_d)+obline10.H_t(p,dp,ob_k,ob_mu,ob_d)+obline11.H_t(p,dp,ob_k,ob_mu,ob_d)+obline12.H_t(p,dp,ob_k,ob_mu,ob_d)+obline13.H_t(p,dp,ob_k,ob_mu,ob_d)+obline14.H_t(p,dp,ob_k,ob_mu,ob_d)+obline15.H_t(p,dp,ob_k,ob_mu,ob_d)+obline16.H_t(p,dp,ob_k,ob_mu,ob_d)+obline17.H_t(p,dp,ob_k,ob_mu,ob_d)+obline18.H_t(p,dp,ob_k,ob_mu,ob_d)+obline19.H_t(p,dp,ob_k,ob_mu,ob_d)+obline20.H_t(p,dp,ob_k,ob_mu,ob_d)+obline21.H_t(p,dp,ob_k,ob_mu,ob_d);
// 		
		p = p + dp*T;
		dp = q_t*eta;// + phi_p*(-1);
		ddp = dq_t*eta;// + H*dp*(-1);
		dddp = ddq_t*eta;// + H_t*dp*(-1) - H*ddp;
		
		//Force_env = phi_p;

	//////////////////////////////////////////////////////////////////////////

		cur_HState.xd = p;
		cur_HState.dxd = dp;
		cur_HState.ddxd = ddp;
		cur_HState.dddxd = dddp;

	////////////////////////////POST the state //////////////////////////////

		WaitForSingleObject(g_HStateMutex,INFINITE);
		memcpy(&g_HState, &cur_HState, sizeof(HState));
		ReleaseMutex(g_HStateMutex);

		TDelay = P_delay1+P_delay2;
		C_loss = loss_check;
		C_data = data_check;
		
	}
	
	return iResult;
}

