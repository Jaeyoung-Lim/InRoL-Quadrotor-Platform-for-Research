#include "stdafx.h"
#include "Tracker.h"
#include <algorithm>

extern HANDLE g_TStateMutex;
extern HANDLE g_TStateMutex2;
extern TState g_TState;
extern TState g_TState2;

DWORD WINAPI TrackerPollingThreadFunc (LPVOID lpParam);
double CaliPosition[3];
int TCounter = 0;
Tracker::Tracker(void)
{	
	hTrackerPollingThread = INVALID_HANDLE_VALUE;
	updateInterval = 200-200; //200ms default
	LARGE_INTEGER freq;
	QueryPerformanceFrequency(&freq);
	cpuFreq = freq.QuadPart;
}

Tracker::~Tracker(void)
{
	this->Close();
}
int Tracker::Ini(void)
{
	int iResult = 0;

	bool TransmitMulticast = false;
	std::string HostName = "localhost:801";

	
  // Connect to a server
  std::cout << "Connecting to " << HostName << " ..." << std::flush;
  while( !MyClient.IsConnected().Connected )
  {
    // Direct connection
    MyClient.Connect( HostName );
    std::cout << ".";
    Sleep( 200 );
  }
  std::cout<<std::endl;
  MyClient.EnableSegmentData();
  MyClient.EnableMarkerData();
  MyClient.EnableUnlabeledMarkerData();
  MyClient.EnableDeviceData();

  std::cout << "Segment Data Enabled: "          << Adapt( MyClient.IsSegmentDataEnabled().Enabled )         << std::endl;
  std::cout << "Marker Data Enabled: "           << Adapt( MyClient.IsMarkerDataEnabled().Enabled )          << std::endl;
  std::cout << "Unlabeled Marker Data Enabled: " << Adapt( MyClient.IsUnlabeledMarkerDataEnabled().Enabled ) << std::endl;
  std::cout << "Device Data Enabled: "           << Adapt( MyClient.IsDeviceDataEnabled().Enabled )          << std::endl;

  // Set the streaming mode
  MyClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ClientPull );
  // MyClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ClientPullPreFetch );
  // MyClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ServerPush );

  MyClient.SetAxisMapping(	Direction::Forward, 
							Direction::Right, 
							Direction::Down ); // Z-down
  Output_GetAxisMapping _Output_GetAxisMapping = MyClient.GetAxisMapping();
  std::cout << "Axis Mapping: X-" << Adapt( _Output_GetAxisMapping.XAxis ) 
                         << " Y-" << Adapt( _Output_GetAxisMapping.YAxis ) 
                         << " Z-" << Adapt( _Output_GetAxisMapping.ZAxis ) << std::endl;

  // Discover the version number
  Output_GetVersion _Output_GetVersion = MyClient.GetVersion();
  std::cout << "Version: " << _Output_GetVersion.Major << "." 
                           << _Output_GetVersion.Minor << "." 
                           << _Output_GetVersion.Point << std::endl;

  if( TransmitMulticast )
  {
    MyClient.StartTransmittingMulticast( "localhost", "224.0.0.0" );
  }

  // Here Start the Polling loop
	hTrackerPollingThread = CreateThread(NULL, 0, TrackerPollingThreadFunc, this, 0, NULL);
  return iResult;
}

int Tracker::Close(void)
{
	int iResult = 0;
	if(hTrackerPollingThread != INVALID_HANDLE_VALUE)
	CloseHandle(hTrackerPollingThread);
	MyClient.Disconnect();
	return iResult;

}

bool Less(TState item1, TState item2)
{
	return item1.time_to_go < item2.time_to_go;
}

DWORD WINAPI TrackerPollingThreadFunc (LPVOID lpParam)  // The rate is about 100Hz
{
	Tracker *pTracker = (Tracker *)lpParam;

	TState last_TState;
	last_TState.FrameNumber = pTracker->MyClient.GetFrameNumber().FrameNumber;
	last_TState.TotalLatency = pTracker->MyClient.GetLatencyTotal().Total;
	last_TState.NoOfSubjects = pTracker->MyClient.GetSubjectCount().SubjectCount;
	std::string SubjectName = pTracker->MyClient.GetSubjectName(0).SubjectName;
	last_TState.NoOfSegments = pTracker->MyClient.GetSegmentCount(SubjectName).SegmentCount;
	std::string SegmentName = pTracker->MyClient.GetSegmentName(SubjectName,0).SegmentName;

	while(true)
	{
		while( pTracker->MyClient.GetFrame().Result != Result::Success )
		{
		  // Sleep a little so that we don't lumber the CPU with a busy poll
			Sleep( pTracker->updateInterval );
		}
		
	TState cur_TState;
	cur_TState.FrameNumber = pTracker->MyClient.GetFrameNumber().FrameNumber;
	cur_TState.TotalLatency = pTracker->MyClient.GetLatencyTotal().Total;
	cur_TState.NoOfSubjects = pTracker->MyClient.GetSubjectCount().SubjectCount;
	std::string SubjectName = pTracker->MyClient.GetSubjectName(0).SubjectName;
	cur_TState.NoOfSegments = pTracker->MyClient.GetSegmentCount(SubjectName).SegmentCount;
	std::string SegmentName = pTracker->MyClient.GetSegmentName(SubjectName,0).SegmentName;

	/*
	TState cur_TState2;
	cur_TState2.FrameNumber = pTracker->MyClient.GetFrameNumber().FrameNumber;
	cur_TState2.TotalLatency = pTracker->MyClient.GetLatencyTotal().Total;
	cur_TState2.NoOfSubjects = pTracker->MyClient.GetSubjectCount().SubjectCount;
	std::string SubjectName2 = pTracker->MyClient.GetSubjectName(1).SubjectName;
	cur_TState2.NoOfSegments = pTracker->MyClient.GetSegmentCount(SubjectName2).SegmentCount;
	std::string SegmentName2 = pTracker->MyClient.GetSegmentName(SubjectName2,0).SegmentName;
	*/
	//Here calculate velocity

	static LARGE_INTEGER l;
	static LARGE_INTEGER llast = {0};
	
	QueryPerformanceCounter(&l);
	cur_TState.seqNo = l.QuadPart;
	double freq = (double)pTracker->cpuFreq;

	double T = (double)((l.QuadPart)-(llast.QuadPart))/freq;
	llast.QuadPart = l.QuadPart;
	
	//printf("The time interval %f\r",T);
	static double xlast[3]  = {0};
	static double vlast[3] = {0};
	
	memcpy(cur_TState.Translation, pTracker->MyClient.GetSegmentGlobalTranslation( SubjectName, SegmentName ).Translation, sizeof(cur_TState.Translation)); // position
	memcpy(cur_TState.RotationMatrix, pTracker->MyClient.GetSegmentGlobalRotationMatrix( SubjectName, SegmentName ).Rotation, sizeof(cur_TState.RotationMatrix)); // rotation matrix
	memcpy(cur_TState.RotationEuler, pTracker->MyClient.GetSegmentGlobalRotationEulerXYZ(SubjectName, SegmentName ).Rotation, sizeof(cur_TState.RotationEuler)); //euler angle

	static double xlast2[3]  = {0};
	static double vlast2[3] = {0};

	/*
	memcpy(cur_TState2.Translation, pTracker->MyClient.GetSegmentGlobalTranslation( SubjectName2, SegmentName2 ).Translation, sizeof(cur_TState2.Translation)); // position
	memcpy(cur_TState2.RotationMatrix, pTracker->MyClient.GetSegmentGlobalRotationMatrix( SubjectName2, SegmentName2 ).Rotation, sizeof(cur_TState2.RotationMatrix)); // rotation matrix
	memcpy(cur_TState2.RotationEuler, pTracker->MyClient.GetSegmentGlobalRotationEulerXYZ(SubjectName2, SegmentName2 ).Rotation, sizeof(cur_TState2.RotationEuler)); //euler angle
	*/
	//velocity
	double Wlast = 0.5;
	cur_TState.Velocity[0] = (1.0-Wlast)*(cur_TState.Translation[0] - xlast[0])/T + Wlast*vlast[0]; 
	cur_TState.Velocity[1] = (1.0-Wlast)*(cur_TState.Translation[1] - xlast[1])/T + Wlast*vlast[1];
	cur_TState.Velocity[2] = (1.0-Wlast)*(cur_TState.Translation[2] - xlast[2])/T + Wlast*vlast[2];

	/*
	cur_TState2.Velocity[0] = (1.0-Wlast)*(cur_TState2.Translation[0] - xlast2[0])/T + Wlast*vlast2[0]; 
	cur_TState2.Velocity[1] = (1.0-Wlast)*(cur_TState2.Translation[1] - xlast2[1])/T + Wlast*vlast2[1];
	cur_TState2.Velocity[2] = (1.0-Wlast)*(cur_TState2.Translation[2] - xlast2[2])/T + Wlast*vlast2[2];
	*/
	
	memcpy(xlast, cur_TState.Translation, sizeof(xlast));
	memcpy(vlast, cur_TState.Velocity, sizeof(vlast));
	
	/*
	memcpy(xlast2, cur_TState2.Translation, sizeof(xlast2));
	memcpy(vlast2, cur_TState2.Velocity, sizeof(vlast2));
	*/

	// copying TState from cur_TState to g_TState
	WaitForSingleObject(g_TStateMutex, INFINITE);
	memcpy(&g_TState, &cur_TState, sizeof(TState));
	ReleaseMutex(g_TStateMutex);

	/*
	WaitForSingleObject(g_TStateMutex2, INFINITE);
	memcpy(&g_TState2, &cur_TState2, sizeof(TState));
	ReleaseMutex(g_TStateMutex2);
	*/	
	//Sleep(2);
	}//while(true)
}