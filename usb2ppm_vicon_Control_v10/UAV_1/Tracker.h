#pragma once
#include "DataStructure.h"
#include <windows.h>
#include "Client.h"
#include <iostream>
#include <deque>
using namespace ViconDataStreamSDK::CPP;
namespace
{
  std::string Adapt( const bool i_Value )
  {
    return i_Value ? "True" : "False";
  }

  std::string Adapt( const Direction::Enum i_Direction )
  {
    switch( i_Direction )
    {
      case Direction::Forward:
        return "Forward";
      case Direction::Backward:
        return "Backward";
      case Direction::Left:
        return "Left";
      case Direction::Right:
        return "Right";
      case Direction::Up:
        return "Up";
      case Direction::Down:
        return "Down";
      default:
        return "Unknown";
    }
  }

  std::string Adapt( const DeviceType::Enum i_DeviceType )
  {
    switch( i_DeviceType )
    {
      case DeviceType::ForcePlate:
        return "ForcePlate";
      case DeviceType::Unknown:
      default:
        return "Unknown";
    }
  }

  std::string Adapt( const Unit::Enum i_Unit )
  {
    switch( i_Unit )
    {
      case Unit::Meter:
        return "Meter";
      case Unit::Volt:
        return "Volt";
      case Unit::NewtonMeter:
        return "NewtonMeter";
      case Unit::Newton:
        return "Newton";
      case Unit::Unknown:
      default:
        return "Unknown";
    }
  }
}


class Tracker
{

public:
	Tracker(void);
	Tracker(int);
	~Tracker(void);
private:
	HANDLE hTrackerPollingThread;
public:
	int Ini(void);
	int Close(void);
private:
	Client MyClient;
private:
	int updateInterval;
	long long cpuFreq;
public:
	friend DWORD WINAPI TrackerPollingThreadFunc (LPVOID lpParam);
public:
	std::deque<TState> dq_TState;
};
