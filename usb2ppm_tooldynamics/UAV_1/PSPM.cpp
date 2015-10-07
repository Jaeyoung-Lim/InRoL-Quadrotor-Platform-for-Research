#include "StdAfx.h"
#include "PSPM.h"
#include <iostream>
#include <time.h>
#include <algorithm>
#include <windows.h>

#include <iomanip>  
#include <math.h>
using namespace std;

PSPM::PSPM(void)
{
	LARGE_INTEGER freq;
	QueryPerformanceFrequency(&freq);
	CpuFreq = (unsigned long long)freq.HighPart << 32 | freq.LowPart;
}
PSPM::PSPM(double springK, double dampingB,double initialEPercent, double maximumE, bool smoothOrNot, unsigned int SetCounter)
{
	LARGE_INTEGER freq;
	QueryPerformanceFrequency(&freq);
	CpuFreq = (unsigned long long)freq.HighPart << 32 | freq.LowPart;
	K = springK;
	B = dampingB;
	Eini = initialEPercent;
	Emax = maximumE;
	E = Eini * Emax;
	Ex = 0;
	temp_smooth= 0;

	IsSmooth = smoothOrNot;
	Counter = SetCounter;

	IsFirst = true;
}
PSPM::~PSPM(void)
{
}
int PSPM::Modulate(double ownPosition, double positionCommand, double positionCommandSeqMark, double EShuffledFromConterpart)
{
	x = ownPosition;
	y = positionCommand;
	Ey = EShuffledFromConterpart;
	yNo = positionCommandSeqMark;
	RepairData();//if data is new then do nothing
	RepairEy();//if data has been used then make Ey to zero
	Smooth();

	double timeInterval;

	double D;
	double deltP;
	double lhs;
	double rhs;

	long long CurTimeCount;
	LARGE_INTEGER t;
	QueryPerformanceCounter(&t);
	CurTimeCount = (unsigned long long)t.QuadPart;
	CurTime = (long double)CurTimeCount/CpuFreq;

	if(IsFirst) // At t(0), do nothing
	{
		IsFirst = false;
		yMod = y;
		
		LastTime = CurTime;
		xLast = x;
		yLastLastLastLastLast = y;
		yLastLastLastLast = y;
		yLastLastLast = y;
		yLastLast = y;
		yLast = y;
		yNoLast = yNo;
		return 0;
	}
	else
	{
		//get yMod
		timeInterval = CurTime - LastTime;

		D = B * pow((xLast - x), 2.0)/timeInterval;
		lhs = 2.0*(E + Ey + D)/K + pow(x - yLast, 2.0);
		rhs = sqrt(lhs); // rhs must be positive by theory

		if (abs(x - y) > rhs) //if y is resulting additional energy
		{
			yMod =x - Sign(x - y) * rhs;
			
		}
		else
		{
			yMod = y;
		}
		deltP = K / 2 * (pow( x - yMod, 2.0) - pow( x - yLast, 2.0));	
		E = E + Ey + D - deltP;
		
		//cout << "E + Ey + D - deltP: "<<E<<"="<< E - Ey - D + deltP<<"+"<< Ey<<"+"<<D<<"-"<<deltP<<endl;
		//get E and Ex
		if (E > Emax)
		{
			Ex = E - Emax;
			E = Emax;
		}
		else
		{
			if (E < 0)
			{
				E = 0.0;
			}
			Ex = 0.0;
		}

		LastTime = CurTime;
		xLast = x;
		yLastLastLastLastLast = yLastLastLastLast;
		yLastLastLastLast = yLastLastLast;
		yLastLastLast = yLastLast;
		yLastLast = yLast;
		yLast = yMod;
		
		yNoLast = yNo;

		//std::cout << std::setprecision(10)<< "D: "<< D<<endl;
		//std::cout << std::setprecision(10)<< "P: "<<deltP <<endl;
		return 0;
	}
}
void PSPM::RepairData()
{
	if (yNoLast > yNo)
	{
		y = yLast;
	}
}
void PSPM::RepairEy()
{
	if(yNoLast == yNo)
	{
		Ey = 0;
	}
}
void PSPM::Smooth()
{
	double Lambda = 0.01;
	if(IsSmooth) //maybe problem
	{
		temp_smooth = temp_smooth + Lambda * (y - temp_smooth);//(y+yLast+yLastLast+yLastLastLast+yLastLastLastLast)/4;
		y = temp_smooth;
	}

}
int PSPM::Sign(double v)
{
	if ( v > 0)
		return 1;
	else if (v == 0)
		return 0;
	else
		return -1;
}

