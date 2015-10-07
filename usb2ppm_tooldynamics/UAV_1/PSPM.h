#pragma once

class PSPM
{
public:
	PSPM(void);
	PSPM::PSPM(double K, double B,double initialEPercent, double maximumE, bool smoothOrNot, unsigned int SetCounter);
public:
	~PSPM(void);
public:
	double K;
	double B;
public:
	double E;
	double Eini;
	double Emax;
	double Ex;
public:
	bool IsSmooth;
	unsigned int Counter;
public:
	double x;//current position
	double xLast;//last position at t(k-1)
	double y;//position command
	double yNo;//sequence mark
	double yNoLast;
	double yLastLastLastLastLast;
	double yLastLastLastLast;
	double yLastLastLast;
	double yLastLast;
	double yLast;//last position command
	double yMod; //after modulation
	double Ey;//energy from counterpart
public: 
	double temp_smooth;
public:
	double CurTime;
	double LastTime;
public:
	bool IsFirst;
public:
	unsigned long long CpuFreq;
public:
	bool CheckCounter (void);
	void RepairData (void);
	void RepairEy(void);
	void Smooth (void);
	int Modulate (double ownPosition, double positionCommand, double positionCommandSeqMark, double EShuffledFromConterpart);

	inline int Sign(double v);

};
