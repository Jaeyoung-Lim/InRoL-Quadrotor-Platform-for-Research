#include "StdAfx.h"
#include <algorithm>
#include "Vector3.h"
#include "Desired.h"
using namespace std;

Desired::Desired(void) :x(0.0, 0.0, 0.0), v(0.0, 0.0, 0.0), a(0.0, 0.0, 0.0), da(0.0, 0.0, 0.0)
{
}

Desired::Desired(Vector3 _x, Vector3 _v, Vector3 _a, Vector3 _da) : x(_x.x, _x.y, _x.z), v(_v.x, _v.y, _v.z), a(_a.x, _a.y, _a.z), da(_da.x, _da.y, _da.z)
{
}

Desired::~Desired(void)
{
}

//TODO : check what if make _a to be void
Desired::Desired(int mode, int index, double time, double time0)
{
	Vector3 _x;
	Vector3 _v;
	Vector3 _a;
	Vector3 _da;
	double ntime = time - time0;

	if (mode == 1) // teleoperation
	{
		mode = 2;
		//haptic is not available
	}
	/////////////////////////////////////////////////////////////////////////////////

	else if (mode == 2) // main trajectory
	{
		if (index == 1) // for hovering
		{
			Vector3 xd(0.0, 0.0, -0.5);
			_x= xd;
		}

		else if (index == 2) // for autonomous flying
		{
			double ll = 0.5;
			double h = -1.5;
			int dt = 15;
			int sector = (int(ntime) / dt) % 4;
			double mtime = ntime - dt* (int(ntime) / dt);
			if (sector == 0)
			{
				Vector3 xd(max(-ll, ll - 3 * ll / dt*mtime), -ll, h);
				Vector3 dxd(-2 * ll / dt, 0.0, 0.0);
				_x= xd;	_v= dxd;
			}
			else if (sector == 1)
			{
				Vector3 xd(-ll, min(ll, -ll + 3 * ll / dt*mtime), h);
				Vector3 dxd(0.0, 2 * ll / dt, 0.0);
				_x= xd;	_v= dxd;
			}
			else if (sector == 2)
			{
				Vector3 xd(min(ll, -ll + 3 * ll / dt*mtime), ll, h);
				Vector3 dxd(2 * ll / dt, 0.0, 0.0);
				_x= xd;	_v= dxd;
			}
			else if (sector == 3)
			{
				Vector3 xd(ll, max(-ll, ll - 2 * ll / dt*mtime), h);
				Vector3 dxd(0.0, -2 * ll / dt, 0.0);
				_x= xd;	_v= dxd;
			}
		}

		else if (index == 3) // for landing
		{
			Vector3 xd(0.0, 0.0, -0.05);

			_x= xd;
		}
		else if (index == 4) // stay
		{
			double h = -1.0;
			Vector3 xd(0.0, 0.0, h);
			_x= xd;

		}
	}
	///////////////////////////////////////////////////////////////////////////
	//Erased Mode 3

	x = _x;
	v = _v;
	a = _a;
	da = _da;

}