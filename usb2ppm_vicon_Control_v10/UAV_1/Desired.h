#pragma once
#include "Vector3.h"
class Desired
{
public:
	Desired(void);
	Desired(Vector3, Vector3, Vector3, Vector3);//x,v,a,dot(a)
	~Desired(void);

public:
	Desired (int, int, double, double);//mode index time time0

public:
	Vector3 x;
	Vector3 v;
	Vector3 a;
	Vector3 da;
};
