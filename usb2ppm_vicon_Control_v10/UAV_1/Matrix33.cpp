#include "StdAfx.h"
#include "Matrix33.h"

Matrix33::Matrix33(void): r1(1,0,0), r2(0,1,0), r3(0,0,1)
{
}
Matrix33::Matrix33(double (&vec)[9]): r1(vec[0],vec[1],vec[2]), r2(vec[3],vec[4],vec[5]), r3(vec[6],vec[7],vec[8])
{
}
Matrix33::~Matrix33(void)
{
}

void Matrix33::Display(void) const
{
	printf("%2.7f	%2.7f	%2.7f\n", r1.x,r1.y,r1.z);
	printf("%2.7f	%2.7f	%2.7f\n", r2.x,r2.y,r2.z);
	printf("%2.7f	%2.7f	%2.7f\n", r3.x,r3.y,r3.z);
	printf("\n");
}
Vector3 Matrix33::operator *(Vector3 &iVec) const
{
	return Vector3(r1.Dot(iVec), r2.Dot(iVec), r3.Dot(iVec));
}
Matrix33 Matrix33::operator *(double iDouble) const
{
	Matrix33 oM;
	oM.r1 = r1*iDouble;
	oM.r2 = r2*iDouble;
	oM.r3 = r3*iDouble;
	return oM;
}
Matrix33 Matrix33::operator /(double iDouble) const
{
	Matrix33 oM;
	oM.r1 = r1/iDouble;
	oM.r2 = r2/iDouble;
	oM.r3 = r3/iDouble;
	return oM;
}
Matrix33 Matrix33::operator +(Matrix33 iM) const
{
	Matrix33 oM;
	oM.r1 = r1 + iM.r1;
	oM.r2 = r2 + iM.r2;
	oM.r3 = r3 + iM.r3;
	return oM;
}
Matrix33 Matrix33::Trans(void) const
{
	Matrix33 mat;
	mat.r1.x = r1.x; mat.r1.y = r2.x; mat.r1.z = r3.x;
	mat.r2.x = r1.y; mat.r2.y = r2.y; mat.r2.z = r3.y;
	mat.r3.x = r1.z; mat.r3.y = r2.z; mat.r3.z = r3.z;
	return mat;
}

Matrix33 Matrix33::operator *(Matrix33 iM) const
{
	Matrix33 oM;
	double Array[9];
	Array[0] = r1.x;
	Array[1] = r1.y;
	Array[2] = r1.z;
	Array[3] = r2.x;
	Array[4] = r2.y;
	Array[5] = r2.z;
	Array[6] = r3.x;
	Array[7] = r3.y;
	Array[8] = r3.z;

	oM.r1.x = (Array[0]*iM.r1.x) + (Array[1]*iM.r2.x) + (Array[2]*iM.r3.x);
	oM.r1.y = (Array[0]*iM.r1.y) + (Array[1]*iM.r2.y) + (Array[2]*iM.r3.y);
	oM.r1.z = Array[0]*iM.r1.z + Array[1]*iM.r2.z + Array[2]*iM.r3.z;

	oM.r2.x = Array[3]*iM.r1.x + Array[4]*iM.r2.x + Array[5]*iM.r3.x;
	oM.r2.y = Array[3]*iM.r1.y + Array[4]*iM.r2.y + Array[5]*iM.r3.y;
	oM.r2.z = Array[3]*iM.r1.z + Array[4]*iM.r2.z + Array[5]*iM.r3.z;

	oM.r3.x = Array[6]*iM.r1.x + Array[7]*iM.r2.x + Array[8]*iM.r3.x;
	oM.r3.y = Array[6]*iM.r1.y + Array[7]*iM.r2.y + Array[8]*iM.r3.y;
	oM.r3.z = Array[6]*iM.r1.z + Array[7]*iM.r2.z + Array[8]*iM.r3.z;

	
	return oM;
}

std::ostream& operator<<(std::ostream &out, Matrix33 &mat)
{
	out<<mat.r1.x<<"	"<<mat.r1.y<<"	"<<mat.r1.z<<"\n"
	   <<mat.r2.x<<"	"<<mat.r2.y<<"	"<<mat.r2.z<<"\n"
	   <<mat.r3.x<<"	"<<mat.r3.y<<"	"<<mat.r3.z<<std::endl;
	return out;
}

