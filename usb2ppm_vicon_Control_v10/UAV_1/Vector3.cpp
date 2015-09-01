#include "StdAfx.h"
#include "Vector3.h"
#include "math.h"
Vector3::Vector3(void):x(0.0),y(0.0),z(0.0)
{
}
Vector3::Vector3(double ix, double iy, double iz):x(ix),y(iy),z(iz)
{
}
Vector3::~Vector3(void)
{
}
Vector3& Vector3::operator =(const Vector3& iVec)
{
	x = iVec.x;
	y = iVec.y;
	z = iVec.z;
	return *this;
}
void Vector3::display(void) const
{
	printf("(%f, %f, %f)\n ", x, y, z);
}
Vector3 Vector3::operator+(Vector3 &iVec) const
{
	return Vector3(x+iVec.x, y+iVec.y, z+iVec.z);
}
Vector3 Vector3::operator-(Vector3 &iVec) const
{
	return Vector3(x-iVec.x, y-iVec.y, z-iVec.z);
}
Vector3 Vector3::operator*(double mult)const
{
	return Vector3(x*mult, y*mult, z*mult);
}
Vector3 Vector3::operator /(double divisor) const
{
	return Vector3(x/divisor, y/divisor, z/divisor);
}
double Vector3::Magnitude(void) const
{
	return(sqrt(x*x+y*y+z*z));
}
double Vector3::Dot(Vector3 &iVec) const
{
	return x*iVec.x + y*iVec.y + z*iVec.z;
}
std::ostream& operator<<(std::ostream& out, const Vector3& iVec)
{
	out << iVec.x<<"	"<<iVec.y<<"	"<<iVec.z<<"\t";
	return out;
}

/* Test in the main func by the following
	Vector3 v1(1,2,3);
	Vector3 v2;
	v2=v1;
	double arry[9] = {1,2,3,4,5,6,7,8,9};
	Matrix33 m1(arry);
	(v1+v2).display();
	(v1-v2).display();
	cout<<v2.Magnitude()<<endl;
	cout<<m1;
	cout<<m1*v1;
	cout<<m1.Trans();
*/