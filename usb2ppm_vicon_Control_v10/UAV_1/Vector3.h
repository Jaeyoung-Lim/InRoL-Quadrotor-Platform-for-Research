#pragma once
#include <iostream>
class Vector3
{
public:
	Vector3(void);
	Vector3(double, double, double);
	~Vector3(void);
public:
	Vector3& operator=(const Vector3&);
	Vector3 operator+(Vector3&) const;
	Vector3 operator-(Vector3&) const;
	Vector3 operator*(double) const;
	Vector3 operator/(double) const;
	double Dot(Vector3&) const;
	Vector3 Cross(Vector3&) const;
	double Magnitude(void) const;
	void display (void) const;
public:
	double x;
	double y;
	double z;
public:
	friend std::ostream& operator<<(std::ostream&, const Vector3&);

};
