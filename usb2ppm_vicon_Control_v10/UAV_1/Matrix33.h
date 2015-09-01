#pragma once
#include "Vector3.h"
#include <iostream>
class Matrix33
{
public:
	Vector3 r1, r2, r3;
public:
	Matrix33(void);
	Matrix33(double (&)[9]);
	~Matrix33(void);
public:
	Matrix33 Trans(void) const;
	Vector3 operator*(Vector3 &) const;
	Matrix33 operator*(double) const;
	Matrix33 operator/(double) const;
	Matrix33 operator+(Matrix33) const;
	Matrix33 operator*(Matrix33) const;
	void Display(void) const;
public:
	friend std::ostream& operator<<(std::ostream&, Matrix33&);
};
