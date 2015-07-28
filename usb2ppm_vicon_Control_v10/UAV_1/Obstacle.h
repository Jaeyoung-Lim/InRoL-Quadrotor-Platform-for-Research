#pragma once
#include "Vector3.h"
#include <vector>
#include "Matrix33.h"
enum EnumShape
{
	Point,
	Line,
	Plane,
	Sphere,
	Cube
};
class Obstacle
{
public:
	Obstacle(void);
	Obstacle(Vector3 ob_center);
	Obstacle(Vector3 A, Vector3 B, unsigned int Npoints);
	Obstacle(Vector3 ob_center, double ob_radius);
	Obstacle(Vector3 ob_center, double ob_length, double ob_width);
	Obstacle(Vector3 A, Vector3 B, Vector3 C, unsigned int ABpoints, unsigned int BCpoints);
	Obstacle(Vector3 A, Vector3 B, Vector3 C, double H, unsigned int ABpoints, unsigned int BCpoints, unsigned int Hpoints);

	~Obstacle(void);
public:
	Vector3 center;
	double radius;
public:
	std::vector<Obstacle> Components;
public:
	Vector3 Uobstacle(Vector3 UAVpos, double k, double mu, double d);
	Matrix33 H(Vector3 p, double k, double mu, double d);
	Matrix33 H_t(Vector3 p, Vector3 dp, double k, double mu, double d);
public:
	Obstacle& operator=(const Obstacle &iOb);
};
