#include "StdAfx.h"
#include "Obstacle.h"
#include "math.h"

Obstacle::Obstacle(void)
{
}
Obstacle::Obstacle(Vector3 ob_center) //point
{
	center = ob_center;
	radius = 0.0;
}
Obstacle::Obstacle(Vector3 A, Vector3 B, unsigned int Npoints) //line
{
	if(Npoints <= 0)
	{
		std::cout << "error at creating line" <<std::endl;
	}
	center = (A+B)/2;
	for(unsigned int i = 0; i<1+Npoints; i++)
	{
		Components.push_back(Obstacle(A+(B-A)/Npoints*i));
	}
}
Obstacle::Obstacle(Vector3 ob_center, double ob_radius) //sphere
{
	center = ob_center;
	radius = ob_radius;
}
Obstacle::Obstacle(Vector3 ob_center, double ob_length, double ob_width) //cube
{
	center = ob_center;
	radius = sqrt(ob_length*ob_length/4+ob_width*ob_width/4);
}
Obstacle::Obstacle(Vector3 A, Vector3 B, Vector3 C, unsigned int ABpoints, unsigned int BCpoints) //plane
{
	if(ABpoints<= 0 || BCpoints<=0)
	{
		std::cout << "error at creating Plane" <<std::endl;
	}
	center = (A + C)/2.0;
	radius = (A - C).Magnitude()/2;
	Vector3 D = A + C - B;
	
	for(unsigned int i = 0; i<1+BCpoints; i++)
	{
		
		Vector3 ADi = A + (D-A)/BCpoints * i;
		Vector3 BCi = B + (C-B)/BCpoints * i;
		Obstacle l(ADi, BCi, ABpoints);
		Components.push_back(l);//push line to form plane
	}
}
Obstacle::Obstacle(Vector3 A, Vector3 B, Vector3 C, double H, unsigned int ABpoints, unsigned int BCpoints, unsigned int Hpoints)//A,B,C should on the xy plane
{
	if(ABpoints<= 0 || BCpoints<=0 || Hpoints<=0)
	{
		std::cout << "error at creating Cube" <<std::endl;
	}
	center = (A+C)/2 + Vector3(0,0,-1)*H/2;
	radius = sqrt((A - C).Magnitude()*(A - C).Magnitude()/4+H*H/4);

	for(unsigned int i=0; i<Hpoints+1; i++)
	{
		Vector3 Ai = A + Vector3(0,0,-1)*H/Hpoints*i;
		Vector3 Bi = B + Vector3(0,0,-1)*H/Hpoints*i;
		Vector3 Ci = C + Vector3(0,0,-1)*H/Hpoints*i;
		Components.push_back(Obstacle(Ai,Bi,Ci,ABpoints,BCpoints));
	}
}
Obstacle::~Obstacle(void)
{
}
Vector3 Obstacle::Uobstacle(Vector3 UAVpos, double k, double mu, double d) 
{
	//std::cout <<"size"<< Components.size()<<std::endl;
	Vector3 U_obs(0.0,0.0,0.0);
	
	if(Components.size() == 0)
		{
			double di = (UAVpos - center).Magnitude();
			//std::cout<<center;
			if(di>mu && di<=d)
			{
				U_obs = U_obs-(UAVpos-center)*4*k*(d*d - mu*mu)*(di*di - d*d)/pow((di*di - mu*mu),3);
				//std::cout << "____"<<U_obs<<std::endl;
			}
			/*
			double plimit = 500.0;
			double nlimit = -500.0;
			if(U_obs.x > plimit)
				U_obs.x = plimit;
			if(U_obs.x < nlimit)
				U_obs.x = nlimit;

			if(U_obs.y > plimit)
				U_obs.y = plimit;
			if(U_obs.y < nlimit)
				U_obs.y = nlimit;

			if(U_obs.z > plimit)
				U_obs.z = plimit;
			if(U_obs.z < nlimit)
				U_obs.z = nlimit;
			*/
			return U_obs;
		}
	unsigned int SizeofComponent =Components.size();
	for(unsigned int i = 0; i< SizeofComponent; i++)
	{
		U_obs = U_obs + Components[i].Uobstacle(UAVpos, k, mu, d);
	}
	return U_obs;
	
}

Matrix33 Obstacle::H(Vector3 p, double k, double mu, double d)
{
	double rtVector[9] = {0};
	Matrix33 rt(rtVector);

	if(Components.size() == 0)
		{				
			double x_mag = (p - center).Magnitude();
			if(x_mag>mu && x_mag<=d)
			{
				double a = x_mag*x_mag - mu*mu;
				double b = x_mag*x_mag -d*d;

				double ax = 2*(p.x - center.x);
				double bx = ax;
				double ay = 2*(p.y - center.y);
				double by = ay;
				double az = 2*(p.z - center.z);
				double bz = az;

				double E11 = (bx*a - 3*b*ax)/pow(a,4)*(p.x - center.x) + b/pow(a,3);
				double E12 = (by*a - 3*b*ay)/pow(a,4)*(p.x - center.x);
				double E13 = (bz*a - 3*b*az)/pow(a,4)*(p.x - center.x);
				
				double E21 = (bx*a - 3*b*ax)/pow(a,4)*(p.y - center.y);
				double E22 = (by*a - 3*b*ay)/pow(a,4)*(p.y - center.y) + b/pow(a,3);
				double E23 = (bz*a - 3*b*az)/pow(a,4)*(p.y - center.y);

				double E31 = (bx*a - 3*b*ax)/pow(a,4)*(p.z - center.z);
				double E32 = (by*a - 3*b*ay)/pow(a,4)*(p.z - center.z);
				double E33 = (bz*a - 3*b*az)/pow(a,4)*(p.z - center.z) + b/pow(a,3);

				double Es[9] = {E11,E12,E13,E21,E22,E23,E31,E32,E33};
				Matrix33 temprt(Es);	
				rt =  temprt*4*k*(d*d - mu*mu);
			}

			return rt;
		}
	unsigned int SizeofComponent =Components.size();
	for(unsigned int i = 0; i< SizeofComponent; i++)
	{
		rt = rt + Components[i].H(p,k,mu,d);
	}

	return rt;
}
Matrix33 Obstacle::H_t(Vector3 p, Vector3 dp, double k, double mu, double d)
{
	double rtValue[9] = {0};
	Matrix33 rt(rtValue);

	if(Components.size() == 0)
	{
		double x_mag = (p - center).Magnitude();
		if(x_mag>mu && x_mag<=d)
		{
				double a = x_mag*x_mag - mu*mu;
				double b = x_mag*x_mag -d*d;

				double ax = 2*(p.x - center.x);
				double bx = ax;
				double ay = 2*(p.y - center.y);
				double by = ay;
				double az = 2*(p.z - center.z);
				double bz = az;

			double cx = bx*a - 3*b*ax;
			double cy = by*a - 3*b*ay;
			double cz = bz*a - 3*b*az;
			
			double at = 2*((p.x - center.x)*dp.x + (p.y - center.y)*dp.y + (p.z - center.z)*dp.z);
			double bt = at;
			double cxt = (2*at - 6*bt)*(p.x - center.x) + (2*a - 6*b)*dp.x;
			double cyt = (2*at - 6*bt)*(p.y - center.y) + (2*a - 6*b)*dp.y;
			double czt = (2*at - 6*bt)*(p.z - center.z) + (2*a - 6*b)*dp.z;

			double ba3t = (bt*a - 3*b*at)/pow(a,4);
			
			double ctxa = (cxt*a - 4*cx*at)/pow(a,5);
			double ctya = (cyt*a - 4*cy*at)/pow(a,5);
			double ctza = (czt*a - 4*cz*at)/pow(a,5);
		

			double E11 = ctxa*(p.x - center.x) + cx/pow(a,4)*dp.x + ba3t;
			double E12 = ctya*(p.x - center.x) + cy/pow(a,4)*dp.x;
			double E13 = ctza*(p.x - center.x) + cz/pow(a,4)*dp.x;

			double E21 = ctxa*(p.y - center.y) + cx/pow(a,4)*dp.y;
			double E22 = ctya*(p.y - center.y) + cy/pow(a,4)*dp.y + ba3t;
			double E23 = ctza*(p.y - center.y) + cz/pow(a,4)*dp.y;

			double E31 = ctxa*(p.z - center.z) + cx/pow(a,4)*dp.z;
			double E32 = ctya*(p.z - center.z) + cy/pow(a,4)*dp.z;
			double E33 = ctza*(p.z - center.z) + cz/pow(a,4)*dp.z + ba3t;

				double Es[9] = {E11,E12,E13,E21,E22,E23,E31,E32,E33};
				Matrix33 temprt(Es);	
				rt =  temprt*4*k*(d*d - mu*mu);
		}
		return rt; //once components are not zero
	}
	unsigned int SizeofComponent =Components.size();
	for(unsigned int i = 0; i< SizeofComponent; i++)
	{
		rt = rt + Components[i].H_t(p,dp,k,mu,d);
	}
	return rt;
}
Obstacle& Obstacle::operator =(const Obstacle &iOb)
{
	center = iOb.center;
	radius = iOb.radius;
	Components = iOb.Components;
	return *this;
}
