#ifndef _POINT_2D_H_
#define _POINT_2D_H_  
#include <math.h>
#define M_PI_X2    6.283185307179586476925
#define M_PI       3.14159265358979323846
#define M_PI_2     1.57079632679489661923
#define M_PI_3     1.047197551196597746154
#define M_PI_4     0.785398163397448309616

#define M_R2Angle(x) ((x)*180.0/M_PI)
#define M_A2Radian(angle) ((angle)*M_PI/180.0)
#define M_ESP (1e-10)


static double m_normalize_radian(double rad)
{
	if(rad<0){
		while(rad<0){
		    rad+=M_PI_X2;
		}
	}
	else if(rad>M_PI_X2){
		while(rad>M_PI_X2){
		    rad-=M_PI_X2;
		}
	}
	return rad;
}


class Point2D
{
public:
	Point2D(){_x=0;_y=0;}
	Point2D(double x,double y)
	:_x(x),_y(y){}

	Point2D(double x1,double y1,double x2,double y2)
	{
	    _x=x2-x1;
		_y=y2-y1;
	}

	Point2D(double rad)
	{  
		_x=cos(rad);
		_y=sin(rad);
	}

	~Point2D(void){}

	double _x,_y;

	Point2D& normalize()
	{
		double r=length();
		if(r!=0){
		    _x/=r;
			_y/=r;
		}
		return *this;
	}

	double length() const 
	{
	    double r=sqrt(_x*_x+_y*_y);
		return r;
	}

	double radian()
	{
		double res=0;

		if(abs(_x)<=M_ESP){
			if(_y>0){
			    res=M_PI/2.0;
			}
			else{
			    res=-M_PI/2.0;
			}
		}
		else{
		    res=atan(_y/_x);
			if(_x<-M_ESP){
			    res+=M_PI;
			}
		}
		// -M_PI/2 M_PI/2
		if(res<0){
		    res+=M_PI_X2;
		}
		return res;
	}

	Point2D& operator*=(double k)
	{
	    _x*=k;
		_y*=k;
		return *this;
	}

	Point2D& operator+=(const Point2D& p)
	{
	    _x+=p._x;
		_y+=p._y;
		return *this;
	}

	Point2D rotate(double rad)
	{
		 double a,b;

		 a=cos(rad);
		 b=sin(rad);
		 
		 return Point2D(_x*a-_y*b,_x*b+_y*a);
	}

	Point2D rotate(const Point2D & center,double rad)
	{
		Point2D a(_x-center._x,_y-center._y);
		Point2D b=a.rotate(rad);
		
		return Point2D(center._x+b._x,center._y+b._y);
	}
};


static Point2D operator+(const Point2D &p1,const Point2D& p2)
{
	return Point2D(p1._x+p2._x,p1._y+p2._y);
}

double operator*(const Point2D& p1,const Point2D& p2)
{
	return (p1._x*p2._x+p1._y*p2._y);
}

static Point2D operator-(const Point2D& p1, const Point2D&p2)
{
	return Point2D(p1._x-p2._x,p1._y-p2._y);
}

static Point2D operator*(double k,const Point2D&p)
{
	return Point2D(k*p._x,k*p._y);
}

static double operator%(const Point2D& p1,const Point2D& p2)
{
	return p1._x*p2._y-p2._x*p1._y;
}

class Line2D
{
public:
	double A;
	double B;
	double C;

	Line2D(double a,double b,double c):A(a),B(b),C(c){}
	Line2D(const Point2D& p,double theta)
	{
	    A=sin(theta);
		B=-sqrt(1-A*A);
		C=p._x*A+p._y*B;
	}
};


static bool m_solve(const Line2D& line1,const Line2D& line2,Point2D& res)
{
	double a=line1.A*line2.B-line2.A*line1.B;

	if(abs(a)<M_ESP){
	    return false;
	}

	res._x=(line1.C*line2.B-line2.C*line1.B)/a;
	res._y=(line2.C*line1.A-line1.C*line2.A)/a;

	return true;
}

#endif