/*
 * jvtypes.h
 *
 *  Created on: Sep 2, 2009
 *      Author: jscholz
 */

#ifndef JVTYPES_H_
#define JVTYPES_H_

#define RANDNM(N,M) N + ((M-N) * (rand() / ((double)RAND_MAX + 1))) // random # between N&M


#ifdef WIN32
#include <cv.h>
#include <highgui.h>
#else
#include <opencv/cv.h>
#include <opencv/highgui.h>
#endif

#include <vector>
#include <iostream>

using namespace std;

class jvtypes {
public:
	jvtypes();
	virtual ~jvtypes();
};

// Container for points or 2D vectors (not dependent on OpenCV)
class point {
public:
	point();
	point(double X, double Y);

	double x; // used to be an int!
	double y; // used to be an int!

	double mag(); // returns magnitude as though point represents a vector

	friend ostream& operator<<(ostream& os,const point& p0);

	// Vector addition/subtraction:
	friend point operator+(const point& p1, const point& p2);
	friend point operator-(const point& p1, const point& p2);

	// Scalar multiplication/division:
	friend point operator*(const point& p1, const double sval);
	friend point operator/(const point& p1, const double sval);
};

// A simpler container for lines than CVSeq
class linevec {
public:
	//TODO: switch to using point class?
	double x1;
	double y1;
	double x2;
	double y2;

	/**
	 * flag for indicating whether the line is a function in term of x (not parallel to y-axis)
	 * or not (parallel to y-axis)
	 */
	bool parallelToYAxis;

	linevec();
	linevec(double X1, double Y1, double X2, double Y2);
	linevec(point p1, double theta, double length);

	// Function to draw line on provided image
	void drawLine(IplImage* img, int B = 0, int G = 0, int R = 255);

	// Returns the length of the line
	double length();
	double slope();

	// Angle functions for upper-left origin (computer vision applications)
	double getAngleULO();
	void setAngleULO(double theta, double len = 0);

	// Slope functions for lower-left origin (standard applications)
	double getAngleLLO();
	void setAngleLLO(double theta, double len = 0);

	virtual linevec pix2wrk_coords();
	virtual linevec wrk2pix_coords();

	friend ostream& operator<<(ostream& os,const linevec& l);
};

class polygon {
public:
	polygon();
	~polygon();

	vector<point> pt;
	bool isObstacle;

	// Function to draw outline of polygon on provided image
	void drawPoly(IplImage* img, int B = 0, int G = 0, int R = 255, int thickness = 3);

	friend ostream& operator<<(ostream& os,const polygon& p);
};

class quad : public polygon {
public:
	quad();
	quad(point c, double a, double w = 60, double h = 20);
	/*
	 * Beware when using above constructor: width and height are defined in a dumb way
	 *   width is always NE edge, and height is always NW edge, even if it should be
	 *   based on the orientation such that the dimension which most contributes to
	 *   height gets the height term (if cos(t)<sin(t) then switch...)
	 */
	~quad();

	point center; // Coords of centroid
	double width; // Defined as the dim along the line from point 1 (top-most) to 2 (right-most)
	double height;// Defined as the dim along the line from point 0 (left-most) to 1 (top-most)
	double angle; // angle of LONG axis of quad from x-axis {-90:90}

	friend ostream& operator<<(ostream& os,const quad& q);
	friend bool operator> (const quad &q1, const quad &q2);
	friend bool operator<= (const quad &q1, const quad &q2);
	friend bool operator< (const quad &q1, const quad &q2);
	friend bool operator>= (const quad &q1, const quad &q2);
};

class action{
public:
	action();
	action(int o, int d);

	int object;
	int direction;

	linevec a2L(vector<quad> quads);

	friend ostream& operator<<(ostream& os,const action& a);
};


#endif /* JVTYPES_H_ */
