/*
 * jvtypes.cpp
 *
 *  Created on: Sep 2, 2009
 *      Author: jscholz
 */

#include "jvtypes.h"

jvtypes::jvtypes() {
	// TODO Auto-generated constructor stub

}

jvtypes::~jvtypes() {
	// TODO Auto-generated destructor stub
}

// Point: container for points or 2D vectors (not dependent on OpenCV)
point::point(){}

point::point(double X, double Y) {
	x = X;
	y = Y;
}

double point::mag()
{
	return sqrt(pow(x, 2) + pow(y, 2));
}

ostream& operator<<(ostream& os, const point& p) {
	os << "[" << p.x << "," << p.y << "]";
	return os;
}

// Vector addition/subtraction:
point operator+(const point& p1, const point& p2) {
	point result;
	result.x = p1.x + p2.x;
	result.y = p1.y + p2.y;
	return result;
}

point operator-(const point& p1, const point& p2) {
	point result;
	result.x = p1.x - p2.x;
	result.y = p1.y - p2.y;
	return result;
}

// Scalar multiplication/division:
point operator*(const point& p1, const double sval) {
	point result;
	result.x = p1.x * sval;
	result.y = p1.y * sval;
	return result;
}

point operator/(const point& p1, const double sval) {
	point result;
	result.x = p1.x / sval;
	result.y = p1.y / sval;
	return result;
}


// Linevec: a simpler container for lines than CVSeq
linevec::linevec() {
	// null constructor
}

linevec::linevec(double X1, double Y1, double X2, double Y2)
{
	x1 = X1;
	y1 = Y1;
	x2 = X2;
	y2 = Y2;

	if (x1 == x2){
		parallelToYAxis = true;
	} else {
		parallelToYAxis = false;
	}
}

linevec::linevec(point p1, double theta, double length)
{
	x1 = p1.x;
	y1 = p1.y;
	x2 = 0;
	y2 = 0;
	setAngleULO(theta,length);
	if (x1 == x2){
			parallelToYAxis = true;
		} else {
			parallelToYAxis = false;
		}
}



void linevec::drawLine(IplImage* img, int B, int G, int R)
{
	cvLine(img, cvPoint(x1,y1),cvPoint(x2,y2),cvScalar(B,G,R,0.5),3);
}

double linevec::length()
{
	return (sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2)));
}

double linevec::slope()
{
	if (!parallelToYAxis)
		return ((y2 - y1)/(x2 - x1));
	else return NULL;
}

double linevec::getAngleULO()
{
	// Angle method SPECIFICALLY for working in pixel coords with UL origin
	// *want angles w/ origin at lower left, not upper left, so need to flip Y (locally)
	double theta = atan2((-y2) - (-y1), x2 - x1);

	if (isinf(theta) == 1)
		theta = CV_PI / 2.;
	if (isinf(theta) == -1)
		theta = -CV_PI / 2.;

	return (theta);
}

double linevec::getAngleLLO()
{
	// Angle method SPECIFICALLY for working in standard coords with LL origin
	double theta = atan2(y2 - y1, x2 - x1);

	if (isinf(theta) == 1)
		theta = CV_PI / 2.;
	if (isinf(theta) == -1)
		theta = -CV_PI / 2.;

	return (theta);
}

void linevec::setAngleULO(double theta, double len) // pass theta in radians
{
	// SET ANGLE method SPECIFICALLY for working in pixel coords with UL origin
	if (len == 0)
		len = this->length();

	x2 = x1 + len * cos(theta);
	y2 = y1 - len * sin(theta);
}

void linevec::setAngleLLO(double theta, double len) // pass theta in radians
{
	// SET_ANGLE method SPECIFICALLY for working in standard coords with UL origin
	if (len == 0)
		len = this->length();

	x2 = x1 + len * cos(theta);
	y2 = y1 + len * sin(theta);
}

linevec linevec::pix2wrk_coords()
{

	static double pix2wrk_calibmtrx[2][3] = {
			{-0.00000560,-0.00569763,1.23908362},
			{0.00570281,0.00000000,-1.22566265}};

	// convert from pixel to workspace coordinates
	return linevec(x1*pix2wrk_calibmtrx[0][0] + y1*pix2wrk_calibmtrx[0][1] + pix2wrk_calibmtrx[0][2],
			x1*pix2wrk_calibmtrx[1][0] + y1*pix2wrk_calibmtrx[1][1] + pix2wrk_calibmtrx[1][2],
			x2*pix2wrk_calibmtrx[0][0] + y2*pix2wrk_calibmtrx[0][1] + pix2wrk_calibmtrx[0][2],
			x2*pix2wrk_calibmtrx[1][0] + y2*pix2wrk_calibmtrx[1][1] + pix2wrk_calibmtrx[1][2]);
}

linevec linevec::wrk2pix_coords()
{
	static double  wrk2pix_calibmtrx[2][3] = {
			{-0.00000000,175.35211268,214.92253521},
			{-175.51133734,-0.17245537,217.26209567}};

	// convert from workspace to pixel coordinates
	return linevec(x1*wrk2pix_calibmtrx[0][0] + y1*wrk2pix_calibmtrx[0][1] + wrk2pix_calibmtrx[0][2],
			x1*wrk2pix_calibmtrx[1][0] + y1*wrk2pix_calibmtrx[1][1] + wrk2pix_calibmtrx[1][2],
			x2*wrk2pix_calibmtrx[0][0] + y2*wrk2pix_calibmtrx[0][1] + wrk2pix_calibmtrx[0][2],
			x2*wrk2pix_calibmtrx[1][0] + y2*wrk2pix_calibmtrx[1][1] + wrk2pix_calibmtrx[1][2]);
}

ostream& operator<<(ostream& os, const linevec& l) {
	os << "[[" << l.x1 << "," << l.y1 << "];[" << l.x2 << "," << l.y2 << "]]";
	return os;
}


// Polygon - for storing contour information
ostream& operator<<(ostream& os, const polygon& p) {
	os << "[";
	for (int i = 0; i < p.pt.size(); i++) {
		os << p.pt[i];
	}
	os << "]";
	return os;
}

polygon::polygon()
{
	// Empty for now
}

polygon::~polygon()
{
	// Empty for now
}


void polygon::drawPoly(IplImage* img, int B, int G, int R, int thickness)
{
	for (int i=0; i<pt.size()-1; ++i){
		cvLine(img, cvPoint(pt[i].x,pt[i].y),cvPoint(pt[i+1].x, pt[i+1].y),cvScalar(B,G,R,0.5),thickness);
	}
	cvLine(img, cvPoint(pt.back().x,pt.back().y),cvPoint(pt[0].x, pt[0].y),cvScalar(B,G,R,0.5),thickness);
}

quad::quad()
{
	// Empty for now
}

quad::quad(point c, double a, double w, double h)
{
	center = c;
	angle = a; // constructor for degrees

	width = w;
	height = h;

	// Simple fix for the switch of height & width when crossing 0 degrees:
	double theta;
	if (angle > 0)
		theta = (90 - angle) * CV_PI/180;
	else
		theta = angle * CV_PI/180;

	// Quick check to verify that h & w are consistent with definition of angle:
	if ((angle >= 0 && h < w) || (angle <= 0 && w < h)) {
		width = h;
		height = w;
	}
	//TODO fix this by fixing the height vs. width logic in jvfeatures
	// to switch when cos(t)<sin(t) (more complicated, since things
	// switch at 0 and 45 degrees...)

	pt.resize(4);

	pt[0].x = center.x - width/2 * cos(theta)	- height/2 * sin(fabs(theta));
	pt[0].y = center.y - width/2 * sin(fabs(theta)) + height/2 * cos(theta);

	pt[1].x = center.x - width/2 * cos(theta) + height/2 * sin(fabs(theta));
	pt[1].y = center.y - width/2 * sin(fabs(theta)) - height/2 * cos(theta);

	pt[2].x = center.x + width/2 * cos(theta) + height/2 * sin(fabs(theta));
	pt[2].y = center.y + width/2 * sin(fabs(theta)) - height/2 * cos(theta);

	pt[3].x = center.x + width/2 * cos(theta) - height/2 * sin(fabs(theta));
	pt[3].y = center.y + width/2 * sin(fabs(theta)) + height/2 * cos(theta);

}

quad::~quad()
{
	// Empty for now
}

// Over-ridden stream method for quads
ostream& operator<<(ostream& os, const quad& q) {
	os << "[" << q.center.x << "," << q.center.y << "][" << q.angle << "]";
	return os;
}

// Some comparison operators for quad objects
bool operator> (const quad &q1, const quad &q2)
{
return q1.height + q1.width > q2.height + q2.width;
}

bool operator<= (const quad &q1, const quad &q2)
{
return q1.height + q1.width <= q2.height + q2.width;
}

bool operator< (const quad &q1, const quad &q2)
{
return q1.height + q1.width < q2.height + q2.width;
}


bool operator>= (const quad &q1, const quad &q2)
{
return q1.height + q1.width >= q2.height + q2.width;
}

// Null constructor for action
action::action()
{

}

// Standard constructor for action
action::action(int o, int d) {
	object = o;
	direction = d;
}

linevec action::a2L(vector<quad> quads)
{
	if (quads.size()==0)
		return linevec(0,0,0,0);

	double pushdist = 37; // length of movement vector, in pixels // 45
	double cushion = 15; // 15
	double pushangle;

	// params to
	double fudgefac = 0.1;
	point fudge;

	point p1;
	point pcush;

	// ## Uncomment to visualize action vectors {
//	jvf.draw_quadsimg();
//	direction=0;
//	object=0;
//	while(1) {
//	if (direction > 6)
//		direction = 0;
//	if (object > (neurl.num_objs-1))
//		object = 0;

	// Action possiblities: create vectors down short edges of block,
	// or through the center.  Have to accommodate both possible
	// orientations of the block, since pt[0] is always at xmin
	switch (direction) {

	// Push on NW face
	case 0:
		p1 = quads[object].pt[0] + (quads[object].pt[1] - quads[object].pt[0]) / 2;
		pushangle = quads[object].angle;
		if (pushangle > 0)
			pushangle = pushangle - 90; // ensure we push on NW face
		break;

	// Push on NE face
	case 1:
		p1 = quads[object].pt[1] + (quads[object].pt[2]	- quads[object].pt[1]) / 2;
		pushangle = quads[object].angle - 90;
		if (pushangle > -90)
				pushangle = pushangle - 90; // ensure we push on NE face
		break;

	// Push on SE face
	case 2:
		p1 = quads[object].pt[2] + (quads[object].pt[3]	- quads[object].pt[2]) / 2;
		pushangle = quads[object].angle + 180;
		if (pushangle > 180)
			pushangle = pushangle - 90; // ensure we push on SE face
		break;

	// Push on SW face
	case 3:
		p1 = quads[object].pt[0] + (quads[object].pt[3]	- quads[object].pt[0]) / 2;
		pushangle = quads[object].angle + 90;
		if (pushangle > 90)
			pushangle = pushangle - 90; // ensure we push on SW face
		break;

	// Push top right corner (rotate CW)
	case 4:
		pushangle = quads[object].angle - 90;
		if (quads[object].height > quads[object].width) {
			p1 = quads[object].pt[1];
			fudge = quads[object].pt[0] - quads[object].pt[1];
		} else {
			p1 = quads[object].pt[2];
			fudge = quads[object].pt[1] - quads[object].pt[2];
		}
		p1 = p1 + fudge * fudgefac;
		break;

	// Push bottom right corner (rotate CCW)
	case 5:
		pushangle = quads[object].angle + 90;
		if (quads[object].height > quads[object].width) {
			p1 = quads[object].pt[2];
			fudge = quads[object].pt[0] - quads[object].pt[1];
		} else {
			p1 = quads[object].pt[3];
			fudge = quads[object].pt[1] - quads[object].pt[2];
		}
		p1 = p1 + fudge * fudgefac;
		break;

	// Null action - should just zip around in the upper left corner
	case 6:
		// needs to be semi-random for moveP2P to realize it's a new action
		p1 = point(10, 10 + (int)RANDNM(0,10)); // jvf.roiX, jvf.roiY + (int)RANDNM(0,10)
		pushangle = 0;
		break;
	}

	// Translate by cushion (pixels) AWAY from goal to leave room for ee to maneuver
	pcush = point(p1.x + cushion * cos((pushangle+180)*CV_PI/180), p1.y - cushion * sin((pushangle+180)*CV_PI/180));

	// ## Uncomment to visualize action vectors {
//	IplImage* quadimg = cvCreateImage(cvSize(500,500), 8, 3);
//	cvNamedWindow("Quadimg",0);
//	for (int i=0; i<quads.size();++i)
//		quads[i].drawPoly(quadimg);
//	linevec m = linevec(pcush, pushangle*CV_PI/180, pushdist);
//	cout << "pushing object " << object << " (currently at " << quads[object].angle <<" degrees), act.direction = " << direction << endl;
//	cvLine(quadimg,cvPoint(m.x1,m.y1),cvPoint(m.x2, m.y2),cvScalar(0,0,255-20*direction,0.5),3);
//	cvLine(quadimg,cvPoint(m.x2-3,m.y2-3),cvPoint(m.x2, m.y2),cvScalar(0,255-20*direction,0,0.5),3);
//	cvShowImage("Quadimg", quadimg);
//	cvWaitKey(0);


	return linevec(pcush, pushangle*CV_PI/180, pushdist).pix2wrk_coords();
}

// Stream operator for action:
ostream& operator<<(ostream& os,const action& a){
    os << "[object:" << a.object<< ", direction: " << a.direction << "]";
    return os;
}
