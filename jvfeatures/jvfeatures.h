/*
 * jvfeatures.h
 *
 *  Created on: March 13, 2009
 *      Author: jscholz
 */

#ifndef JVFEATURES_H_
#define JVFEATURES_H_


#include "jvtypes.h"

using namespace std;

/* Overview: This class wraps up some standard opencv algorithms,
 as well as adding a few simple calculations of my own, for the
 purpose of extracting features for robot control.  The general
 organization is around a bunch of public member variables
 describing various costs or features, which become populated by
 public methods.  The purpose of this, as opposed to returning
 the feature values to the user, is that these calculations are
 cumulative, and this system makes it trivial to ensure that all
 calculations are performed at most once for each frame.  */

class jvfeatures {
public:

	// Initialization
	bool showsrcimg;
	jvfeatures(const char* filename, bool show = true); // single image mode constructor
	jvfeatures(IplImage *img, bool show = true); // raw IplImage constructor
	jvfeatures(int device, bool show = true); // streaming video mode constructor
	jvfeatures(bool show = true); // null constructor

	void init();
	void reset(bool show = true);
	void loadFile(const char* filename, bool show = false);
	void loadImage(IplImage *img, bool show = false); // manually load a new image

	~jvfeatures();

	// For everyone
	enum feat_functions {
		ENTROPY,
		LINESCATTER,
		LINEORTHO,
		CHAREA,
		NUM_FUNCS
	};
	CvFont font;
	CvMemStorage* storage;
	CvCapture* capture;
	IplImage* srcimg;		// 3 channel source image
	IplImage* srcimgBW; 	// 1 channel version of srcimg
	int roiX, roiY, roiW, roiH;
	bool useROI;
	void setROI();
	void resetROI();
	void configROI(int x = 0, int y = 0, int width = 640, int height = 480);
	void showimg(IplImage* img, const char* title = "Image");

	// For Threshold image
	IplImage* threshimg;
	bool ran_thresh;
	bool showthreshimg;
	void draw_threshimg();
	void thresh(bool show = true, int thresh = 45, double wB = 1, double wG = 1, double wR = 1); // 70 for blocks, 150 (110?) for sim

	// For Canny Edge
	bool showedgeimg;
	bool ran_edge;
	IplImage* edgeimg;
	void draw_edgeimg();
	void edges(bool show = true);

	// For houghlines
	vector<linevec> lines;
	IplImage* houghimg;
	IplImage* houghtemp;
	bool showhoughimg;
	bool ran_hough;
	int houghmethod;
	void draw_houghimg();
	void houghlines(bool show = true, int method = 1);

	// For segmentation
	enum segtype {
	  PYRMEANSHIFT,
	  PYRAMID,
	  WATERFALL,
	};
	CvSeq* comp; // PYRAMID needs this
	IplImage *segimg;
	void segment(bool show = true, int method = PYRMEANSHIFT);

	// For hist
	CvHistogram *hist;
	IplImage* histimg;
	std::vector<double> histvec;
	bool showhistimg;
	bool ran_hist;
	void draw_histimg(int nbins);
	void calchist(bool show = true, int nbins = 64);

	// For entropy
	double entropy;
	void histentropy();

	// For linescatter
	double scatterscore;
	void linescatter();

	// For lineortho
	double orthoscore;
	void lineortho();

	// For convexhull
	IplImage* hullimg;
	bool showhullimg;
	bool ran_hull;
	CvSeq* cvhull;
	void draw_hullimg();
	void convexhull(bool show = true);

	// For findContours
	IplImage* contoursimg;
	int num_contours;
	bool showcontoursimg;
	bool ran_contours;
	CvSeq* contours; // full contour
	void draw_contoursimg(bool show = true);
	void findContours(bool show = true);

	// For findQuads
	IplImage* quadsimg;
	CvSeq* polygons; // polygon approximation of contour
	vector<quad> quads;
	bool showquadsimg;
	bool ran_quads;
	void draw_quadsimg();
	void findQuads(bool show = true, int minlength = 55, int obstPerimThresh = 110); // 35 picked for ee in srlib

	// For clutterarea
	double charea;
	double chperim;
	double goaldist;
	point goalpt;
	point chcentroid;
	void clutterarea(point gpt = point(250,250));

	// For cluttercost
	IplImage* costimg;
	double cost;
	std::vector<double> featweights;
	bool showcostimg;
	//bool ran_cost;
	std::vector<double> costvec; // just to have a convenient representation of costs
	void draw_costimg();
	void cluttercost(bool show = true);

	// For singlecapture
	void singlecapture(bool show);

	// For livecapture
	IplImage* frame;
	CvVideoWriter* writer;
	void livecapture(bool saveMovie = false);

	// For getting image info under the mouse pointer
	CvPoint mousept;
	bool clicked;
	CvSeq *clickSeq;
	void clickExtras();
	void select_pixel();

	/********************* Utilities **********************/
	double get_featval(int i);
	void cleanup();

	static double dist2axis(double theta);
};

// This has to be a static to function as a callback
static void on_mouse(int event, int x, int y, int flags, void* object);

#endif /* JVFEATURES_h_ */
