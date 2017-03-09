/*
 * jvfeatures.cpp
 *
 *  Created on: March 13, 2009
 *      Author: jscholz
 */

#include <iostream>
#include <math.h>
#include <algorithm>
#include <cstdio>
#include "jvfeatures.h"
#include <algorithm>
#include <cstdio>


#ifndef isnan
# define isnan(x) \
  (sizeof (x) == sizeof (long double) ? isnan_ld (x) \
   : sizeof (x) == sizeof (double) ? isnan_d (x) \
   : isnan_f (x))
static inline int isnan_f  (double       x) { return x != x; }
static inline int isnan_d  (double      x) { return x != x; }
static inline int isnan_ld (long double x) { return x != x; }
#endif

#ifndef isinf
# define isinf(x) \
  (sizeof (x) == sizeof (long double) ? isinf_ld (x) \
   : sizeof (x) == sizeof (double) ? isinf_d (x) \
   : isinf_f (x))
static inline int isinf_f  (double       x) { return isnan (x - x); }
static inline int isinf_d  (double      x) { return isnan (x - x); }
static inline int isinf_ld (long double x) { return isnan (x - x); }
#endif

using namespace std;

// Main JVFEATURES definitions

// Constructor for image file
jvfeatures::jvfeatures(const char* filename,  bool show)
{
	showsrcimg = show;
	useROI = false;

	// Create source image from file
	loadFile(filename, show);
}

// Constructor for raw IplImage
jvfeatures::jvfeatures(IplImage *img, bool show) {
	showsrcimg = show;
	useROI = false;

	// set srcimg pointer
//	srcimgBW = cvCreateImage(cvGetSize(img), 8, 1);
//	cvCvtColor(img, srcimgBW, CV_BGR2GRAY); // Convert 3 channel to 1 channel (for now)
	srcimg = cvCreateImage(cvGetSize(img), 8, 3);
	cvCopy(img, srcimg, 0); // Copy in full color

	// Allocate space for other images
	init();

	// Default weights
	featweights = vector<double>(NUM_FUNCS,1);

	reset(showsrcimg);
}

// Constructor for camera capture
jvfeatures::jvfeatures(int device, bool show)
{
  showsrcimg = show;
  srcimg = NULL;
  useROI = false;
  capture = cvCaptureFromCAM(device);
  //capture = cvCreateFileCapture("/home/jscholz/Desktop/LearningOpenCV_Code/LearningOpenCV_Code/tree.avi");

  /* **Can also easily grab frames from an avi file using openCV **
     just didn't implement b/c it needs the char* constructor and i don't need it now:
     capture = cvCaptureFromAVI( avifile );
  */
  if (!capture) {
    fprintf(stderr,"Could not initialize capturing...\n");
    exit(-1);
  }

	frame = 0;
	frame = cvQueryFrame(capture);
	if (!frame) {
		fprintf(stderr,"Failed to grab frame from device\n");
		exit(-1);
	}

	// Create source image from frame
	srcimg = cvCreateImage(cvGetSize(frame), 8, 3);

	// Allocate space for other images
	init();

	// Default weights
	featweights = vector<double>(NUM_FUNCS,1);

  reset(showsrcimg);
}

// Null constructor - assumes we'll be calling loadImage manually to provide images to process
jvfeatures::jvfeatures(bool show)
{
  showsrcimg = show;

  // Default weights
  featweights = vector<double>(NUM_FUNCS,1);

  srcimg = NULL; // important - let's loadImage know whether or not to allocate memory

  useROI = false;
}

jvfeatures::~jvfeatures()
{
  cvDestroyAllWindows();
  cvReleaseMemStorage(&storage);
  cvReleaseImage(&srcimgBW);
  cvReleaseImage(&edgeimg);
  cvReleaseImage(&histimg);
  cvReleaseImage(&houghimg);
  cvReleaseImage(&houghtemp);
  cvReleaseImage(&hullimg);
  cvReleaseImage(&threshimg);
  cvReleaseImage(&contoursimg);
  cvReleaseImage(&quadsimg);
  cvReleaseImage(&segimg);
  cvReleaseImage(&costimg);
}

void jvfeatures::init()
{
	// Create images
	srcimgBW = cvCreateImage(cvGetSize(srcimg), 8, 1);
	edgeimg = cvCreateImage(cvGetSize(srcimg), 8, 1);
	histimg = cvCreateImage(cvSize(320, 200), 8, 3);
	houghimg = cvCreateImage(cvGetSize(srcimg), 8, 3);
	houghtemp = cvCreateImage(cvGetSize(srcimg), 8, 1);
	hullimg = cvCreateImage(cvGetSize(srcimg), 8, 3);
	threshimg = cvCreateImage(cvGetSize(srcimg), 8, 1);
	contoursimg = cvCreateImage(cvGetSize(srcimg), 8, 3);
	quadsimg = cvCreateImage(cvGetSize(srcimg), 8, 3);
	segimg = cvCreateImage(cvGetSize(srcimg), 8, 3);
	costimg = cvCreateImage(cvSize(320, 200), 8, 3);

	// Create storage
	storage = cvCreateMemStorage(0);
}

void jvfeatures::reset(bool show)
{
	showsrcimg = show;

	// Mark all calculations as un-performed
	ran_thresh = false;
	ran_edge = false;
	ran_hull = false;
	ran_hough = false;
	ran_hist = false;
	ran_contours = false;
	ran_quads = false;

	lines.clear();
	quads.clear();

	// Reset all costs
	costvec.clear();
	entropy = 0;
	scatterscore = 0;
	orthoscore = 0;
	charea = 0;
	cost = 0;

	if (showsrcimg == true)
		showimg(srcimg, "Source Image");
}

void jvfeatures::loadFile(const char* filename, bool show)
{
	showsrcimg = show;

	// Allocate space for other images
	if (srcimg==NULL){
		srcimg = cvLoadImage(filename, 1); // should only be here once - 0 for b/w
		init();
	}

	if (!srcimg) {
		cout << "Could not load image " << filename << endl;
		exit(-1);
	} else {
		cout << "loaded image " << filename << endl;
	}

	// Default weights
	featweights = vector<double> (NUM_FUNCS, 1);

	reset(showsrcimg);
}

void jvfeatures::loadImage(IplImage *img, bool show)
{
	showsrcimg = show;

	if (storage != NULL) {
		cleanup();
	}

	if (srcimg==NULL){
		srcimg = cvCreateImage(cvGetSize(img), 8, 3); // should only be here once
		init();
	}

	cvCopy(img,srcimg,0);

	reset(showsrcimg);
}

void jvfeatures::showimg(IplImage* img, const char* title)
{
  cvNamedWindow(title, 0);
  cvShowImage(title, img);
}

void jvfeatures::configROI(int x, int y, int width, int height) {
	roiX = x;
	roiY = y;
	roiW = width;
	roiH = height;
	useROI = true; // Must turn this off manually to stop using rois
}

void jvfeatures::setROI()
{
	cvSetImageROI(srcimg, cvRect(roiX, roiY, roiW, roiH));
	cvSetImageROI(srcimgBW, cvRect(roiX, roiY, roiW, roiH));
	cvSetImageROI(edgeimg, cvRect(roiX, roiY, roiW, roiH));
	cvSetImageROI(houghimg, cvRect(roiX, roiY, roiW, roiH));
	cvSetImageROI(houghtemp, cvRect(roiX, roiY, roiW, roiH));
	cvSetImageROI(contoursimg, cvRect(roiX, roiY, roiW, roiH));
	cvSetImageROI(threshimg, cvRect(roiX, roiY, roiW, roiH));
	cvSetImageROI(quadsimg, cvRect(roiX, roiY, roiW, roiH));
	cvSetImageROI(segimg, cvRect(roiX, roiY, roiW, roiH));
}

void jvfeatures::resetROI()
{
	cvResetImageROI(srcimg);
	cvResetImageROI(srcimgBW);
	cvResetImageROI(edgeimg);
	cvResetImageROI(houghimg);
	cvResetImageROI(houghtemp);
	cvResetImageROI(contoursimg);
	cvResetImageROI(threshimg);
	cvResetImageROI(quadsimg);
	cvResetImageROI(segimg);
}

void jvfeatures::thresh(bool show, int thresh, double wB, double wG, double wR)
{
	/*
	 * This is a wrapper for cvThreshold that allows filtering out individual
	 * channels.  Thresholding only works on grayscale images, but by specifying
	 * how to combine the individual channels with the weights wB,wG,wR it is
	 * possible to uniquely threshold for any desired color (just specify its
	 * rgb value as weights and the grayscale image will be "optimized" for that
	 * color)
	 */
	showthreshimg = show;

	// Turn ROI on for the image-based calculations
	if (useROI)
		setROI();

	  // Allocate individual image  planes.
	  IplImage* r = cvCreateImage(cvGetSize(srcimg), IPL_DEPTH_8U, 1 );
	  IplImage* g = cvCreateImage(cvGetSize(srcimg), IPL_DEPTH_8U, 1 );
	  IplImage* b = cvCreateImage(cvGetSize(srcimg), IPL_DEPTH_8U, 1 );

	  // Split image onto the color planes.
	  cvSplit(srcimg, b, g, r, NULL);

	  // Add weighted rgb values.
	  double wSum = wR + wG + wB;
	  cvAddWeighted(r, wR/wSum, g, wG/wSum, 0.0, srcimgBW);
	  cvAddWeighted(srcimgBW, (wR+wG)/wSum, b, wB/wSum, 0.0, srcimgBW);

	  // To visualize intermediate images:
//	cvNamedWindow("B", 1);
//	cvShowImage("B", b);
//	cvNamedWindow("G", 1);
//	cvShowImage("G", g);
//	cvNamedWindow("R", 1);
//	cvShowImage("R", r);
//	cvWaitKey(0);
//	cvNamedWindow("srcimgBW", 1);
//	cvShowImage("srcimgBW", srcimgBW);

	  // Truncate values above 100.
	  cvThreshold(srcimgBW, threshimg, thresh, 255, CV_THRESH_BINARY);

	  cvReleaseImage(&r);
	  cvReleaseImage(&g);
	  cvReleaseImage(&b);

	// Turn ROI back off
	if (useROI)
		resetROI();

	ran_thresh = true;

	if (showthreshimg)
		draw_threshimg();
}

void jvfeatures::draw_threshimg()
{
	if (ran_thresh == false)
		thresh(false); // warning: don't make this true or we'll get stuck in a loop

	cvNamedWindow("Thresh Image", 1);
	cvShowImage("Thresh Image", threshimg);
}


void jvfeatures::edges(bool show)
{
	showedgeimg = show;

	if (useROI)
		setROI();

	cvCvtColor(srcimg, srcimgBW, CV_BGR2GRAY);
	cvCanny(srcimgBW, edgeimg, 50, 200, 3);

	if (useROI)
		resetROI();

	ran_edge = true;

	if (showedgeimg)
		draw_edgeimg();

}

void jvfeatures::draw_edgeimg()
{
	if (ran_edge == false)
		edges(false); // warning: don't make this true or we'll get stuck in a loop

	// Silly to separate, since hard drawing work is already done by cvCanny, but still...
	cvNamedWindow("Edge Image", 1);
	cvShowImage("Edge Image", edgeimg);
}

void jvfeatures::houghlines(bool show, int method)
{
	showhoughimg = show;
	houghmethod = method;
	int houghthresh = 15;
	int minlen = 40; // 40 or 110 to tweak for simSplinter
	int minsep = 15;
	CvSeq* l;

	// Dependencies:
	if (ran_edge == false)
		edges(false);

	// Initialize
	//	houghtemp = cvCloneImage(edgeimg); // Need temp container since Probabilistic method modifies image
	cvCopy(edgeimg, houghtemp);

	// Turn ROI on for the image-based calculations
	if (useROI)
		setROI();

	// Calculate the lines on the canny-processed image:
	if (method == 0) { // Standard
			l = cvHoughLines2(houghtemp, storage, CV_HOUGH_STANDARD, 1, CV_PI / 180, 100, 0, 0);
			for (int i = 0; i < MIN(l->total,100); ++i) {
				double* line = (double*) cvGetSeqElem(l, i);
				double rho = line[0];
				double theta = line[1];
				CvPoint pt1, pt2;
				double a = cos(theta), b = sin(theta);
				double x0 = a * rho, y0 = b * rho;
				pt1.x = cvRound(x0 + 1000* (- b));
				pt1.y = cvRound(y0 + 1000*(a));
				pt2.x = cvRound(x0 - 1000*(-b));
				pt2.y = cvRound(y0 - 1000*(a));
				lines.push_back(linevec(pt1.x, pt1.y, pt2.x, pt2.y));
			}
	} else if (method == 1) { // Probabilistic
			l = cvHoughLines2(houghtemp, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI / 180, houghthresh, minlen, minsep);
			for (int i = 0; i < MIN(l->total,100); ++i) {
				CvPoint* line = (CvPoint*)cvGetSeqElem(l,i);
				lines.push_back(linevec(line[0].x, line[0].y, line[1].x, line[1].y));
			}
	}

	// Turn ROI back off & fix lines to account for roi offset
	if (useROI) {
		resetROI();
		for(int i = 0; i < lines.size(); ++i) {
			lines[i].x1 += roiX;
			lines[i].y1 += roiY;
			lines[i].x2 += roiX;
			lines[i].y2 += roiY;
	    }
	}

	ran_hough = true;

	if (showhoughimg)
		draw_houghimg();
}

void jvfeatures::draw_houghimg()
{
	if (ran_hough == false)
		houghlines(false, houghmethod); // warning: don't make this true or we'll get stuck in a loop

	cvCvtColor(houghtemp, houghimg, CV_GRAY2BGR );

	// Draw lines on houghimg (up to 100 lines)
	for (int i = 0; i < MIN(lines.size(),100); ++i) {
		CvPoint pt1, pt2;
		pt1.x = lines[i].x1;
		pt1.y = lines[i].y1;
		pt2.x = lines[i].x2;
		pt2.y = lines[i].y2;
		cvLine(houghimg, pt1, pt2, CV_RGB(255,0,0), 3, CV_AA, 0);
	}

	cvNamedWindow("Hough Image", 1);
	cvShowImage("Hough Image", houghimg);
}

void jvfeatures::segment(bool show, int method)
{
  // make the funky "termination condition" structure that opencv uses for iterative algos:
  CvTermCriteria termcrit = cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 5, 1);

  // segment according to method requested
  switch (method) {
  case PYRMEANSHIFT: {
    double spatialRadius = 2;
    double colorRadius = 40;
    int level = 3;
    cvPyrMeanShiftFiltering(srcimg, segimg, spatialRadius, colorRadius, level, termcrit);
    }
    break;

  case PYRAMID: {
    comp = NULL;
    int level = 50;
    int threshold1 = 4;
    int threshold2 = 200;
    cvPyrSegmentation(srcimg, segimg, storage, &comp, level, threshold1, threshold2);
    }
    break;

    //TODO implement the rest of these
  }

  if (show) {
	cvNamedWindow("Seg Image", 1);
	cvShowImage("Seg Image", segimg);
  }
}

void jvfeatures::calchist(bool show, int nbins)
{
	/*
	 * Currently collapses all color channels to grayscale, and computes
	 * histogram in that space.  If a fancier method is desired, see
	 * O'reilly OpenCV book p203
	 */

	showhistimg = show;

	// Manually specify range (hack - fix for color channels or widen range)
	float ranges[] = { 0, 256 };
	float* channel_ranges = { ranges };

	// Initialize and compute the histogram
	hist = cvCreateHist(1, &nbins, CV_HIST_ARRAY, &channel_ranges, 1);

	// Turn ROI on for the image-based calculations
	if (useROI)
		setROI();

	cvCvtColor(srcimg, srcimgBW, CV_BGR2GRAY);
	cvCalcHist(&srcimgBW, hist, 0, NULL);

	// Turn ROI back off
	if (useROI)
		resetROI();

	// Convert to probability - useful for calculating entropy
	cvNormalizeHist(hist, 1);

	int v;
	for (int i = 0; i < nbins; ++i) {
		histvec.push_back(cvGetReal1D(hist->bins, i)); // push the new probability
	}

	ran_hist = true;

	if (showhistimg == true)
		draw_histimg(nbins);
}

void jvfeatures::draw_histimg(int nbins)
{
	if (ran_hist == false)
		calchist(false, 64); // warning: don't make this true or we'll get stuck in a loop

	cvZero(histimg);

	// Scale and configure histimg
	int bin_w = histimg->width / nbins;
	float max_value = 0;
	cvGetMinMaxHistValue(hist, NULL, &max_value, NULL, NULL);
	cvSet(histimg, cvScalarAll(255), 0);

	// Draw rectangle bars for each pixel value
	double windowscale = histimg->height / max_value;
	int v;
	for (int i = 0; i < nbins; ++i) {
		histvec.push_back(cvGetReal1D(hist->bins, i)); // push the new probability
		v = cvRound(histvec[i] * windowscale); // height of hist bar in pixels
		cvRectangle(histimg, cvPoint(i * bin_w, histimg->height), cvPoint((i
				+ 1) * bin_w, histimg->height - v), cvScalarAll(0), -1, 8, 0);
	}

	cvNamedWindow("Hist Image", 1);
	cvShowImage("Hist Image", histimg);
}

void jvfeatures::histentropy()
{
  if (ran_hist == false)
    calchist(false, 64);

  for (int i=0; i<histvec.size(); ++i)
    if (histvec[i] != 0) {
      entropy += histvec[i]*log(1/histvec[i]);
    }
}

void jvfeatures::linescatter()
{
	// Score for parallel or orthogonal relationships among houghlines
	if (ran_hough == false)
		houghlines(false, 1);

	// Make all pairwise comparisons, and
	double theta;
	for (int i = 0; i < lines.size(); ++i) {
		double ref = lines[i].getAngleULO(); // want pixel version of line slope
		for (int j = i + 1; j < lines.size(); ++j) {
			// Extract line, compute slope, and compare to current - don't care about angles > 180
			theta = (ref - lines[j].getAngleULO());

			// Pass theta through dist2axis, which produces that "^"
			// function which in this case indicates how far from square
			// the relationship between the two lines is
			scatterscore += dist2axis(theta);
		}
	}
	// And finally, normalize for the number of lines
	scatterscore = scatterscore / lines.size();
}

void jvfeatures::lineortho()
{
	// Score for how parallel houghlines are to principle axes of image
	if (ran_hough == false)
		houghlines(false, 1);

	double theta;
	for (int i = 0; i < lines.size(); ++i) {
		// Extract line, compute slope, and compare to current - don't care about angles > 180
		theta = lines[i].getAngleULO();
		orthoscore += dist2axis(theta);
	}
	orthoscore = orthoscore / lines.size(); // normalize by #lines
}

void jvfeatures::convexhull(bool show) {
	showhullimg = show;

	/* thoughts:
	 * need an array of cvpoints to pass to convexhull2, so probably the best
	 * thing to do is to specify a bounding box (just inside the table, for example)
	 * and loop through it for all white pixels in the edge image.  After getting
	 * the return from convexhull (which is points on the hull), save in a nice format
	 * (perferably learn cvseq and others, otherwise a vector or custom struct).
	 * Then optionally make an image to show
	 */

	//TODO add a toggle between edge or contour img

	// Dependencies:
	//if (ran_contours == false)
	draw_contoursimg(false);

	CvPoint pt;
	CvSeq* ptseq = cvCreateSeq(CV_SEQ_KIND_GENERIC | CV_32SC2,
			sizeof(CvContour), sizeof(CvPoint), storage);
	CvSeq* hull;

	// loop through image the fast way and add points to ptseq
	int pixval;
	// concatenate channels end to end
	for (int ch = 0; ch < contoursimg->nChannels; ch++) {
		for (int y = 0; y < contoursimg->height; y++) {
			uchar* ptr = (uchar*) (contoursimg->imageData + y * contoursimg->widthStep);
			for (int x = 0; x < contoursimg->width; x++) {
				pixval = ptr[contoursimg->nChannels * x + ch];
				if (pixval != 0) {
					pt.x = x;
					pt.y = y;
					cvSeqPush(ptseq, &pt);
				}
			}
		}
	}

	cvhull = cvConvexHull2(ptseq, 0, CV_CLOCKWISE, 0);

	ran_hull = true;

	if (showhullimg == true)
		draw_hullimg();
}

void jvfeatures::draw_hullimg()
{
	if (ran_hull == false)
		convexhull(false); // warning: don't make this true or we'll get stuck in a loop

	// compute hull image:
	//cvCvtColor(edgeimg, hullimg, CV_GRAY2BGR);
	cvCopy(contoursimg, hullimg);

	if (cvhull == NULL)
		return;

	int hullcount = hullcount = cvhull->total;
	// initialize pt to last element so we can connect the 1st point to it
	CvPoint pt = **CV_GET_SEQ_ELEM(CvPoint*, cvhull, hullcount - 1);

	// Loop through hull points and connect with lines
	for (int i = 0; i < hullcount; i++) {
		CvPoint p = **CV_GET_SEQ_ELEM(CvPoint*, cvhull, i);
		cvLine(hullimg, pt, p, CV_RGB( 0, 255, 0 ), 1, CV_AA, 0);
		pt = p;
	}

	cvNamedWindow("Hull Image", 1);
	cvShowImage("Hull Image", hullimg);
}

void jvfeatures::findContours(bool show)
{
	showcontoursimg = show;

	// Dependencies:
	if (ran_thresh == false)
		thresh(false);

    // Optional: down-scale and upscale the image to filter out the noise *needs div-by-2 roi
	// IplImage* pyr = cvCreateImage( cvSize(), 8, 3 );
	// cvPyrDown( timg, pyr, 7 );
	// cvPyrUp( pyr, timg, 7 );

	// make threshimg for each color channel and loop, adding

	num_contours = cvFindContours(threshimg, storage, &contours,
			sizeof(CvContour), CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
	//cout << "Found " << num_contours << " contours" << endl;

	// for splinter: Go down to the interior contours to find the blocks
	if (contours->v_next != NULL)
		contours = contours->v_next;

	ran_contours = true;

	if (showcontoursimg)
		draw_contoursimg();
}

void jvfeatures::draw_contoursimg(bool show)
{
	if (ran_contours == false)
		findContours(false); // warning: don't make this true or we'll get stuck in a loop

	cvZero(contoursimg);

	int n = 0;
	int minlength = 150; // 150 - picked to prevent code from finding ee in simulator
	for (CvSeq *c = contours; c != NULL; c = c->h_next) {
		//printf("found %d points in polygon %d\n", c->total, n);
		//printf("Contour length = %2.2f\n",cvContourPerimeter(c));

		// * minlength is only a threshold for DRAWING *
		if (cvContourPerimeter(c) > minlength) {
			CvPoint* pt0 = CV_GET_SEQ_ELEM( CvPoint, c, 0 );
			CvPoint* ptlast = pt0;
			for (int i = 0; i < c->total; i++) {
				CvPoint* pt = CV_GET_SEQ_ELEM( CvPoint, c, i );
				//printf("%d: (%d,%d)\n",n , pt->x, pt->y );
				cvLine(contoursimg, *ptlast, *pt, cvScalar(255 - 100* n , 0 + 100* n , 0), 3);
				ptlast = pt;
			}
			cvLine(contoursimg, *pt0, *ptlast, cvScalar(255 - 100* n , 0 + 100*
					n , 0), 3);
		}
		n++;
	}

	// Need an extra show so that cvhull can generate the image for it's purposes
	// without making us display it
	if (show) {
		cvNamedWindow("Contours Image", 1);
		cvShowImage("Contours Image", contoursimg);
	}
}

void jvfeatures::findQuads(bool show, int minlength, int obstPerimThresh)
{
	showquadsimg = show;

	// Dependencies:
	if (ran_contours== false)
		findContours(false);

	// Optional: do polygon approximation of all contours before drawing
	polygons = cvApproxPoly(contours, sizeof(CvContour), storage, CV_POLY_APPROX_DP,
				cvContourPerimeter(contours) * 0.02, 1);

	for (CvSeq *poly = polygons; poly != NULL; poly = poly->h_next) {
		if (cvContourPerimeter(poly) > minlength) {

			CvBox2D box = cvMinAreaRect2(poly,storage);
			CvPoint2D32f box_vtx[4];
			cvBoxPoints(box, box_vtx);

			// add points to polygon
			quad q;
			quads.push_back(q); // push back empty polygon and fill it
			//TODO rotate these points to keep assignment rotation invariant
			quads.back().pt.push_back(point(box_vtx[0].x, box_vtx[0].y));
			quads.back().pt.push_back(point(box_vtx[1].x, box_vtx[1].y));
			quads.back().pt.push_back(point(box_vtx[2].x, box_vtx[2].y));
			quads.back().pt.push_back(point(box_vtx[3].x, box_vtx[3].y));
			quads.back().center = point(box.center.x,box.center.y);
			//TODO: should switch width and height if angle>45 so that width is always along x axis
			quads.back().width = box.size.width;
			quads.back().height = box.size.height;
			// tweak for splinter: store angle wrt long axis
			if (box.size.width < box.size.height)
				quads.back().angle = box.angle;
			else
				quads.back().angle = box.angle - 90;

			// Obstacle hack (should make color-based one day) [~150 for erasers, ~110 for blocks]
			if (quads.back().width + quads.back().height > obstPerimThresh)
				quads.back().isObstacle = true;
			else
				quads.back().isObstacle = false;
		}
	}

	// Sort by perimeter of object (see operator "<" above)
	sort(quads.begin(), quads.end());

	ran_quads = true;

	if (showquadsimg)
		draw_quadsimg();
}

void jvfeatures::draw_quadsimg()
{
	if (ran_quads== false)
		findQuads(false); // warning: don't make this true or we'll get stuck in a loop

	cvZero(quadsimg);

	for (int i=0; i<quads.size(); i++) {
		cvLine(quadsimg, cvPoint(quads[i].pt[0].x,
				quads[i].pt[0].y), cvPoint(quads[i].pt[1].x,
				quads[i].pt[1].y), cvScalar(255 - 100* i ,
				0 + 100* i , 0), 3);
		cvLine(quadsimg, cvPoint(quads[i].pt[1].x,
				quads[i].pt[1].y), cvPoint(quads[i].pt[2].x,
				quads[i].pt[2].y), cvScalar(255 - 100* i,
				0 + 100*i , 0), 3);
		cvLine(quadsimg, cvPoint(quads[i].pt[2].x,
				quads[i].pt[2].y), cvPoint(quads[i].pt[3].x,
				quads[i].pt[3].y), cvScalar(255 - 100*i ,
				0 + 100*i , 0), 3);
		cvLine(quadsimg, cvPoint(quads[i].pt[3].x,
				quads[i].pt[3].y), cvPoint(quads[i].pt[0].x,
				quads[i].pt[0].y), cvScalar(255 - 100*i ,
				0 + 100*i , 0), 3);

//		for (int j=0; j<quads[i].pt.size(); j++) {
//			printf("x,y = %d, %d\n", quads[i].pt[j].x, quads[i].pt[j].y);
//		}
	}

	cvNamedWindow("Quads Image", 1);
	cvShowImage("Quads Image", quadsimg);

}

void jvfeatures::clutterarea(point gpt) {
	// A cost feature representing the size and location of the region with objects in it.
	// Derived from cvhull, and produces chperimeter, chcentroid, and goaldist as intermediates

	if (ran_hull == false)
		convexhull(false);

	if (cvhull == NULL) {
		charea = 0;
		return;
	}

	goalpt = gpt;

	/* area algorithm: recursively break sequence of points into triangles using 3 consecutive points,
	 * summing area we go.  when back at start, recurse on new polygon of points using bases of previous
	 * triangles.
	 */

	chperim = 0;
	double xsum = 0;
	double ysum = 0;
	CvPoint *plast = *CV_GET_SEQ_ELEM(CvPoint*, cvhull, 0);
	for (int i = 0; i < cvhull->total; i++) {
		CvPoint *p= *CV_GET_SEQ_ELEM(CvPoint*, cvhull, i);
		chperim += linevec(p->x, p->y, plast->x, plast->y).length();
		xsum += p->x;
		ysum += p->y;
		plast = p;
	}

	// Compute centroid of convex hull
	chcentroid = point(xsum/cvhull->total,ysum/cvhull->total);

	// Compute distance between centroid and goalpt
	goaldist = linevec(goalpt.x, goalpt.y, chcentroid.x, chcentroid.y).length();

	// Normalize by roi/window perimeter
	double roiperim;
	if (useROI) {
		roiperim = 2*roiW + 2*roiW;
	}
	else {
		roiperim = 2*srcimg->width + 2*srcimg->height;
	}

	// Normalize by roi/window diameter
	double roidiag;
	if (useROI) {
		roidiag = sqrt(pow(roiW,2.0) + pow(roiH,2.0));
	}
	else {
		roidiag = sqrt(pow(srcimg->width,2.0) + pow(srcimg->height,2.0));
	}

	// Set charea to normalized costs (linear version)
//	charea = (goaldist/roidiag + chperim/roiperim); // use dist and perim features
//	charea = chperim/roiperim; // use just perim feature
//	charea = goaldist/roidiag; // use just dist feature

	// Set charea to normalized costs (quadratic version)
	charea = (sqrt(goaldist)/sqrt(roidiag) + sqrt(chperim)/sqrt(roiperim)); // use dist and perim features
}

//TODO write object seg code

void jvfeatures::cluttercost(bool show)
{
	showcostimg = show;

	// Weighted sum of feature scores
	for (int i = 0; i < NUM_FUNCS; ++i) {
		// If feature is requested for cluttercost, be sure it was calculated
		if (featweights[i] != 0) {
			costvec.push_back(featweights[i] * get_featval(i));
			cost += costvec.back();
		}
		else {
			costvec.push_back(featweights[i]); // so it shows an empty bar here
		}

	}
	costvec.push_back(cost); // last element is cost itself

	if (showcostimg)
		draw_costimg();
}

void jvfeatures::draw_costimg()
{
	// Use openCV to draw cost bars for each feature
	double ranges[] = { 0, 256 };
	double* channel_ranges = { ranges };

	// Set up text drawing
	double hScale = 0.55;
	double vScale = 0.55;
	int lineWidth = 1;
	cvInitFont(&font, CV_FONT_HERSHEY_COMPLEX_SMALL, hScale, vScale, 0, lineWidth);

	// Set up bar colors
	CvScalar barcolors[NUM_FUNCS];
	barcolors[ENTROPY] = CV_RGB(255,0,0);
	barcolors[LINESCATTER] = CV_RGB(0,255,0);
	barcolors[LINEORTHO] = CV_RGB(100,100,255);
	barcolors[CHAREA] = CV_RGB(225,195,0);
	barcolors[NUM_FUNCS] = CV_RGB(225,111,0); // this one is for the weighted cost total


	// Create image for drawing bars depicting the cost array
	cvSet(costimg, cvScalarAll(255), 0);
	int hborder = 10;
	int vborder = 40;
	int plotwidth = costimg->width - hborder;
	int plotheight = costimg->height - vborder;

	// Scale and configure costimg
	static double max_costbar = 0;
	int bin_w = plotwidth / costvec.size();
	for (int i = 0; i < costvec.size(); ++i)
		if (costvec[i] > max_costbar)
			max_costbar = costvec[i];
	double windowscale = plotheight / max_costbar;

	// Write labels for each bar
	cvPutText(costimg, "Entropy", cvPoint(0* bin_w , costimg->height - vborder / 5), &font, cvScalar(0, 0, 0));
	cvPutText(costimg, "Scatter", cvPoint(1* bin_w , costimg->height - vborder / 5), &font, cvScalar(0, 0, 0));
	cvPutText(costimg, "Ortho", cvPoint(2* bin_w , costimg->height - vborder / 5), &font, cvScalar(0, 0, 0));
	cvPutText(costimg, "Area", cvPoint(3* bin_w , costimg->height - vborder / 5), &font, cvScalar(0, 0, 0));
	cvPutText(costimg, "Cost", cvPoint(4* bin_w , costimg->height - vborder / 5), &font, cvScalar(0, 0, 0));

	for (int i = 0; i < costvec.size(); ++i) {
		int v = cvRound(costvec[i] * windowscale); // height of cost bar in pixels
		//cout << "cost " << i << " = " << costvec[i] << endl; // ##
		cvRectangle(costimg, cvPoint(i * bin_w, plotheight + vborder / 2),
				cvPoint((i + 1) * bin_w, plotheight - v + vborder / 2),
				barcolors[i], -1, 8, 0);
	}

	cvNamedWindow("Cost Image", 1);
	cvShowImage("Cost Image", costimg);
}

void jvfeatures::singlecapture(bool show)
{
	showsrcimg = show;

	// Grab frame from device
//	frame = cvQueryFrame(capture); // This should work!  Major bug gripe here!! have to waste 130ms!!!
// there should be a fix, as livecapture is never more than 1 frame behind at 20 hz or whatever...
	cvGrabFrame(capture);
	usleep(130000);
	frame=cvRetrieveFrame(capture);

	if (!frame)
		exit(0);

	cvCopy(frame, srcimg, 0); // Copy in full color

	// Reset previous whatever
	reset(showsrcimg);
}

void jvfeatures::livecapture(bool saveMovie)
{
	if (saveMovie) {
	    double fps = cvGetCaptureProperty(capture, CV_CAP_PROP_FPS);
	    CvSize size = cvSize(
	        (int)cvGetCaptureProperty( capture, CV_CAP_PROP_FRAME_WIDTH),
	        (int)cvGetCaptureProperty( capture, CV_CAP_PROP_FRAME_HEIGHT));
	    writer = cvCreateVideoWriter(  // On linux Will only work if you've installed ffmpeg development files correctly,
	           "out.avi",                               // otherwise segmentation fault.  Windows probably better.
	           CV_FOURCC('D','X','5','0'),
	           //CV_FOURCC('M','J','P','G'),
	           //CV_FOURCC('D', 'I', 'V', 'X'),
//	           CV_FOURCC('F', 'L', 'V', '1'),
	           fps,
	           size
	       );
	}

	while (1) {
		// Grab frame from device
		frame = cvQueryFrame(capture);
		if (!frame)
			exit(0);

		if (saveMovie)
			cvWriteToAVI(writer, frame);

		//srcimg->origin = frame->origin; // necessary?
		cvCopy( frame, srcimg, 0 ); // do a plain copy if everything is in 3 channel

		// Reset previous whatever
		reset(showsrcimg);

		// Allow user to click on the image to extract pixel location (for calibrating, mainly)
		select_pixel();

		// Manually run image-generating methods so visualization can be toggled
		if (showedgeimg)
			edges(showedgeimg);
		if (showhullimg)
			convexhull(showhullimg);
		if (showhistimg)
			calchist(showhistimg, 64);
		if (showhoughimg)
			houghlines(showhoughimg, 1);
		if (showthreshimg)
				thresh(showthreshimg);
		if (showcontoursimg)
			findContours(showcontoursimg);
		if (showquadsimg)
			findQuads(showquadsimg);

		// Calculate cost based on current feature weights
		if (showcostimg)
			cluttercost(showcostimg);

		// Poll for feedback from user
		int c;
		c = cvWaitKey(10);
		if ((char) c == 'q') {
			cvReleaseCapture( &capture );
			exit(0);
			break;
		}
		switch ((char) c) {
		case 's':
			showsrcimg = !showsrcimg;
			if (!showsrcimg)
				cvDestroyWindow("Source Image");
			break;

		case 'e':
			showedgeimg = !showedgeimg;
			if (!showedgeimg)
				cvDestroyWindow("Edge Image");
			break;

		case 'f':
			showhoughimg = !showhoughimg;
			if (!showhoughimg)
				cvDestroyWindow("Hough Image");
			break;

		case 'h':
			showhistimg = !showhistimg;
			if (!showhistimg)
				cvDestroyWindow("Hist Image");
			break;

		case 'v':
			showhullimg = !showhullimg;
			if (!showhullimg)
				cvDestroyWindow("Hull Image");
			break;

		case 't':
			showthreshimg = !showthreshimg;
			if (!showthreshimg)
				cvDestroyWindow("Thresh Image");
			break;

		case 'C':
			showcontoursimg= !showcontoursimg;
			if (!showcontoursimg)
				cvDestroyWindow("Contours Image");
			break;

		case 'Q':
			showquadsimg = !showquadsimg;
			if (!showquadsimg)
				cvDestroyWindow("Quads Image");
			break;

		case 'c':
			showcostimg = !showcostimg;
			if (!showcostimg)
				cvDestroyWindow("Cost Image");
			break;
		}

		// Clear storage and other tidy things
		cleanup();
	}
	cvReleaseVideoWriter(&writer);
	cvReleaseCapture(&capture);
	cvReleaseImage(&frame);
}

void jvfeatures::select_pixel()
{
  clickSeq = cvCreateSeq(CV_SEQ_ELTYPE_POINT | CV_32SC2, sizeof(CvSeq), sizeof(CvPoint), storage);
  cvSetMouseCallback("Source Image", on_mouse, this);
}

void jvfeatures::clickExtras()
{
  /*
   * Optional extra in-class code to run during mouse callbacks
   */

   uchar* yptr = (uchar*) (srcimg->imageData + mousept.y * srcimg->widthStep);
   int r = yptr[3*mousept.x+2];
   int g = yptr[3*mousept.x+1];
   int b = yptr[3*mousept.x+0];
   printf("you clicked at (%d,%d) RGB:[%d,%d,%d]\n",mousept.x,mousept.y,r,g,b);

   // Uncomment to draw as we go
//   CvPoint* pLast = (CvPoint*)cvGetSeqElem(clickSeq, clickSeq->total - 1);
//   cvLine(srcimg, *pLast, mousept, cvScalar(0,255,0), 1, 8, 0);
//   showimg(srcimg, "Source Image");

  // for now, I'll use this to compute some running stats on the points that have been clicked
  cout << clickSeq->total << " points clicked so far" << endl;

  // Calculate mean and variance of each channel

  // Mean:
  double rSum = 0;
  double gSum = 0;
  double bSum = 0;
  for( int i=0; i<clickSeq->total; ++i ) {
    CvPoint* p = (CvPoint*)cvGetSeqElem(clickSeq, i);
    uchar* yptr = (uchar*) (srcimg->imageData + p->y * srcimg->widthStep);
    int r = yptr[3*p->x+2];
    int g = yptr[3*p->x+1];
    int b = yptr[3*p->x+0];
    rSum += r;
    gSum += g;
    bSum += b;
  }
  double rMean = rSum/clickSeq->total;
  double gMean = gSum/clickSeq->total;
  double bMean = bSum/clickSeq->total;

  // Variance:
  double rSSD = 0;
  double gSSD = 0;
  double bSSD = 0;
  for( int i=0; i<clickSeq->total; ++i ) {
    CvPoint* p = (CvPoint*)cvGetSeqElem(clickSeq, i);
    uchar* yptr = (uchar*) (srcimg->imageData + p->y * srcimg->widthStep);
    int r = yptr[3*p->x+2];
    int g = yptr[3*p->x+1];
    int b = yptr[3*p->x+0];
    rSSD += pow(r - rMean,2);
    gSSD += pow(g - gMean,2);
    bSSD += pow(b - bMean,2);
  }
  double rVar = rSSD/clickSeq->total;
  double gVar = gSSD/clickSeq->total;
  double bVar = bSSD/clickSeq->total;

  printf("Means: (%2.2f,%2.2f,%2.2f)\n",rMean,gMean,bMean);
  printf("Variances: (%2.2f,%2.2f,%2.2f)\n",rVar,gVar,bVar);

  cout << endl;
}

// --------------------- Utilities ---------------------

static void on_mouse(int event, int x, int y, int flags, void* object)
{
	jvfeatures* ob = (jvfeatures*) object;
    if(!ob->srcimg)
        return;

	switch (event) {
	case CV_EVENT_LBUTTONDOWN: {
		ob->clicked = true;
		ob->mousept = cvPoint(x, y);
		cvSeqPush(ob->clickSeq, (void*) &ob->mousept);
		ob->clickExtras();
		break;
	}

	case CV_EVENT_LBUTTONUP: {
		ob->clicked = false;
		break;
	}

	case CV_EVENT_MOUSEMOVE: {
		if (ob->clicked) {
			ob->mousept = cvPoint(x, y);
			cvSeqPush(ob->clickSeq, (void*) &ob->mousept);
			ob->clickExtras();
		}
	}
	}

//    if (event == CV_EVENT_LBUTTONDOWN) {
//    	ob->clicked = true;
//    	ob->mousept = cvPoint(x,y);
//	cvSeqPush(ob->clickSeq, (void*)&ob->mousept);
//	ob->clickExtras();
//    }
//
//    if (event == CV_EVENT_LBUTTONUP){
//    	ob->clicked = false;
//	//ob->mousept = cvPoint(x,y);
//	//cvSeqPush(ob->clickSeq, (void*)&ob->mousept);
//	//ob->clickExtras();
//    }
}

double jvfeatures::dist2axis(double theta)
{
  // Returns distance of line to nearest axis, which can be used to
  // indicate orthagonality of lines: piecewise linear with max at 45
  // degrees and minima at 0 and 90.

  double d2lower = fabs(fmod((double)theta,(double)CV_PI/2.f));  //

  double d = min( d2lower, fabs(CV_PI/2. - d2lower) ); //

  return d;
}

double jvfeatures::get_featval(int i)
{
  // Could do this with func pointers, but i'm lazy right now
  switch(i) {
  case ENTROPY:
    histentropy();
    return entropy;
    //break;

  case LINESCATTER:
    linescatter();
    return scatterscore;
    //break;

  case LINEORTHO:
    lineortho();
    return orthoscore;
    //break;

  case CHAREA:
    clutterarea();
    return charea;
    //break;
  }
}

void jvfeatures::cleanup() {
	// Whatever it turns out I need to do after each frame to keep shit cool

	histvec.clear();
	quads.clear();
	cvClearMemStorage(storage); // might break things
}

