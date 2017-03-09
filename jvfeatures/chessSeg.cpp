/*
 * chessSeg.cpp
 *
 *  Created on: Nov 24, 2009
 *      Author: jscholz
 */

#include "jvfeatures.h"
#include <iostream>
#include <math.h>

using namespace std;

void segmentFromStats(IplImage *srcimg, IplImage *output);

typedef struct {
	double meanR;
	double meanG;
	double meanB;
	double varR;
	double varG;
	double varB;
} colorStats;

colorStats brownPiece;
colorStats whitePiece;
colorStats whiteSquare;
colorStats greenSquare;

int main(int argc, char *argv[]) {
	if (argc == 2) {
		const char* filename;
		filename = argv[1];

		cout << filename << endl;


		brownPiece.meanR = 62.43;
		brownPiece.meanG = 46.40;
		brownPiece.meanB = 30.22;
		brownPiece.varR = 210.74;
		brownPiece.varG = 133.51;
		brownPiece.varB = 86.08;

		whitePiece.meanR = 122.74;
		whitePiece.meanG = 95.53;
		whitePiece.meanB = 56.72;
		whitePiece.varR = 458.90;
		whitePiece.varG = 352.61;
		whitePiece.varB = 228.42;

		whiteSquare.meanR = 159.25;
		whiteSquare.meanG = 149.65;
		whiteSquare.meanB = 138.93;
		whiteSquare.varR = 11.67;
		whiteSquare.varG = 11.36;
		whiteSquare.varB = 24.06;

		greenSquare.meanR = 25.83;
		greenSquare.meanG = 38.44;
		greenSquare.meanB = 27.63;
		greenSquare.varR = 4.82;
		greenSquare.varG = 1.93;
		greenSquare.varB = 7.77;

		jvfeatures *jvf = new jvfeatures(filename, true);

		IplImage *segimg = cvCreateImage(cvGetSize(jvf->srcimg), 8, 3);
		segmentFromStats(jvf->srcimg, segimg);

		cvNamedWindow("segimg",0);
		cvShowImage("segimg",segimg);



		cvWaitKey(0);

	}
}

void segmentFromStats(IplImage *srcimg, IplImage *output)
{
	/*
	 * Loop through srcimg and color each corresponding output
	 * pixel according to the rules provide below
	 */

	/*
	 * TODO: write a new segmentation function for jvfeatures that uses
	 * selectpixels to gather stats (right click to move on to new object, or done)
	 * and colors the seg image based on the object color stats:
	 * segment->selectpixel->[some new clickExtras function]
	 * 	* overload selectpixel to include a pfn argument
	 */

	double sqthresh = 1.0;
	double piecethresh = 0.1;

	for (int y = 0; y < srcimg->height; y++) {
		uchar* yptr = (uchar*) (srcimg->imageData + y * srcimg->widthStep);
		uchar* outYptr = (uchar*) (output->imageData + y * output->widthStep);
		for (int x = 0; x < srcimg->width; x++) {
		    int r = yptr[3*x+2];
		    int g = yptr[3*x+1];
		    int b = yptr[3*x+0];

		    if ((fabs(r - brownPiece.meanR) < (piecethresh * brownPiece.varR)) &&
				(fabs(g - brownPiece.meanG) < (piecethresh * brownPiece.varG)) &&
		    	(fabs(b - brownPiece.meanB) < (piecethresh * brownPiece.varB))) {
		    	outYptr[3*x+2] = 0;
		    	outYptr[3*x+1] = 255;
		    	outYptr[3*x+0] = 0;
		    }

		    if ((fabs(r - whitePiece.meanR) < (piecethresh * whitePiece.varR)) &&
				(fabs(g - whitePiece.meanG) < (piecethresh * whitePiece.varG)) &&
		    	(fabs(b - whitePiece.meanB) < (piecethresh * whitePiece.varB))) {
		    	outYptr[3*x+2] = 255;
		    	outYptr[3*x+1] = 0;
		    	outYptr[3*x+0] = 0;
		    }

		    if ((fabs(r - whiteSquare.meanR) < (sqthresh * whiteSquare.varR)) &&
				(fabs(g - whiteSquare.meanG) < (sqthresh * whiteSquare.varG)) &&
		    	(fabs(b - whiteSquare.meanB) < (sqthresh * whiteSquare.varB))) {
		    	outYptr[3*x+2] = 0;
		    	outYptr[3*x+1] = 0;
		    	outYptr[3*x+0] = 255;
		    }

		    if ((fabs(r - greenSquare.meanR) < (sqthresh * greenSquare.varR)) &&
				(fabs(g - greenSquare.meanG) < (sqthresh * greenSquare.varG)) &&
		    	(fabs(b - greenSquare.meanB) < (sqthresh * greenSquare.varB))) {
		    	outYptr[3*x+2] = 255;
		    	outYptr[3*x+1] = 255;
		    	outYptr[3*x+0] = 0;
		    }
		}
	}


}
