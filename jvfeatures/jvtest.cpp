#include <iostream>
#include <cstdio>
#include <math.h>
#include <cstdio>
#include "jvfeatures.h"

using namespace std;

int main(int argc, char *argv[]) {
	const char* filename;

	//cvStartWindowThread();
	// Option 1: camera capture
	if (argc == 1 || (argc == 2 && strlen(argv[1]) == 1 && isdigit(argv[1][0]))) {
		jvfeatures *jvp = new jvfeatures(argc == 2 ? argv[1][0] - '0' : 0, true);
		jvp->configROI(20,120,580,360);

		// Uncomment for livecapture
		jvp->livecapture(false);

		// Uncomment for singlecapture
//		while (1) {
//		jvp->singlecapture(true);
//		jvp->select_pixel();
//		jvp->thresh(true, 90);
//		jvp->findQuads(100, true);
//		cvWaitKey(0);
//		}


	} else if ((argc == 2) && (strcmp("d",argv[1]) == 0)) {
		// Option 2: load a pre-defined texture
		printf("Drawing a random texture\n");

		// create image
		IplImage* scratch = cvCreateImage(cvSize(320, 200), 8, 3);

		// Draw basic shapes
		cvLine(scratch, cvPoint(5, 10), cvPoint(50, 100), CV_RGB(20, 255, 20), 2, 8);
		cvRectangle(scratch, cvPoint(5, 10), cvPoint(50, 100), CV_RGB(20, 255, 20), 2, 8);
		cvCircle(scratch, cvPoint(200, 140), 50, CV_RGB(200, 20, 200), 1, 8);
		cvEllipse(scratch, cvPoint(260, 80), cvSize(60,30), 45, 0, 360, CV_RGB(255, 215, 0), -1, 8);

		// Draw some filled polygons
		CvPoint  curve1[]={10,180,  20,100,  100,110,  120,130};
		CvPoint  curve2[]={230,100,  240,20,  260,30,  280,70,  300,115};
		CvPoint* curveArr[2]={curve1, curve2};
		int      nCurvePts[2]={4,5};
		cvFillPoly(scratch, curveArr, nCurvePts, 2, cvScalar(0, 255, 0), 1);

		// Write some text too
		CvFont font;
		cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, 0.5, 0.5, 0, 1);
		cvPutText (scratch,"Hand-drawn textures",cvPoint(70,20), &font, cvScalar(122, 103, 238));

		// Now pass to jvfeatures:
		jvfeatures jvf(scratch, true);
		jvf.select_pixel();
//		jvf.cluttercost(true);
//		jvf.draw_hullimg();
//		jvf.draw_houghimg();

		cvWaitKey(0);

		cvReleaseImage(&scratch);

	} else if (argc == 2) {
		// Option 3: load image from file

		filename = argv[1];
		jvfeatures *jvp = new jvfeatures(filename, true);
//		jvp->configROI(80,160,480,290); // 30, 63, 440, 400
		jvp->select_pixel();

		// Extract some features:
//		jvp->thresh(true, 10, 24, 52, 29); // 16, 22, 18
		jvp->edges(true);
//		jvp->houghlines(true);
//		jvp->segment();
//		jvp->calchist(true);
//		jvp->histentropy();
//		jvp->linescatter();
//		jvp->lineortho();
//		jvp->clutterarea();
//		jvp->findContours(true);
//		jvp->findQuads(true);
//		jvp->convexhull(true);

		// Calculate the cost params:
//		jvp->cluttercost(); // Don't have to call individual functions if computing cost
//		cout << "scat = " << jvp->scatterscore << endl;
//		cout << "entropy = " << jvp->entropy << endl;
//		cout << "ortho = " << jvp->orthoscore << endl;
//		cout << "charea = " << jvp->charea << endl;
//		cout << "cost = " << jvp->cost << endl;

		// Test quad constructor and draw functions:
//		for (int i=0; i<jvp->quads.size(); ++i){
//			quad q(jvp->quads[i].center, jvp->quads[i].angle, jvp->quads[i].width, jvp->quads[i].height);
//			q.drawPoly(jvp->quadsimg);
//		}
//		cvWaitKey(0);
//		cvShowImage("Quads Image", jvp->quadsimg);

		// Test angles
//		quad q1(point(250,200),-30, 20,60);
//		q1.drawPoly(jvp->quadsimg);
//		quad q2(point(250,300),0, 20,60);
//		q2.drawPoly(jvp->quadsimg);
//		quad q3(point(250,400),30, 20,60);
//		q3.drawPoly(jvp->quadsimg);
//		cvShowImage("Quads Image", jvp->quadsimg);
//		cvWaitKey(0);

		// List quads:
		for (int i=0; i<jvp->quads.size(); i++) {
			cout << "Quad " << i << " " << jvp->quads[i] << ", obst = " << jvp->quads[i].isObstacle << ", dimsum=" << jvp->quads[i].width + jvp->quads[i].height << endl;
			cout << "Width = " << jvp->quads[i].width << ", Height = " << jvp->quads[i].height << endl << endl;
		}

		cvWaitKey(0);
	} else {
		printf("Error: wrong number of args provided\n");
		return -1;
	}

	return 0;
}
