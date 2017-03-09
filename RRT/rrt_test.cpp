#include "RRT.h"
#include <iostream>
#include <vector>
#include <assert.h>

using namespace std;

/**
 * @file rrt_test.cpp
 * Demonstrates implementation of an RRT for a simple
 * 2D path planning problem in the presence of circular
 * obstacles.
 *
 *  @date Feb 27, 2012
 *  @author Jon Scholz
 */

class TestRRT : public RRT {
// class TestRRT : public BidirectionalRRT {
public:	
	virtual ~TestRRT() {
		std::cout << "calling TestRRT destructor" << std::endl;
	}
	vector<config> obstCenters; /// Obstacle parameters
	vector<double> obstRadii; /// Radius (of hypersphere)
	
	void installObstacle(config pos, double radius) {
		obstCenters.push_back(pos);
		obstRadii.push_back(radius);
	}
	
	// simple collision model (hyperspheres)
	virtual bool checkCollisions(config &c) {
		assert(obstCenters.size() == obstRadii.size());
		for (int i = 0; i < obstCenters.size(); ++i) {
			if ((c - obstCenters[i]).getCost() < obstRadii[i]) 
				return true;	
		}
		return false;
	}
};

int main()
{
	// // Some rrt obj
	// TestRRT rrt;
	// 
	// // Specify task configuration
	// config init(2);
	// init[0]=0.5;
	// init[1]=0.5;
	// 
	// config goal(2);
	// goal[0]=3.5;
	// goal[1]=3.5;
	// 
	// config minConf(2);
	// minConf[0] = 0;
	// minConf[1] = 0;
	// 
	// config maxConf(2);
	// maxConf[0] = 4;
	// maxConf[1] = 4;
	
	// // Add some obstacles
	// config obst1(2);
	// obst1[0] = 1;
	// obst1[1] = 1;
	// rrt.installObstacle(obst1, 0.5);
	// 
	// obst1[0] = 2;
	// obst1[1] = 2;
	// rrt.installObstacle(obst1, 0.7);
	// 
	// obst1[0] = 3;
	// obst1[1] = 3;
	// rrt.installObstacle(obst1, 0.5);
	// 
	// obst1[0] = 3;
	// obst1[1] = 1;
	// rrt.installObstacle(obst1, 0.65);
	// 
	// obst1[0] = 1;
	// obst1[1] = 3;
	// rrt.installObstacle(obst1, 0.65);
	
	// Our RRT problem:
	// rrt.initialize(init, goal, minConf, maxConf);
	// cout << "initConfig = " << rrt.getStart() << endl;
	// cout << "goalConfig = " << rrt.getGoal() << endl;
	// cout << "minConfig = " << rrt.getMinConf() << endl;
	// cout << "maxConfig = " << rrt.getMaxConf() << endl;
	//  	for (int i = 0; i < rrt.obstCenters.size(); ++i)
	// 	cout << "obstacle " << i << ": {" << rrt.obstCenters[i] << "," << rrt.obstRadii[i] << "}" << endl;
	
	// Our RRT solution:
	// rrt.run(20, false);
	// rrt.printTree();
	// rrt.printPath();
	// cout << "Done." << endl;

	// double *v = new double[2];
	// double *y = v;
	// delete[] v;
	// delete[] y;
	// double *v;
	// delete[] v;
	config a(2);
	config b;
	b = config(a);
	
	
	return 0;
}
