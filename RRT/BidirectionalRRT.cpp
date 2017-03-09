/*
 * Copyright (c) 2010, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file BidirectionalRRT.cpp
 *
 *  @date Feb 27, 2012
 *  @author Jon Scholz
 */


#include "RRT.h"
#include <algorithm>
#include <assert.h>

using namespace std;

BidirectionalRRT::BidirectionalRRT() 
{	
}

BidirectionalRRT::~BidirectionalRRT() 
{
	rrtS.cleanup();
	rrtG.cleanup();
}

void BidirectionalRRT::initialize(const config &initConf, const config &goalConf,
		const config &minConf, const config &maxConf)
{		
	this->rrtS.initialize(initConf, goalConf, minConf, maxConf);
	this->rrtG.initialize(goalConf, initConf, minConf, maxConf);
	this->rrtS.parent = this;
	this->rrtG.parent = this;
}

void BidirectionalRRT::setStart(const config &initConf) {
	this->rrtS.initConfig = config(initConf);
	this->rrtG.goalConfig = config(initConf);
}

config BidirectionalRRT::getStart() {
	return this->rrtS.initConfig;
}

void BidirectionalRRT::setGoal(const config &goalConfig) {
	this->rrtS.goalConfig = config(goalConfig);
	this->rrtG.initConfig = config(goalConfig);
	this->rrtS.bestConfigIdx = -1;
	this->rrtG.bestConfigIdx = -1;
	this->rrtS.bestCost = DBL_MAX;
	this->rrtG.bestCost = DBL_MAX;
}

config BidirectionalRRT::getGoal() {
	return this->rrtS.goalConfig;
}

void BidirectionalRRT::setMinConf(const config &minConf) {
	this->rrtS.minconfig = config(minConf);
	this->rrtG.minconfig = config(minConf);
}

config BidirectionalRRT::getMinConf() {
	return this->rrtS.minconfig; // meh asserting is for cowards
}

void BidirectionalRRT::setMaxConf(const config &maxConf) {
	this->rrtS.maxconfig = config(maxConf);
	this->rrtG.maxconfig = config(maxConf);
}

config BidirectionalRRT::getMaxConf() {
	return this->rrtS.maxconfig; // meh asserting is for cowards
}

// Set relevant runtime parameters for InnerRRTs
void BidirectionalRRT::setRunParams(double step_size, int max_nodes, 
		double term_thresh, int linear_limit, double ANN_epsilon) 
{
	this->rrtS.setRunParams(step_size, max_nodes/2, term_thresh, linear_limit, ANN_epsilon);
	this->rrtG.setRunParams(step_size, max_nodes/2, term_thresh, linear_limit, ANN_epsilon);
}

// Main run method: wraps stepGreedy and stepRandom
void BidirectionalRRT::run(int greedyFreq, bool verbose) 
{
	rrtS.verbose = verbose;
	rrtG.verbose = verbose;
	if (this->rrtS.step_size == -1)
		setRunParams();

	int ctr = 0;
	while (rrtS.bestCost > 0.005 && rrtG.bestCost > 0.005 && 
			rrtS.configVector.size() < rrtS.max_nodes &&
			rrtG.configVector.size() < rrtG.max_nodes) {

		if (ctr % greedyFreq == 0)
			stepTowardsClosest();
		else
			stepRandom();
		ctr++;
	}
}

void BidirectionalRRT::stepRandom() 
{
	rrtS.stepRandom();
	rrtG.stepRandom();
}

/**
 * Steps both RRTs towards the most recently added 
 * node in the opposite tree
 */
void BidirectionalRRT::stepTowardsRecent() 
{
	rrtS.setGoal(rrtG.configVector.back());
	rrtS.stepGreedy();
	rrtG.setGoal(rrtS.configVector.back());
	rrtG.stepGreedy();
}

/**
 * Steps both RRTs towards some approximation
 * of the closest node in the opposite tree
 */
void BidirectionalRRT::stepTowardsClosest() 
{
	// Use tree root node as closest
	rrtS.setGoal(rrtG.configVector[rrtG.getNearestNeighbor(rrtS.initConfig)]);
	rrtS.stepGreedy();
	rrtG.setGoal(rrtS.configVector[rrtS.getNearestNeighbor(rrtG.initConfig)]);
	rrtG.stepGreedy();
}

// Traces both paths and merges them
std::vector<config> BidirectionalRRT::getPath() 
{
	// Figure out which rrt terminated
	InnerRRT *winner = rrtS.bestCost < rrtG.bestCost ? &rrtS : &rrtG;
	InnerRRT *loser = winner == &rrtS ? &rrtG : &rrtS;
	assert(winner->bestCost < winner->term_thresh);

	// Set the bestConf index in the loser to its NN to the winner's terminal node
	loser->bestConfigIdx = loser->getNearestNeighbor(winner->bestConfig);

	// Extract paths as usual, reversing rrtG
	path.clear();
	vector<config> p1 = rrtS.getPath();
	vector<config> p2 = rrtG.getPath();
	reverse(p2.begin(), p2.end());

	path.insert(path.end(), p1.begin(), p1.end());
	path.insert(path.end(), p2.begin(), p2.end());

	return path;
}

// stub
std::vector<config> BidirectionalRRT::getPath(int nodeIdx) {
	cout << "Not implemented.  Also why are you calling me?" << endl;
	exit(-1);
}

// Dumps the points in the tree
void BidirectionalRRT::printTree() 
{
	rrtS.printTree();
	rrtG.printTree();
}

bool BidirectionalRRT::checkCollisions(config &c)
{
	return false;
}
