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
 *
 */

/**
 * @file RRT.cpp
 *
 *  @date Feb 27, 2012
 *  @author Jon Scholz
 */

#include "RRT.h"

using namespace std;

#define RANDNM(N,M) N + ((M-N) * (rand() / ((double)RAND_MAX + 1))) // random # between N&M

RRT::RRT() {
	this->step_size = -1;
	this->max_nodes = -1;
	this->linear_limit = -1;
	this->ANNeps = -1;
	this->dataPts = NULL;
}

void RRT::initialize(const config &initConf, const config &goalConf, 
					const config &minConf, const config &maxConf)
{
	// AbstractRRT::initialize(initConf, goalConf, minConf, maxConf);
	// this->qtmp(initConf);
	this->ndim = initConf.size();
}

RRT::~RRT() {
	cout << "calling RRT destructor" << endl;
	cleanup();
}

void RRT::cleanup()
{
	AbstractRRT::cleanup();

	if (dataPts != NULL) {
		annDeallocPts(dataPts);
		delete[] nnIdx;
		delete[] dists;
		delete kdTree;
		annClose();
	}
}

void RRT::Init_ANN()
{
	int maxPts = max_nodes; // node_limit+1
	int k = 1;
	
	linearNNstart=0;
	
	queryPt = annAllocPt(ndim);					// allocate query point
	for(int i=0; i<ndim; i++)
		queryPt[i] = 0.1*i;
	
	dataPts = annAllocPts(max_nodes, ndim);		// allocate data points
	nnIdx = new ANNidx[k];						// allocate near neighbor indices
	dists = new ANNdist[k];						// allocate near neighbor dists

	nPts = 0;									// counter for number of data points

	addNode(initConfig, bestConfigIdx); 				// Add initConfig config and "-1" to state vectors and ANN

	kdTree = new ANNkd_tree(		// build search structure
		dataPts,					// the data points
		nPts,						// current number of points
		ndim);						// dimension of space
}

// Set relevant runtime parameters
void RRT::setRunParams(double step_size, int max_nodes, 
	double term_thresh, int linear_limit, double ANN_epsilon) 
{
	this->step_size = step_size;
	this->max_nodes = max_nodes;
	this->term_thresh = term_thresh;
	this->linear_limit = linear_limit;
	this->ANNeps = ANN_epsilon;
	Init_ANN();
}

void RRT::run(int greedyFreq, bool verbose) 
{
	cleanup();
	this->verbose = verbose;
	if (this->step_size == -1)
		setRunParams();
	
	int ctr = 0;
	while (this->bestCost > 0.005 && this->configVector.size() < this->max_nodes) {
		if (ctr % greedyFreq == 0)
			stepGreedy();
		else
			stepRandom();
		ctr++;
	}
}

/*
 * Take a step in a random direction (wraps getRandomConfig,
 * getNearestNeighbor, and tryStep)
 */
void RRT::stepRandom()
{
	getRandomConfig();
	int NNidx = getNearestNeighbor(qtmp);
	tryStep(qtmp, NNidx);
}

/*
 * Implement this to take a step towards a specific configuration (also
 * wraps getRandomConfig, getNearestNeighbor, and addNode)
 */
void RRT::stepGreedy()
{
	int NNidx = getNearestNeighbor(goalConfig);
	tryStep(goalConfig, NNidx);
}

/*
 * Calculates a new node to grow towards qtry, checks for collisions, and adds
 * [also maintains distance to goalConfig]
 */
void RRT::tryStep(config qtry, int NNidx)
{
	config qnear(ndim);
	config qnew(ndim);
	qnear = configVector[NNidx];	// sets qnear to the closest configuration to qsamp
	
	// cout << "dims: " << qtry.size() << ","<< qnear.size() << ","<< qnew.size() << "," << endl;
	
	// Compute direction and magnitude of vector from qnew to qtry
	double sumsq = SSD(qtry, qnear, qnew);
	double edist = sqrt(sumsq);
	cout << "qtry: " << qtry << endl;
	cout << "qnear: " << qnear << endl;
	cout << "qnew: " << qnew << ", " << edist << endl;
		
	// config qnear2 = configVector[NNidx];
	// config qnew2 = qtry - qnear2;
	// double edist2 = qnew.getCost();
	// cout << "qtry: " << qtry << endl;
	// cout << "qnear2: " << qnear2 << endl;
	// cout << qnew2 << ", " << edist2 << endl << endl;

	// Scale this vector to step_size and add to end of qnear
	double scale = (double)step_size / edist;
	for (int i=0; i<ndim; ++i){
		qnew[i] = qnew[i] * scale + qnear[i];
	}

	if (!checkCollisions(qnew)) {
		addNode(qnew, NNidx);
		double cost = (qnew - goalConfig).getCost();

		if (cost < bestCost) {
			bestConfigIdx = configVector.size() - 1;	// if last node is closest, mark idx as bestConf
			bestCost = cost;
			bestConfig = configVector[bestConfigIdx];
			
			if (verbose)
				cout << "achieved best cost: " << bestCost << " (treesize=" << configVector.size() << ")" << endl;
		}
	}
}

/*
 * Expands RRT by attaching qnew at parentID (and
 * balances tree)
 */
void RRT::addNode(config &qnew, int parentID)
{
	// Update graph vectors
	configVector.push_back(qnew);
	parentVector.push_back(parentID);

	// add points to ANN data set
	for(int i=0; i < ndim; i++)
		dataPts[nPts][i] = qnew[i];
	nPts++;
	
	// after "linear_limit" steps build new tree
	if(configVector.size()-linearNNstart > linear_limit){
		delete kdTree;
		kdTree = new ANNkd_tree(dataPts,nPts,ndim);
		linearNNstart = configVector.size();
	}
}

/*
 * Samples a random point for qtmp in the configuration space,
 * bounded by the provided configuration vectors (and returns ref to it)
 */
config& RRT::getRandomConfig()
{
	for (int i = 0; i < ndim; ++i) {
		qtmp[i] = RANDNM(minconfig[i], maxconfig[i]);
	}
	return qtmp;
}

/*
 * Returns ID of config node nearest to qsamp
 */
int RRT::getNearestNeighbor(config &qsamp)
{
	double min = DBL_MAX;
	double cost = DBL_MAX;
	int nearest = 0;

	// First search the linear vector
	for(int i = linearNNstart; i < configVector.size(); ++i){
		cost = (qsamp - configVector[i]).getCost();
	
		if(cost < min) {
			min = cost;
			nearest = i;
		}
	}
	
	//Then search the ANN kd-tree
	if(nPts > linear_limit){
		for(int i = 0; i < ndim; ++i)
			queryPt[i] = qsamp[i];
	
		kdTree->annkSearch(queryPt, 1, nnIdx, dists, ANNeps);
	
		// take best result from ANN & list
		if (dists[0] < min)
			nearest = nnIdx[0];
	}
	return nearest;
}

// traces the path from bestConfigIdx to the tree root
vector<config> RRT::getPath() {
	return getPath(bestConfigIdx);
}

// traces the path from given nodeIdx to the tree root
vector<config> RRT::getPath(int nodeIdx)
{
	if (nodeIdx > 0) {
		int x = nodeIdx;
		path.clear();
		while (parentVector[x] != -1) {
			path.insert(path.begin(), configVector[x]);
			x = parentVector[x];
		}
	}
	return path;
}

/*
 * Dumps the child-parent mappings for the current tree
 */
void RRT::printTree()
{
	cout << "<tree> (" << configVector.size() << " nodes)" << endl;
	for (int i=0; i < parentVector.size(); ++i) {
		int parIdx = parentVector[i];
		if (parIdx != -1) 
			cout << configVector[i] << " -> " << configVector[parIdx] << endl;
	}
	cout << "</tree>" << endl;
}

bool RRT::checkCollisions(config &c)
{
	return false;
}
