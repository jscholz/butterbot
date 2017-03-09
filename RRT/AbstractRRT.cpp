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
 * @file AbstractRRT.cpp
 *
 *  @date Feb 27, 2012
 *  @author Jon Scholz
 */

#include "RRT.h"
#include "float.h"
#include <assert.h>

using namespace std;

//#define DEBUG
#define RANDNM(N,M) N + ((M-N) * (rand() / ((double)RAND_MAX + 1))) // random # between N&M

// config operator-(const config& v1, const config& v2) {
// 	config result(v1);
// 	for (int i=0; i < v1.size(); ++i) {
// 		result[i] = v1[i] - v2[i];
// 	}
// 	return result;
// }
// 
// ostream& operator<<(ostream& os, const config& c) {
// 	for (int i = 0; i < c.size(); ++i){
// 		os << c[i];
// 		if (i < c.size() - 1)
// 			os << " ";
// 	}
// 	return os;
// }

AbstractRRT::AbstractRRT() {

}

/**
 * Sets the task configuration space and starting/terminal conditions
 * @param initConf
 * @param goalConf
 * @param minConf
 * @param maxConf
 */
void AbstractRRT::initialize(const config &initConf, const config &goalConf,
							 const config &minConf, const config &maxConf)
{
	// clean-up from previous
	cleanup();

	setMinConf(minConf);
	setMaxConf(maxConf);
	setGoal(goalConf);
	setStart(initConf);

	this->parentVector.clear();
	this->configVector.clear();

	srand(time(NULL));
}

AbstractRRT::~AbstractRRT() {
	std::cout << "calling AbstractRRT destructor" << std::endl;
	cleanup();
}

void AbstractRRT::cleanup()
{
	parentVector.clear();
	configVector.clear();
	parentVector.resize(0);
	configVector.resize(0);
}

void AbstractRRT::setStart(const config &initConf) {
	this->initConfig = config(initConf);
}

config AbstractRRT::getStart() {
	return this->initConfig;
}

void AbstractRRT::setGoal(const config &goalConfig) {
	this->goalConfig = config(goalConfig);
	this->bestConfigIdx = -1;
	this->bestCost = DBL_MAX;
}

config AbstractRRT::getGoal() {
	return this->goalConfig;
}

void AbstractRRT::setMinConf(const config &minConf) {
	this->minconfig = config(minConf);
}

config AbstractRRT::getMinConf() {
	return this->minconfig;
}

void AbstractRRT::setMaxConf(const config &maxConf) {
	this->maxconfig = config(maxConf);
}

config AbstractRRT::getMaxConf() {
	return this->maxconfig;
}

void AbstractRRT::printPath()
{
	if (path.size() == 0)
		getPath();
	cout << "<path> (" << path.size() << " nodes)" << endl;
	for (int i = 0; i < path.size(); ++i) {
		cout << path[i] << endl;
	}
	cout << "</path>" << endl;
}

// Computes sum of squared differences between vectors a and b
double AbstractRRT::SSD(const config &a, const config &b) {
	assert(a.size() == b.size());

	double ssd = 0;
	for (int i = 0; i < a.size(); ++i) {
		double d = a[i] - b[i];
		ssd += d * d;
	}
	return ssd;
}

// computes SSD and saves component-wise difference in c
double AbstractRRT::SSD(const config &a, const config &b, config &out) {
	assert((a.size() == b.size()) && (a.size() == out.size()));

	double ssd=0;
	for (int i = 0; i < a.size(); ++i) {
		double d = a[i] - b[i];
		out[i] = d;
		ssd += d * d;
	}
	return ssd;
}
