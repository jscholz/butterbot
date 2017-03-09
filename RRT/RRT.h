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
 * @file RRT.h
 * Compact RRT library for path planning in n-dimensions.  
 *
 * To apply to your domain, you only need to specify the following:
 *		- start and end points expressible as a vector of doubles ("config")
 *		- lower and upper bounds on the search space, also expressible as configs
 *		- a collision function which returns true for invalid state configurations
 * 
 * See rrt_test.cpp for an example.
 *
 *  @date Feb 27, 2012
 *  @author Jon Scholz
 */

#ifndef RRT_H_
#define RRT_H_

// #include "AbstractRRT.h"
#include <vector>
#include <stdlib.h>
#include <iostream>
#include "ANN/ANN.h"

#include <assert.h>
template<class T> class Config
{
public:
	Config() {
		this->init(NULL, 0, 0, NULL);
	}
	
	Config(size_t dim) {
		this->init(NULL, dim, 0, NULL);
	}
	
	Config(T *v, size_t dim, double cost = 0, Config<T> *parent = NULL) {
		this->init(v, dim, cost, parent);
	}
	
	Config(const Config<T> &other) {
		this->init(other.v, other.dim, other.cost, other.parent);
	}
	
	virtual ~Config () {
		std::cout << "calling config destructor" << std::endl;
		delete[] this->v;
	};
	
	size_t size() const {
		return this->dim;
	}
	
	double getCost() {
		return this->cost;
	}
	
	void setCost(double c) {
		this->cost = c;
	}
	
	double getParent() {
		return this->parent;
	}
	
	void setParent(Config<T> *parent) {
		this->parent = parent;
	}
	
	T& operator[](int i) const {
		assert(i < this->dim);
		return this->v[i];
	}
	
	friend std::ostream& operator<<(std::ostream& os, const Config& c) {
		// os << "{[";
		for (int i = 0; i < c.dim; ++i){
			os << c.v[i];
			if (i < c.dim - 1)
				os << " ";
		}
		// os << "][" << c.cost << "][" << c.parent << "]}";
		return os;
	}
	
	friend Config operator-(const Config<T>& a, const Config<T>& b) {
			assert(a.size() == b.size());
			Config<T> result(a);
			for (int i = 0; i < a.size(); ++i) {
				result[i] = a[i] - b[i];
				result.cost += result[i] * result[i];
			}
			result.cost = sqrt(result.cost); /* comment for speed */
			return result;
		}
	
public://protected:
	size_t dim;
	T *v;
	double cost;
	Config *parent;
	
	void init(T *v, size_t dim, double cost = 0, Config<T> *parent = NULL) {
		assert(dim >= 0);
		this->dim = dim;
		if (dim > 0) {
			this->v = new T[dim];
			if (v != NULL) {
				for (int i = 0; i < dim; ++i)
					this->v[i] = v[i];
			} 
		}
		else 
			this->v = NULL;
		this->cost = cost;
		this->parent = parent;
	}
};
typedef Config<double> config;

/// A typedef for representing and manipulating state configurations:
// typedef std::vector<double> config;
// config operator-(const config& v1, const config& v2);
// std::ostream& operator<<(std::ostream& os, const config& l);

/**
 * An abstract base for RRT planners.  Supports all the functionality 
 * that's common between single and bi-directional RRT.
 */
 class AbstractRRT {
 public:
 	AbstractRRT();
 	virtual ~AbstractRRT();

 	virtual void initialize(const config &initConf, const config &goalConf,
 							const config &minConf, const config &maxConf);

 	virtual void cleanup();

 	virtual void setStart(const config &initConf);
 	virtual config getStart();

 	virtual void setMinConf(const config &minConf);
 	virtual config getMinConf();

 	virtual void setMaxConf(const config &maxConf);
 	virtual config getMaxConf();

 	virtual void setGoal(const config &goalConf);
 	virtual config getGoal();

 	virtual void run(int greedyFreq = 10, bool verbose = false) = 0;

 	virtual std::vector<config> getPath() = 0;
 	virtual std::vector<config> getPath(int nodeIdx) = 0;

 	// Traces the path and dumps to stdout
 	virtual void printPath();

 protected:
 	config initConfig; 	/// Container for starting configuration
 	config goalConfig; 	/// Container for goalConfig configuration
 	config minconfig; 	/// Container for minimum configuration values
 	config maxconfig; 	/// Container for maximum configuration values
 	config bestConfig;  /// actual best configuration params

 	int bestConfigIdx;	/// refers to an index in configVector
 	double bestCost;		/// squared distance to best config

 	std::vector<int> parentVector;		/// vector of indices to relate configs in AbstractRRT
 	std::vector<config> configVector; 	/// vector of all visited configs
 	std::vector<config> path; 			/// vector of configs between initConfig and bestConf

 	/// Functions for computing sum of squared distances between configurations
 	double SSD(const config &a, const config &b);
 	double SSD(const config &a, const config &b, config &out);

 	/// Implementation-specific function for checking collisions  (must be overridden for MBP)
 	virtual bool checkCollisions(config &c) = 0;
 };

/**
 * RRT: A single-tree implementation.
 * This version performs a one-directional search using a single RRT 
 * from the starting configuration to the goal.  
 */
class RRT : public AbstractRRT {
public:
	RRT();
	virtual ~RRT();
	virtual void initialize(const config &initConf, const config &goalConf, 
			const config &minConf, const config &maxConf);

	virtual void cleanup();

	/// Initialize the nearest-neighbor member (first call setRunParams)
	void Init_ANN();

	/// Set relevant runtime parameters
	virtual void setRunParams(double step_size = 0.02, int max_nodes = 100000, 
			double term_thresh = 0.005, int linear_limit = 1000, double ANN_epsilon = 0);

	/// Main run method: wraps stepGreedy and stepRandom
	virtual void run(int greedyFreq = 10, bool verbose = false);

	/// traces the path from bestConfigIdx to the tree root
	virtual std::vector<config> getPath();

	/// traces the path from given nodeIdx to the tree root
	virtual std::vector<config> getPath(int nodeIdx);

	/// Dumps tree points
	void printTree();

	friend class BidirectionalRRT;

protected:
	int ndim;				/// Dimensionality of configuration space
	double step_size;		/// Step size of RRT
	int max_nodes;			/// Maximum number of nodes in RRT
	double term_thresh;		/// Termination threshold for distance from goal
	int linear_limit;		/// Maximum number of nodes search linearly before rebuilding KD-tree
	double ANNeps;			/// Error parameter in nearest-neighbor lookups

	bool verbose;			/// prints reasonable output information
	int ssi; 				/// steps since improvement - useful for planning TODO: use it
	config qtmp; 			/// Container for random configuration

	/// ANN stuff
	int 				linearNNstart;
	int					nPts;					/// actual number of data points
	ANNpointArray		dataPts;				/// data points
	ANNpoint			queryPt;				/// query point
	ANNidxArray			nnIdx;					/// near neighbor indices
	ANNdistArray		dists;					/// near neighbor distances
	ANNkd_tree*			kdTree;					/// search structure

	/// Optional helper: takes a step from NN in a random direction
	void stepRandom();

	/// Optional helper: takes a greedy step from NN towards goal
	void stepGreedy();

	/// Tries to extend tree towards provided sample (must be overridden for MBP)
	virtual void tryStep(config qtry, int NNidx);

	/// Adds qnew to the tree
	void addNode(config &qnew, int parentID);

	/// Sets qsamp a random configuration (virtual in case you wanna do something else with sampled states)
	virtual config& getRandomConfig();

	/// Returns NN to query point
	int getNearestNeighbor(config &qsamp);

	/// Implementation-specific function for checking collisions
	virtual bool checkCollisions(config &c);
};

/**
 * BidirectionalRRT: A bi-directional implementation.
 * This version performs bi-directional search using two RRTs, 
 * one from the starting configuration and one from the goal.
 */
class BidirectionalRRT : public AbstractRRT {
public:
	BidirectionalRRT();
	virtual ~BidirectionalRRT();

	void initialize(const config &initConf, const config &goalConf, 
			const config &minConf, const config &maxConf);

	virtual void setStart(const config &initConf);
	virtual config getStart();

	virtual void setMinConf(const config &minConf);
	virtual config getMinConf();

	virtual void setMaxConf(const config &maxConf);
	virtual config getMaxConf();

	virtual void setGoal(const config &goalConf);
	virtual config getGoal();

	// Set relevant runtime parameters for InnerRRTs
	virtual void setRunParams(double step_size = 0.02, int max_nodes = 100000, 
			double term_thresh = 0.005, int linear_limit = 1000, double ANN_epsilon = 0);

	// Main run method: wraps stepGreedy and stepRandom
	void run(int greedyFreq = 10, bool verbose = false);

	// Traces both paths and combines them
	virtual std::vector<config> getPath();

	// Unused, but required by interface
	virtual std::vector<config> getPath(int nodeIdx);

	// Dumps the points in both trees
	virtual void printTree();

protected:
	class InnerRRT : public RRT {
	public:
		BidirectionalRRT *parent;		// Pointer to parent, for sharing a collision function

		virtual bool checkCollisions(config &c) {
			return this->parent->checkCollisions(c);
		}
	};

	InnerRRT rrtS;					// an rrt starting at the init configuration
	InnerRRT rrtG;					// an rrt starting at the goal configuration

	// Steps each RRT randomly
	void stepRandom();

	// Step each RRT towards the other
	void stepTowardsRecent();
	void stepTowardsClosest() ;

	// Implementation-specific function for checking collisions
	virtual bool checkCollisions(config &c);
};


#endif /* RRT_H_ */
