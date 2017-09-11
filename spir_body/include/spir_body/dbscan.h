#ifndef DBSCAN_H
#define DBSCAN_H
#include <vector>
#include <boost/numeric/ublas/matrix.hpp>

#include "spir_body/clusters.h"

/** @class dbscan.h
 *  @brief implement dbscan clustering approach
 *
 *  example code found online and modified
 *
 *  @author Wenjie Lu
 *  @contact wenjie.lu@uts.edu.au
 *  @date 2nd Sep 2016
 *  @version 1.0.0
 *  @bug Currently the No known bugs.
 *  @todo None
 */

namespace Clustering{

	class DBSCAN : public Clusters
	{
	public:
		DBSCAN(Points & ps, double eps, unsigned int minPts) : 
		  Clusters(ps), _eps(eps), _minPts(minPts)
		{
			_noise.resize(ps.size(), false);
			_visited.resize(ps.size(), false);
		};

		// 
		// The clustering algo
		//
		void run_cluster() ;

	private:

	// eps radiuus
	// Two points are neighbors if the distance 
	// between them does not exceed threshold value.
		double _eps;

	//minimum number of points
		unsigned int _minPts;

	// noise vector
		std::vector<bool> _noise;

	// noise vector
		std::vector<bool> _visited;
	};
};
#endif