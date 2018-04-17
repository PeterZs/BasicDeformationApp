#pragma once

#include "EigenTypes.h"
#include "EnergySymDir.h"
//#include "Position.h"

#include <memory>

using namespace std;

class Energy
{
public:
	Energy();

	void init(const Eigen::MatrixX3d& V, const Eigen::MatrixX3i& F);
	void evaluate_f(const Vec& x, double& f);
	void evaluate_fgh(const Vec& x, double& f, Vec& g, SpMat& h);

	// helper functions
    double w;
// 	unique_ptr<Position> position;
	unique_ptr<DistortionSymDir> symDirichlet;
};