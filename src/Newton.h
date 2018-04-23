#pragma once

#include "Solver.h"
#include "EigenTypes.h"
#include "PardisoSolver.h"

#include <iostream>
#include <Eigen/SparseCholesky>

using namespace std;

class Newton : public Solver
{
public:
	Newton();

	double step();
	bool test_progress();
	void internal_init();

private:
	// norm of the progress on the mesh
	double diff_norm;

	unique_ptr<PardisoSolver<vector<int>, vector<double>>> pardiso = nullptr;
	long long int prevTime;
};