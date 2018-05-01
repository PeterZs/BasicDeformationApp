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

	int step();
	void linesearch();
	bool test_progress();
	void internal_init();
	void internal_update_external_mesh();

private:
	// Wrapper function for flip_avoiding_line_search
	double eval_ls(Eigen::MatrixXd& x);

	// norm of the progress on the mesh
	double diff_norm;

	unique_ptr<PardisoSolver<vector<int>, vector<double>>> pardiso = nullptr;
	long long int prevTime;
};