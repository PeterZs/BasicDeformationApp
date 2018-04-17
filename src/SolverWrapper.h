#pragma once

#include "Solver.h"
#include "Newton.h"

class SolverWrapper
{
public:
	SolverWrapper();

	void init(const Eigen::MatrixX3d& V, const Eigen::MatrixX3i& F);

	// Getter & setter for parameters and the mesh
	void set_lambda(double new_lambda);
	void set_position_weight(double new_pos);

	void set_mesh_position(const Eigen::MatrixX3d& Vs_new);
	bool progressed();

	void get_slot();
	void release_slot();

	// Interface to outside
	shared_ptr<Solver> solver;

private:
	// Explicit solver implementations
	shared_ptr<Newton> newton;

	// when moving a green face, restore it again to green after moving
	int restore_constrained_face = -1;
};