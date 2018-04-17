#include "SolverWrapper.h"

SolverWrapper::SolverWrapper()
	:
	newton(make_unique<Newton>())
{
	solver = newton;
}

void SolverWrapper::init(const Eigen::MatrixX3d& V, const Eigen::MatrixX3i& F)
{
	solver->init(V, F);
}

void SolverWrapper::set_lambda(double new_lambda)
{
	solver->wait_for_param_slot();
//	solver->energy->lambda = new_lambda;
	solver->release_param_slot();
}

void SolverWrapper::set_position_weight(double new_pos)
{
	solver->wait_for_param_slot();
//	solver->energy->pos_weight = new_pos;
	solver->release_param_slot();
}

void SolverWrapper::set_mesh_position(const Eigen::MatrixX3d& Vs_new)
{
	solver->wait_for_param_slot();
	solver->Vdef = Vs_new;
	solver->m_x = Eigen::Map<const Vec>(Vs_new.data(), Vs_new.rows() * Vs_new.cols());
	solver->ext_x = Eigen::Map<const Vec>(Vs_new.data(), Vs_new.rows() * Vs_new.cols());
	solver->release_param_slot();
}

bool SolverWrapper::progressed()
{
	return solver->progressed;
}

void SolverWrapper::get_slot()
{
	solver->wait_for_param_slot();
}

void SolverWrapper::release_slot()
{
	solver->release_param_slot();
}