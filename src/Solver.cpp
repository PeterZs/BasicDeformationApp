#include "Solver.h"

#include "Utils.h"
#include "Energy.h"

#include <iostream>
#include <igl/readOBJ.h>
#include <igl/file_dialog_open.h>
#include <igl/flipped_triangles.h>
#include <igl/map_vertices_to_circle.h>
#include <igl/harmonic.h>

Solver::Solver()
	:
	energy(make_shared<Energy>()),
	param_mutex(make_unique<mutex>()),
	mesh_mutex(make_unique<shared_timed_mutex>()),
	param_cv(make_unique<condition_variable>()),
	num_steps(2147483647)
{
}

void Solver::init(const Eigen::MatrixX3d& V, const Eigen::MatrixX3i& F)
{
	using namespace placeholders;
	this->F = F;
	Vdef = V;
	energy->init(V, F);
	m_x = Eigen::Map<Vec>(Vdef.data(), Vdef.rows() * Vdef.cols());
	ext_x = m_x;
	eval_f = bind(&Energy::evaluate_f, energy, _1, _2);
	eval_fgh = bind(&Energy::evaluate_fgh, energy, _1, _2, _3, _4);
	internal_init();
}

int Solver::run()
{
	is_running = true;
	halt = false;
	int steps = 0;
	do
	{
		ret = step();
		linesearch();
		update_external_mesh();
	} while ((a_parameter_was_updated || test_progress()) && !halt && ++steps < num_steps);
	is_running = false;
	cout << "solver stopped" << endl;
	return ret;
}

void Solver::stop()
{
	wait_for_param_slot();
	halt = true;
	release_param_slot();
}

void Solver::update_external_mesh()
{
	give_param_slot();
	unique_lock<shared_timed_mutex> lock(*mesh_mutex);
	internal_update_external_mesh();
	progressed = true;
}

void Solver::get_mesh(Eigen::MatrixX2d& X)
{
	unique_lock<shared_timed_mutex> lock(*mesh_mutex);
	X = Eigen::Map<Eigen::MatrixX2d>(ext_x.data(), ext_x.rows() / 2, 2);
	progressed = false;
}

void Solver::give_param_slot()
{
	a_parameter_was_updated = false;
	unique_lock<mutex> lock(*param_mutex);
	params_ready_to_update = true;
	param_cv->notify_one();
	while (wait_for_param_update)
	{
		param_cv->wait(lock);
		a_parameter_was_updated = true;
	}
	params_ready_to_update = false;
}

void Solver::wait_for_param_slot()
{
	unique_lock<mutex> lock(*param_mutex);
	wait_for_param_update = true;
	while (!params_ready_to_update && is_running)
		param_cv->wait_for(lock, chrono::milliseconds(50));
}

void Solver::release_param_slot()
{
	wait_for_param_update = false;
	param_cv->notify_one();
}