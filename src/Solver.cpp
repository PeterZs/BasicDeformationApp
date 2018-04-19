#include "Solver.h"

#include "Utils.h"
#include "TotalObjective.h"

#include <iostream>

Solver::Solver()
	:
	parameters_mutex(make_unique<mutex>()),
	data_mutex(make_unique<shared_timed_mutex>()),
	param_cv(make_unique<condition_variable>()),
	num_steps(2147483647)
{
}

void Solver::init(shared_ptr<ObjectiveFunction> objective, const Eigen::VectorXd& X0)
{
	this->objective = objective;
	m_x = X0;
	m_x = (m_x.array() + 1).cwiseAbs2();

	ext_x = m_x;
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
		update_external_data();
	} while ((a_parameter_was_updated || test_progress()) && !halt && ++steps < num_steps);
	is_running = false;
	cout << "solver stopped" << endl;
	return ret;
}

void Solver::stop()
{
	wait_for_parameter_update_slot();
	halt = true;
	release_slot();
}

void Solver::update_external_data()
{
	give_parameter_update_slot();
	unique_lock<shared_timed_mutex> lock(*data_mutex);
	internal_update_external_data();
	progressed = true;
}

void Solver::get_data(Eigen::VectorXd& X)
{
	unique_lock<shared_timed_mutex> lock(*data_mutex);
	X = ext_x;
	progressed = false;
}

void Solver::give_parameter_update_slot()
{
	a_parameter_was_updated = false;
	unique_lock<mutex> lock(*parameters_mutex);
	params_ready_to_update = true;
	param_cv->notify_one();
	while (wait_for_param_update)
	{
		param_cv->wait(lock);
		a_parameter_was_updated = true;
	}
	params_ready_to_update = false;
}

void Solver::wait_for_parameter_update_slot()
{
	unique_lock<mutex> lock(*parameters_mutex);
	wait_for_param_update = true;
	while (!params_ready_to_update && is_running)
		param_cv->wait_for(lock, chrono::milliseconds(50));
}

void Solver::release_slot()
{
	wait_for_param_update = false;
	param_cv->notify_one();
}