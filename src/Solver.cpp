#include "Solver.h"

#include "Utils.h"
#include "TotalObjective.h"

#include <iostream>
#include <igl/flip_avoiding_line_search.h>


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
	X = X0;
	ext_x = X;
	internal_init();
}

void Solver::setFlipAvoidingLineSearch(MatrixX3i & F)
{
	FlipAvoidingLineSearch = true;
	this->F = F;
}

int Solver::run()
{
	is_running = true;
	halt = false;
	int steps = 0;
	do
	{
		currentEnergy = step();
		linesearch();
		update_external_data();
	} while ((a_parameter_was_updated || test_progress()) && !halt && ++steps < num_steps);
	is_running = false;
	cout << "solver stopped" << endl;
	return 0;
}

void Solver::linesearch()
{
	Eigen::MatrixXd MatX = Eigen::Map<Eigen::MatrixX2d>(X.data(), X.rows() / 2, 2);
	Eigen::MatrixXd MatP = Eigen::Map<const Eigen::MatrixX2d>(p.data(), p.rows() / 2, 2);

	double step_size;
	auto funcation_evaluator = [this](MatrixXd& x) {
		objective->updateX(x);
		return objective->value();
	};
	if (FlipAvoidingLineSearch)
	{
		double min_step_to_singularity = igl::flip_avoiding::compute_max_step_from_singularities(MatX, F, MatP);
		step_size = std::min(1., min_step_to_singularity*0.8);
	}
	else
		step_size = 1;

	double new_energy = currentEnergy;
	
	int cur_iter = 0; int MAX_STEP_SIZE_ITER = 12;

	while (cur_iter < MAX_STEP_SIZE_ITER)
	{
		Eigen::MatrixXd curr_x = X + step_size * p;

		objective->updateX(curr_x);
		new_energy = objective->value();
		if (new_energy >= currentEnergy)
		{
			step_size /= 2;
		}
		else
		{
			MatX = curr_x;
			break;
		}
		cur_iter++;
	}
}


void Solver::stop()
{
	wait_for_parameter_update_slot();
	halt = true;
	release_parameter_update_slot();
}

void Solver::update_external_data()
{
	give_parameter_update_slot();
	unique_lock<shared_timed_mutex> lock(*data_mutex);
	ext_x = X;
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

void Solver::release_parameter_update_slot()
{
	wait_for_param_update = false;
	param_cv->notify_one();
}