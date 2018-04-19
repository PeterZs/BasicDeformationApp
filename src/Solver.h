#pragma once

#include "TotalObjective.h"
#include "EigenTypes.h"

#include <atomic>
#include <functional>
#include <shared_mutex>
using namespace std;

class Solver
{
public:
	Solver();

	int run();
	void stop();
	void get_data(Eigen::VectorXd& X);
	void init(shared_ptr<ObjectiveFunction> objective, const Eigen::VectorXd& X0);

	// Pointer to the energy class
	shared_ptr<ObjectiveFunction> objective;

	// Activity flags
	atomic_bool is_running = {false};
	atomic_bool progressed = {false};

	// Answer from explicit solver after step done
	int ret;

	// Synchronization functions used by the wrapper
	void wait_for_parameter_update_slot();
	void release_slot();

	// vertices and face
	Eigen::MatrixX2d Vdef;
	MatX3i F;

	// External (interface) and internal working mesh
	Vec ext_x, m_x;

	int num_steps = 2147483647;

	bool full_init_done = false;

protected:
	// Give the wrapper a chance to intersect gracefully
	void give_parameter_update_slot();
	// Updating the data after a step has been done
	void update_external_data();

	// Descent direction evaluated in step
	Vec p;
	
	// Current energy, gradient and hessian
	double f;
	Vec g;
	SpMat h;

	// Synchronization structures
	atomic_bool params_ready_to_update = {false};
	atomic_bool wait_for_param_update = {false};
	atomic_bool a_parameter_was_updated = {false};
	atomic_bool halt = {false};
	
	// Mutex needed in lbfgs - thus protected
	unique_ptr<shared_timed_mutex> data_mutex;

	// pardiso variables
	vector<int> IId, JJd, IIp, JJp, II, JJ;
	vector<double> SSd, SSp, SS;

private:
	virtual int step() = 0;
	virtual void linesearch() = 0;
	virtual bool test_progress() = 0;
	virtual void internal_init() = 0;
	virtual void internal_update_external_data() = 0;

	// Mutex stuff
	unique_ptr<mutex> parameters_mutex;
	unique_ptr<condition_variable> param_cv;
};