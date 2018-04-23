#include "Newton.h"

#include <chrono>

Newton::Newton() {}

double Newton::step()
{
	objective->updateX(X);
	f = objective->value();
	objective->gradient(g);
	objective->hessian();

	pardiso->update_a(objective->SS);
	try
	{
		pardiso->factorize();
	}
	catch (runtime_error& err)
	{
		cout << err.what();
		return -1;
	}
	Vec rhs = -g;
	pardiso->solve(rhs, p);
	return f;
}

bool Newton::test_progress()
{
	return true;
}

void Newton::internal_init()
{
	bool needs_init = pardiso == nullptr;

	if (needs_init)
	{
		pardiso = make_unique<PardisoSolver<vector<int>, vector<double>>>();
		pardiso->set_type(2, true);
	}
	
	objective->updateX(X);
	g.resize(X.size());
	objective->hessian();

	if (needs_init)
	{ 
		pardiso->set_pattern(objective->II, objective->JJ, objective->SS);
		pardiso->analyze_pattern();
	}
}