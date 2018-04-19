#include "Newton.h"

#include <chrono>
#include <igl/flip_avoiding_line_search.h>

Newton::Newton() {}

int Newton::step()
{
	objective->updateX(m_x);
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
	return 0;
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
	
	objective->updateX(m_x);
	objective->hessian();

	if (needs_init)
	{ 
		pardiso->set_pattern(objective->II, objective->JJ, objective->SS);
		pardiso->analyze_pattern();
	}
}

void Newton::internal_update_external_data()
{
	diff_norm = (ext_x - m_x).norm();
	ext_x = m_x;
}

void Newton::linesearch()
{
	Eigen::MatrixXd m_x2 = Eigen::Map<Eigen::MatrixX2d>(m_x.data(), m_x.rows() / 2, 2);
	Eigen::MatrixXd p2 = Eigen::Map<const Eigen::MatrixX2d>(p.data(), p.rows() / 2, 2);
	Eigen::MatrixXd m_plus_p = m_x2 + p2;
	double alpha = igl::flip_avoiding_line_search(F, m_x2, m_plus_p, bind(&Newton::eval_ls, this, placeholders::_1));
	m_x = Eigen::Map<Vec>(m_x2.data(), m_x2.rows() * m_x2.cols());
}

double Newton::eval_ls(Eigen::MatrixXd& x)
{
	double f;
	Vec g;
	SpMat h;
	Vec vec_x = Eigen::Map<Vec>(x.data(), x.rows()  * x.cols(), 1);
	objective->updateX(vec_x);
	f=objective->value();
	return f;
}