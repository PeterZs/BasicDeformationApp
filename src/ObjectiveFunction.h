#pragma once
#include <Eigen/Core>
#include <vector>

using namespace Eigen;

class ObjectiveFunction
{
public:
	ObjectiveFunction();
	virtual ~ObjectiveFunction();
	virtual void init() = 0;
	virtual void updateX(const VectorXd& X) = 0;
	virtual double value() = 0;
	virtual void gradient(VectorXd& g) = 0;
	virtual void hessian() = 0;

	virtual void prepare_hessian() = 0;
	double w;

	// pardiso variables
	std::vector<int> II, JJ;
	std::vector<double> SS;
};

