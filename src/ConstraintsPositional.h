#pragma once

#include <Eigen/Core>
#include <Eigen/Sparse>
#include "ObjectiveFunction.h"

class ConstraintsPositional : public ObjectiveFunction
{

public:

	/**************************************************************************************************************************/
	//INITIALIZATION 
	ConstraintsPositional();

	virtual void init();
	virtual void updateX(const VectorXd& X);
	virtual double value();
	virtual void gradient(Eigen::VectorXd& g);
	virtual void hessian();

	//loop implementation
	void prepare_hessian();

	std::vector<int> ConstrainedVerticesInd;
	Eigen::MatrixX2d ConstrainedVerticesPos;
	Eigen::MatrixX2d CurrConstrainedVerticesPos;
	int numV;

private:
	int n;
};