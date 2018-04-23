#pragma once


#include "ObjectiveFunction.h"
#include "DistortionSymmetricDirichlet.h"
#include "ConstraintsPositional.h"

#include <memory>

using namespace std;

class TotalObjective : public ObjectiveFunction
{
public:
	TotalObjective();
	virtual void init();
	virtual void updateX(const VectorXd& X);
	virtual double value();
	virtual void gradient(VectorXd& g);
	virtual void hessian();

	virtual void prepare_hessian();

	// sub objectives
	vector<ObjectiveFunction*> objectiveList;
	ConstraintsPositional constraintsPositional;
	DistortionSymmetricDirichlet symDirichlet;
};