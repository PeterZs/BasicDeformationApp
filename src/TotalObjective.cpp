#include "TotalObjective.h"
TotalObjective::TotalObjective()
{
	objectiveList.push_back(&symDirichlet);
	objectiveList.push_back(&constraintsPositional);
}

void TotalObjective::init()
{
	//assume that each objective's member have been set outside
	for (auto &objective : objectiveList)
		objective->init();

	prepare_hessian();
}

void TotalObjective::updateX(const VectorXd& X)
{
	for (auto &objective : objectiveList)
		objective->updateX(X);
}

double TotalObjective::value()
{
	double f=0;
	for (auto &objective : objectiveList)
		f+= objective->w*objective->value();

	return f;
}

void TotalObjective::gradient(VectorXd& g)
{
	VectorXd gi;
	g.setZero();
	for (auto &objective : objectiveList)
	{
		objective->gradient(gi);
		g += objective->w*gi;
	}
}

void TotalObjective::hessian()
{
	SS.clear();
	for (auto &objective : objectiveList)
	{
		objective->hessian();
		for (double &val : objective->SS)
			val *= objective->w;
		SS.insert(SS.end(), objective->SS.begin(), objective->SS.end());
	}
}

void TotalObjective::prepare_hessian()
{
	//assume that each subobjective already prepared its hessian
	II.clear(); JJ.clear(); SS.clear();
	for (auto &objective : objectiveList)
	{
		II.insert(II.end(), objective->II.begin(), objective->II.end());
		JJ.insert(JJ.end(), objective->JJ.begin(), objective->JJ.end());
		SS.insert(SS.end(), objective->SS.begin(), objective->SS.end());
	}
}