#include "ConstraintsPositional.h"

using namespace std;
using namespace Eigen;

ConstraintsPositional::ConstraintsPositional()
{
	
}
void ConstraintsPositional::init()
{
	if(numV==0)
		throw "ConstraintsPositional must define members numV before init()!";
	prepare_hessian();
	w = 10000;
}
void ConstraintsPositional::updateX(const Eigen::VectorXd& X)
{
	CurrConstrainedVerticesPos.resizeLike(ConstrainedVerticesPos);
	for (int i = 0; i < ConstrainedVerticesInd.size(); i++)
	{
		CurrConstrainedVerticesPos.row(i) << X(ConstrainedVerticesInd[i]),X(ConstrainedVerticesInd[i] + numV);
	}
}

double ConstraintsPositional::value()
{
	return (ConstrainedVerticesPos - CurrConstrainedVerticesPos).squaredNorm();
}
void ConstraintsPositional::gradient(Eigen::VectorXd& g)
{
	MatrixXd diff = (CurrConstrainedVerticesPos - ConstrainedVerticesPos);
	g.conservativeResize(numV * 2);
	g.setZero();
	for (int i = 0; i < ConstrainedVerticesInd.size(); i++)
	{
		g(ConstrainedVerticesInd[i]) = 2*diff(i, 0);
		g(ConstrainedVerticesInd[i] + numV) = 2*diff(i, 1);
	}
}

void ConstraintsPositional::hessian()
{
	std::fill(SS.begin(), SS.end(), 0);
	for (int i = 0; i < ConstrainedVerticesInd.size(); i++)
	{
		SS[ConstrainedVerticesInd[i]] = 2; SS[ConstrainedVerticesInd[i] + numV] = 2;
	}
}


void ConstraintsPositional::prepare_hessian()
{
	II.resize(2*numV);
	JJ.resize(2*numV);
	for (int i = 0; i < 2*numV; i++)
	{
		II[i] = i;
		JJ[i] = i;
	}
	SS = vector<double>(II.size(), 0.);
}