#pragma once
#include "EigenTypes.h"

template <typename vectorTypeI, typename vectorTypeS>
class EigenSparseSolver
{
public:
	EigenSparseSolver();
	~EigenSparseSolver();
	void set_pattern(const vectorTypeI &II, const vectorTypeI &JJ, const vectorTypeS &SS);
	void analyze_pattern();
	bool factorize(const vectorTypeI &II, const vectorTypeI &JJ, const vectorTypeS &SS);
	Vec solve(Eigen::VectorXd &rhs);
private:
	SolverLU solver;
	SpMat A;
};

