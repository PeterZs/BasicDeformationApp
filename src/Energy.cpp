#include "Energy.h"
Energy::Energy()
	:
//  	position(make_unique<Position>()),
	symDirichlet(make_unique<DistortionSymDir>())
{
}

void Energy::init(const Eigen::MatrixX3d& V, const Eigen::MatrixX3i& F)
{
 	//position->init(F, V.rows());
	
	symDirichlet->V = V;
	symDirichlet->F = F;
	symDirichlet->init();
}

void Energy::evaluate_f(const Vec& x, double& f)
{
	double fd;
	symDirichlet->updateX(x);
	fd = symDirichlet->value();

// 	double fp;
// 	Vec gp;
// 	SpMat hp;
// 	position->evaluate_fgh(X, fp, gp, hp, Position::eval_mode::F);

	f = fd;
}

void Energy::evaluate_fgh(const Vec& x, double& f, Vec& g, SpMat& h)
{
	double fs, fd;
	Vec gs, gd;
	SpMat hd;

	// Distortion
	symDirichlet->updateX(x);
	fd = symDirichlet->value();
	symDirichlet->gradient(gd);
	symDirichlet->hessian();


	// Position
// 	double fp;
// 	Vec gp;
// 	SpMat hp;
// 	position->evaluate_fgh(X, fp, gp, hp, Position::eval_mode::FGH);

	f = fd;
	g = gd;
}