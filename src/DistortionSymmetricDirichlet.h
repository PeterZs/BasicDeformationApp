#pragma once

#include "Utils.h"
#include <functional>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include "ObjectiveFunction.h"

using namespace Eigen;
typedef Triplet<double> T;
typedef SparseMatrix<double> SpMat;
typedef Matrix<double, 6, 6> Matrix6d;
typedef Matrix<double, 6, 1> Vector6d;

class DistortionSymmetricDirichlet : public ObjectiveFunction
{

public:

	/**************************************************************************************************************************/
	//INITIALIZATION 
	DistortionSymmetricDirichlet();

	virtual void init();
	virtual void updateX(const VectorXd& X);
	virtual double value();
	virtual void gradient(VectorXd& g);
	virtual void hessian();

	//loop implementation
	void prepare_hessian();
	/****************************************************************************************************************************/
	double bound=0;
	Eigen::MatrixX3i F;
	Eigen::MatrixX2d V;

	int numV;
	int numE;
	int numS;
	int numF;

	//Jacobian of the parameterization per face
	VectorXd a;
	VectorXd b;
	VectorXd c;
	VectorXd d;
	//Eigen::MatrixXd Juv;		//[a,b,c,d]
	//Eigen::MatrixXd invJuv;	//the order and the signs isn't important because the energy is squared anyway thus will be [a,b,c,d]*1/(ad-bc)
	VectorXd detJuv;		//(ad-bc)
	VectorXd invdetJuv;	//1/(ad-bc)
	SparseMatrix<double> DdetJuv_DUV; //jacobian of the function (detJuv) by UV

	//singular values
	MatrixX2d s; //Singular values s[0]>s[1]
	MatrixX4d v; //Singular vectors 
	MatrixX4d u; //Singular vectors 
	MatrixXd Dsd[2]; //singular values dense derivatives s[0]>s[1]

	//SVD methods
	bool updateJ(const VectorXd& X);
	void UpdateSSVDFunction();
	void ComputeDenseSSVDDerivatives();


	inline Matrix6d ComputeFaceConeHessian(const Vector6d& A1, const Vector6d& A2, double a1x, double a2x);
	inline Matrix6d ComputeConvexConcaveFaceHessian(const Vector6d& a1, const Vector6d& a2, const Vector6d& b1, const Vector6d& b2, double aY, double bY, double cY, double dY, const Vector6d& dSi, const Vector6d& dsi, double gradfS, double gradfs, double HS, double Hs);

	//Energy parts
	//distortion
	Eigen::VectorXd Efi;     //Efi=sum(Ef_dist.^2,2), for data->Efi history

	Eigen::MatrixXi Fuv;                             //F of cut mesh for u and v indices 6XnumF
	Eigen::VectorXd Area;
	Eigen::Matrix3Xd D1d, D2d;						//dense mesh derivative matrices

	Eigen::SparseMatrix<double> a1, a1t, a2, a2t, b1, b1t, b2, b2t;     //constant matrices for cones calcualtion
	Eigen::MatrixXd a1d, a2d, b1d, b2d;					//dense constant matrices for cones calcualtion

//per face hessians vector
   std::vector<Eigen::Matrix<double,6,6>> Hi;
private:
	int n;
};