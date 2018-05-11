#include "EigenSparseSolver.h"
#include <vector>
using namespace std;

template <typename vectorTypeI, typename vectorTypeS>
EigenSparseSolver<vectorTypeI, vectorTypeS>::EigenSparseSolver()
{
}

template <typename vectorTypeI, typename vectorTypeS>
EigenSparseSolver<vectorTypeI, vectorTypeS>::~EigenSparseSolver()
{
}

template <typename vectorTypeI, typename vectorTypeS>
void EigenSparseSolver<vectorTypeI, vectorTypeS>::set_pattern(const vectorTypeI &II, const vectorTypeI &JJ, const vectorTypeS &SS)\
{
	std::vector<Tripletd> tripletList;
	tripletList.reserve(II.size());
	int rows = *std::max_element(II.begin(), II.end()) + 1;
	int cols = *std::max_element(JJ.begin(), JJ.end()) + 1;
	assert(rows == cols && "Rows == Cols at Newton internal init");
	for (int i = 0; i<II.size(); i++)
		tripletList.push_back(Tripletd(II[i], JJ[i], SS[i]));
	A = SpMat(rows, cols);
	A.setFromTriplets(tripletList.begin(), tripletList.end());
}

template <typename vectorTypeI, typename vectorTypeS>
void EigenSparseSolver<vectorTypeI, vectorTypeS>::analyze_pattern()
{
	solver.analyzePattern(A);
}

template <typename vectorTypeI, typename vectorTypeS>
bool EigenSparseSolver<vectorTypeI, vectorTypeS>::factorize(const vectorTypeI &II, const vectorTypeI &JJ, const vectorTypeS &SS)
{
	std::vector<Tripletd> tripletList;
	tripletList.reserve(II.size());
	int rows = *std::max_element(II.begin(), II.end()) + 1;
	int cols = *std::max_element(JJ.begin(), JJ.end()) + 1;
	assert(rows == cols && "Rows == Cols at Newton internal init");
	for (int i = 0; i<II.size(); i++)
		tripletList.push_back(Tripletd(II[i], JJ[i], SS[i]));
	A.resize(rows, cols);
	A.setFromTriplets(tripletList.begin(), tripletList.end());
	solver.factorize(A);
}

template <typename vectorTypeI, typename vectorTypeS>
Vec EigenSparseSolver<vectorTypeI, vectorTypeS>::solve(Eigen::VectorXd &rhs)
{
	return solver.solve(rhs);
}

template class EigenSparseSolver<std::vector<int, std::allocator<int> >, std::vector<double, std::allocator<double> > >;

//template class EigenSparseSolver<Eigen::Matrix<int, -1, 1, 0, -1, 1>, Eigen::Matrix<double, -1, 1, 0, -1, 1> >;