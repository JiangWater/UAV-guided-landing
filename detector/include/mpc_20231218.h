#include <math.h>
#include <Eigen/Dense>
#include "OsqpEigen/OsqpEigen.h"

using namespace Eigen;
using namespace std;

void setDynamicsMatrices(Eigen::Matrix<double, 6, 6> &a, Eigen::Matrix<double, 6, 3> &b, float dt);
void setInequalityConstraints(Eigen::Matrix<double, 6, 1> &xMax, Eigen::Matrix<double, 6, 1> &xMin, Eigen::Matrix<double, 3, 1> &uMax, Eigen::Matrix<double, 3, 1> &uMin);
void setWeightMatrices(Eigen::DiagonalMatrix<double, 6> &Q, Eigen::DiagonalMatrix<double, 3> &R, Eigen::DiagonalMatrix<double, 6> &Q_n);
void setHessianMatrices(const Eigen::DiagonalMatrix<double, 6> &Q, const Eigen::DiagonalMatrix<double, 6> &Q_n, const Eigen::DiagonalMatrix<double, 3> &R, const int &Np, const int &Nc, const int &NumStateVariable, const int &NumInputVariable, Eigen::SparseMatrix<double> &hessianMatrix);
// void PredictMatrices(Eigen::MatrixXd &F, Eigen::MatrixXd &Phi, Eigen::MatrixXd A, Eigen::MatrixXd B, int Np, int Nc);
// void WeightMatrices(Eigen::MatrixXd &Q_bar, Eigen::MatrixXd &R_bar, Eigen::DiagonalMatrix<double, 6> &Q, Eigen::DiagonalMatrix<double, 3> &R, Eigen::DiagonalMatrix<double, 6> &Q_n, int Np, int NumStateVariable, int NumInputVariable);
// void QuadProgMatrices(Eigen::MatrixXd &H, Eigen::MatrixXd &C, Eigen::MatrixXd &Q_bar, Eigen::MatrixXd &R_bar, Eigen::MatrixXd &F, Eigen::MatrixXd &Phi, int Np, int NumStateVariable, int NumInputVariable, Eigen::SparseMatrix<double> &hessianMatrix);
void setGradientMatrices(const Eigen::DiagonalMatrix<double, 6> &Q, const Eigen::DiagonalMatrix<double, 6> &Q_n, const Eigen::Matrix<double, 6, 1> &xRef, const int &Np, const int &Nc, const int &NumStateVariable, const int &NumInputVariable, Eigen::VectorXd &gradient);
void setConstraintMatrix(const Eigen::Matrix<double, 6, 6> &dynamicMatrix, const Eigen::Matrix<double, 6, 3> &controlMatrix, const int &Np, const int &Nc, const int &NumStateVariable, const int &NumInputVariable, Eigen::SparseMatrix<double> &constraintMatrix);
void setConstraintVectors(const Eigen::Matrix<double, 6, 1> &xMax, const Eigen::Matrix<double, 6, 1> &xMin, const Eigen::Matrix<double, 3, 1> &uMax, const Eigen::Matrix<double, 3, 1> &uMin, const Eigen::Matrix<double, 6, 1> &x0, const int &Np, const int &Nc, const int &NumStateVariable, const int &NumInputVariable, Eigen::VectorXd &lowerBound, Eigen::VectorXd &upperBound);
void updateConstraintVectors(const Eigen::Matrix<double, 6, 1> &x0, Eigen::VectorXd &lowerBound, Eigen::VectorXd &upperBound);
// void ConstraintMatrices(int NumStateVariable, int NumInputVariable, int Np, Eigen::SparseMatrix<double> &constraintMatrix);
// void ConstraintVectors(Eigen::Matrix<double, 3, 1> &uMax, Eigen::Matrix<double, 3, 1> &uMin, int NumInputVariable, int Np, Eigen::VectorXd &lowerBound, Eigen::VectorXd &upperBound);
// void GradientMatrices(MatrixXd &G, MatrixXd &C, Eigen::Matrix<double, 6, 1> x0, Eigen::Matrix<double, 6, 1> xRef, Eigen::VectorXd &gradient);
