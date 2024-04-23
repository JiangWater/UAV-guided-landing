#include "mpc.h"

void setDynamicsMatrices(Eigen::Matrix<double, 6, 6> &a, Eigen::Matrix<double, 6, 3> &b, float dt)
{
	a << 1, dt, 0, 0, 0, 0,
		0, 1, 0, 0, 0, 0,
		0, 0, 1, dt, 0, 0,
		0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 1, dt,
		0, 0, 0, 0, 0, 1;

	b << 0, 0, 0,
		dt, 0, 0,
		0, 0, 0,
		0, dt, 0,
		0, 0, 0,
		0, 0, dt;
}

void setInequalityConstraints(Eigen::Matrix<double, 6, 1> &xMax, Eigen::Matrix<double, 6, 1> &xMin,
							  Eigen::Matrix<double, 3, 1> &uMax, Eigen::Matrix<double, 3, 1> &uMin)
{
	uMin << -5.0,
		-5.0,
		-2.0;

	uMax << 5.0,
		5.0,
		2.0;

	xMin << -OsqpEigen::INFTY, -OsqpEigen::INFTY, -OsqpEigen::INFTY,
		-OsqpEigen::INFTY, -OsqpEigen::INFTY, -OsqpEigen::INFTY;

	xMax << OsqpEigen::INFTY, OsqpEigen::INFTY, OsqpEigen::INFTY,
		OsqpEigen::INFTY, OsqpEigen::INFTY, OsqpEigen::INFTY;
}

void setWeightMatrices(Eigen::DiagonalMatrix<double, 6> &Q, Eigen::DiagonalMatrix<double, 3> &R, Eigen::DiagonalMatrix<double, 6> &Q_n)
{
	Q.diagonal() << 120, 5, 120, 5, 80, 1.5;	// 120, 5, 120, 5, 80, 1.5;
	R.diagonal() << 1, 1, 1;					// 1, 1, 1;
	Q_n.diagonal() << 200, 10, 200, 10, 120, 2; // 200, 10, 200, 10, 120, 2;
}

void setHessianMatrices(const Eigen::DiagonalMatrix<double, 6> &Q, const Eigen::DiagonalMatrix<double, 6> &Q_n, const Eigen::DiagonalMatrix<double, 3> &R, const int &Np, const int &Nc, const int &NumStateVariable, const int &NumInputVariable, Eigen::SparseMatrix<double> &hessianMatrix)
{
	hessianMatrix.resize(NumStateVariable * (Np + 1) + NumInputVariable * Np, NumStateVariable * (Np + 1) + NumInputVariable * Np);

	for (int i = 0; i < NumStateVariable * (Np + 1) + NumInputVariable * Np; i++)
	{
		if (i < NumStateVariable * Np)
		{
			int posQ = i % NumStateVariable;
			float value = Q.diagonal()[posQ];
			if (value != 0)
				hessianMatrix.insert(i, i) = value;
		}
		else if (i >= NumStateVariable * Np && i < NumStateVariable * (Np + 1))
		{
			int posQ_n = i % NumStateVariable;
			float value = Q_n.diagonal()[posQ_n];
			if (value != 0)
				hessianMatrix.insert(i, i) = value;
		}
		else
		{
			int posR = i % NumInputVariable;
			float value = R.diagonal()[posR];
			if (value != 0)
				hessianMatrix.insert(i, i) = value;
		}
	}
}

/*
void PredictMatrices(Eigen::MatrixXd &F, Eigen::MatrixXd &Phi, Eigen::MatrixXd A, Eigen::MatrixXd B, int Np, int Nc)
{
	int m1, n1, n_in, n;
	m1 = A.rows();
	n1 = A.cols();
	n_in = B.cols();

	n = n1 + m1;

	Eigen::MatrixXd h = Eigen::MatrixXd::Zero(Nc * m1, n1);
	F = Eigen::MatrixXd::Zero(Np * m1, n1);

	h.topRows(m1) = h.Identity(m1, m1);
	F.topRows(m1) = F.Identity(m1, m1) * A;

	for (int kk = 2; kk <= Np; kk++)
	{
		h.middleRows(kk * m1 - m1, m1) = h.middleRows((kk - 1) * m1 - m1, m1) * A;
		F.middleRows(kk * m1 - m1, m1) = F.middleRows((kk - 1) * m1 - m1, m1) * A;
	}

	Eigen::MatrixXd v = Eigen::MatrixXd::Zero(Nc * m1, n_in);
	v = h * B;

	Phi = Eigen::MatrixXd::Zero(Np * m1, Nc * n_in);
	Phi.leftCols(n_in) = v;

	for (int i = 2; i <= Nc; i++)
	{
		Phi.block((i - 1) * m1, (i - 1) * n_in, Np * m1 - (i - 1) * m1, i * n_in - (i - 1) * n_in) = v.topRows((Np - i + 1) * m1);
	}
}

void WeightMatrices(Eigen::MatrixXd &Q_bar, Eigen::MatrixXd &R_bar, Eigen::DiagonalMatrix<double, 6> &Q, Eigen::DiagonalMatrix<double, 3> &R, Eigen::DiagonalMatrix<double, 6> &Q_terminal, int Np, int NumStateVariable, int NumInputVariable)
{
	Q_bar.setZero(NumStateVariable * Np, NumStateVariable * Np);
	R_bar.setZero(NumInputVariable * Np, NumInputVariable * Np);
	int j;
	for (int i = 0; i < ((Np - 1) * NumStateVariable); i++)
	{
		j = i % NumStateVariable;
		float value = Q.diagonal()[j];
		Q_bar(i, i) = value;
	}

	for (int i = ((Np - 1) * NumStateVariable); i < (Np * NumStateVariable); i++)
	{
		j = i % NumStateVariable;
		float value = Q_terminal.diagonal()[j];
		Q_bar(i, i) = value;
	}

	for (int i = 0; i < (Np * NumInputVariable); i++)
	{
		j = i % NumInputVariable;
		float value = R.diagonal()[j];
		R_bar(i, i) = value;
	}
}

void QuadProgMatrices(Eigen::MatrixXd &H, Eigen::MatrixXd &C, Eigen::MatrixXd &Q_bar, Eigen::MatrixXd &R_bar, Eigen::MatrixXd &F, Eigen::MatrixXd &Phi, int Np, int NumStateVariable, int NumInputVariable, Eigen::SparseMatrix<double> &hessianMatrix)
{
	hessianMatrix.resize(Np * NumInputVariable, Np * NumInputVariable);
	H = Phi.transpose() * Q_bar * Phi + R_bar;
	C = Phi.transpose() * Q_bar * F;
	for (int i = 0; i < H.rows(); i++)
	{
		for (int j = 0; j < H.cols(); j++)
		{
			float value = H(i, j);
			if (value != 0)
			{
				hessianMatrix.insert(i, j) = value;
			}
		}
	}
}
*/

void setGradientMatrices(const Eigen::DiagonalMatrix<double, 6> &Q, const Eigen::DiagonalMatrix<double, 6> &Q_n, const Eigen::Matrix<double, 6, 1> &xRef, const int &Np, const int &Nc, const int &NumStateVariable, const int &NumInputVariable, Eigen::VectorXd &gradient)
{
	Eigen::Matrix<double, 6, 1> Qx_ref, Qxn_ref;
	Qx_ref = Q * (-xRef);
	Qxn_ref = Q_n * (-xRef);

	gradient = Eigen::VectorXd::Zero(NumStateVariable * (Np + 1) + NumInputVariable * Np, 1);
	for (int i = 0; i < NumStateVariable * (Np + 1); i++)
	{
		if (i < NumStateVariable * Np)
		{
			int posQ = i % NumStateVariable;
			float value = Qx_ref(posQ, 0);
			gradient(i, 0) = value;
		}
		else
		{
			int posQ = i % NumStateVariable;
			float value = Qxn_ref(posQ, 0);
			gradient(i, 0) = value;
		}
	}
}

void setConstraintMatrix(const Eigen::Matrix<double, 6, 6> &dynamicMatrix, const Eigen::Matrix<double, 6, 3> &controlMatrix, const int &Np, const int &Nc, const int &NumStateVariable, const int &NumInputVariable, Eigen::SparseMatrix<double> &constraintMatrix)
{
	constraintMatrix.resize(NumStateVariable * (Np + 1) + NumStateVariable * (Np + 1) + NumInputVariable * Np, NumStateVariable * (Np + 1) + NumInputVariable * Np);

	for (int i = 0; i < NumStateVariable * (Np + 1); i++)
	{
		constraintMatrix.insert(i, i) = -1;
	}

	for (int i = 0; i < Np; i++)
	{
		for (int j = 0; j < NumStateVariable; j++)
		{
			for (int k = 0; k < NumStateVariable; k++)
			{
				float value = dynamicMatrix(j, k);
				if (value != 0)
				{
					constraintMatrix.insert(NumStateVariable * (i + 1) + j, NumStateVariable * i + k) = value;
				}
			}
		}
	}

	for (int i = 0; i < Np; i++)
	{
		for (int j = 0; j < NumStateVariable; j++)
		{
			for (int k = 0; k < NumInputVariable; k++)
			{
				float value = controlMatrix(j, k);
				if (value != 0)
				{
					constraintMatrix.insert(NumStateVariable * (i + 1) + j, NumInputVariable * i + k + NumStateVariable * (Np + 1)) = value;
				}
			}
		}
	}

	for (int i = 0; i < NumStateVariable * (Np + 1) + NumInputVariable * Np; i++)
	{
		constraintMatrix.insert(i + (Np + 1) * NumStateVariable, i) = 1;
	}
}

void setConstraintVectors(const Eigen::Matrix<double, 6, 1> &xMax, const Eigen::Matrix<double, 6, 1> &xMin, const Eigen::Matrix<double, 3, 1> &uMax, const Eigen::Matrix<double, 3, 1> &uMin, const Eigen::Matrix<double, 6, 1> &x0, const int &Np, const int &Nc, const int &NumStateVariable, const int &NumInputVariable, Eigen::VectorXd &lowerBound, Eigen::VectorXd &upperBound)
{
	Eigen::VectorXd lowerInequality = Eigen::MatrixXd::Zero(NumStateVariable * (Np + 1) + NumInputVariable * Np, 1);
	Eigen::VectorXd upperInequality = Eigen::MatrixXd::Zero(NumStateVariable * (Np + 1) + NumInputVariable * Np, 1);

	for (int i = 0; i < Np + 1; i++)
	{
		lowerInequality.block(NumStateVariable * i, 0, NumStateVariable, 1) = xMin;
		upperInequality.block(NumStateVariable * i, 0, NumStateVariable, 1) = xMax;
	}
	for (int i = 0; i < Np; i++)
	{
		lowerInequality.block(NumInputVariable * i + NumStateVariable * (Np + 1), 0, NumInputVariable, 1) = uMin;
		upperInequality.block(NumInputVariable * i + NumStateVariable * (Np + 1), 0, NumInputVariable, 1) = uMax;
	}

	Eigen::VectorXd lowerEquality = Eigen::MatrixXd::Zero(NumStateVariable * (Np + 1), 1);
	Eigen::VectorXd upperEquality;
	lowerEquality.block(0, 0, NumStateVariable, 1) = -x0;
	upperEquality = lowerEquality;
	lowerEquality = lowerEquality;

	lowerBound = Eigen::MatrixXd::Zero(2 * NumStateVariable * (Np + 1) + NumInputVariable * Np, 1);
	lowerBound << lowerEquality,
		lowerInequality;

	upperBound = Eigen::MatrixXd::Zero(2 * NumStateVariable * (Np + 1) + NumInputVariable * Np, 1);
	upperBound << upperEquality,
		upperInequality;
}

void updateConstraintVectors(const Eigen::Matrix<double, 6, 1> &x0, Eigen::VectorXd &lowerBound, Eigen::VectorXd &upperBound)
{
	lowerBound.block(0, 0, 6, 1) = -x0;
	upperBound.block(0, 0, 6, 1) = -x0;
}

/*
void ConstraintMatrices(int NumStateVariable, int NumInputVariable, int Np, Eigen::SparseMatrix<double> &constraintMatrix)
{
	constraintMatrix.resize(Np * NumInputVariable, Np * NumInputVariable);
	for (int i = 0; i < NumInputVariable; i++)
	{
		int j = i;
		constraintMatrix.insert(j, i) = 1;
	}
}

void ConstraintVectors(Eigen::Matrix<double, 3, 1> &uMax, Eigen::Matrix<double, 3, 1> &uMin, int NumInputVariable, int Np, Eigen::VectorXd &lowerBound, Eigen::VectorXd &upperBound)
{
	lowerBound = Eigen::MatrixXd::Zero(Np * NumInputVariable, 1);
	upperBound = Eigen::MatrixXd::Zero(Np * NumInputVariable, 1);
	for (int i = 0; i < Np; i++)
	{
		lowerBound.block(NumInputVariable * i, 0, NumInputVariable, 1) = uMin;
		upperBound.block(NumInputVariable * i, 0, NumInputVariable, 1) = uMax;
	}
}

void GradientMatrices(MatrixXd &G, MatrixXd &C, Eigen::Matrix<double, 6, 1> x0, Eigen::Matrix<double, 6, 1> xRef, Eigen::VectorXd &gradient)
{
	G = C * (x0 - xRef);
	gradient.resize(G.rows(), 1);
	for (int i = 0; i < G.rows(); i++)
	{
		gradient(i) = G(i, 0);
	}
}
*/