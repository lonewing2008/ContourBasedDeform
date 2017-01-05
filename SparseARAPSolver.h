#ifndef MY_PAPER_ARAP
#define MY_PAPER_ARAP

/*Inclusion of C/C++ standard library*/
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include <iostream>
#include <fstream>
#include <vector>
using namespace std;

/*Inclusion of Eigen*/
#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <Eigen/SparseQR>
using namespace Eigen;

/*Inclusion of OpenGL*/
#include "glew.h"
#include "wglew.h"
#include "glut.h"
#include "glaux.h"

/*Other inclusion*/
#include "mesh.h"

/*Global variable*/

class SparseARAPSolver{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	SparseARAPSolver(bool bScaleVariantSetting = true);
	~SparseARAPSolver();

	bool InitialARAPMatrix(mesh* meshTargetModel);
	bool SetModel(mesh* meshTargetModel);

	bool IsInitialized(){return bIsInitialized;};
	bool IsScaleVariant(){return bIsScaleVariant;};

	bool SetScaleVariant(bool bScaleVariantSetting);

	bool UpdateConstraint(int iVertexID, double* dVertexPos);
	bool RemoveConstraint(int iVertexID);
	bool ResetConstraint();

	bool DeriveResult();
	bool ApplyDeform();
	bool ApplyDeform(bool bScaleVariantSetting);
	bool GetResult(double** dResultPos);
	bool GetResult(double** dResultPos, bool bScaleVariantSetting);
	bool GetVertexPosition(double** dVertexPos);

private:
	//Member Variable

	SparseMatrix<double, ColMajor> spmtrxSclFreeEdgeVector;
	SparseMatrix<double, ColMajor> spmtrxSclFreeEdgeConstr;
	SparseMatrix<double, ColMajor> spmtrxSclFreeError;
	SparseMatrix<double, ColMajor> spmtrxSclFreeErrorCoef;
	SparseMatrix<double, ColMajor> spmtrxSclFreeResult;

	SparseQR<SparseMatrix<double, ColMajor>, COLAMDOrdering<int>> spqrSclFreeSolver;
	
	SparseMatrix<double, ColMajor> spmtrxRotation;
	SparseMatrix<double, ColMajor> spmtrxRotationAngle;

	SparseMatrix<double, ColMajor> spmtrxSclVarEdgeVector;
	SparseMatrix<double, ColMajor> spmtrxSclVarEdgeConstr;
	SparseMatrix<double, ColMajor> spmtrxSclVarError;
	SparseMatrix<double, ColMajor> spmtrxSclVarErrorCoef;
	SparseMatrix<double, ColMajor> spmtrxSclVarResult;

	SparseQR<SparseMatrix<double, ColMajor>, COLAMDOrdering<int>> spqrSclVarSolver;

	bool     bIsInitialized;
	bool     bIsScaleVariant;

	int      iConstraintNum;

	int      iVertexNum;
	int      iEdgeNum;
	int**    iEdgeRelation;

	bool*    bIsConstraint;

	double** dInitVertexPos;

	mesh*    meshDeformingModel;

	//Member Function
	bool InitializeMatrices();
	bool TerminateMatrices();

	bool AnalyzeEdgeRelation();
	bool InitializeEdgeVector();
};
#endif