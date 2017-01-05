#ifndef MY_PAPER_MFFD
#define MY_PAPER_MFFD

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
#include "SparseARAPSolver.h"
#include "mesh.h"

/*Global variable*/

class SparseMFFDSolver{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	SparseMFFDSolver();
	~SparseMFFDSolver();
	
	bool InitialMFFDMatrix(mesh* meshTargetModel, SparseARAPSolver* ARAPmtrxPtr);
	bool SetModel(mesh* meshTargetModel);

	bool IsInitialized(){return bIsInitialized;};
	bool SetConstraintList(bool* bConstraintID);

	bool UpdateConstraint();
	bool ResetConstraint();

	bool DeriveResult();
	bool DeriveResult(int iStartIter, int EndIter);

private:
	//Member Variable

	SparseARAPSolver* ARAPmtrxSolver;
	
	SparseMatrix<double, ColMajor> spmtrxResult[3];

	bool     bIsInitialized;

	int      iConstraintNum;
	bool*    bIsConstraint;
	double** dConstraintPos;

	double   dAxisRange[3][2];
	double   dMaxAxisLength;
	double** dNormalizedPos;

	int      iVertexNum;
	int      iEdgeNum;
	int**    iEdgeRelation;

	mesh*    meshDeformingModel;

	//Member Function
	bool InitializeModelVariance();
	bool TerminateModelVariance();

	bool InitializeMatrices(int iDeformLevel);
	bool TerminateMatrices();
	bool ApplyDeform(int iDeformLevel);

	double ErrorCalculate(int iDeformLevel);
	double BernPoly(double dVar, int iDegree, int iLevel);

	bool AnalyzeEdgeRelation();
	bool SimpleContourDetect();

	bool ModelNormalize();
	inline double AxisLength(int iAxis);

	inline int LatticeSize(int iDeformLevel);
};

#endif