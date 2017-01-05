/*Other inclusion*/
#include "SparseARAPSolver.h"
#include "Debug.h"

/*Global variable*/
#define globCONSTWEIGHT 1000

SparseARAPSolver::SparseARAPSolver(bool bScaleVariantSetting){
	bIsInitialized = false;
	bIsScaleVariant = bScaleVariantSetting;

	bIsConstraint  = NULL;
	dInitVertexPos = NULL;
	
	iVertexNum    = 0;
	iEdgeNum	  = 0;
	iEdgeRelation = NULL;

	meshDeformingModel = NULL;
};

SparseARAPSolver::~SparseARAPSolver(){
	TerminateMatrices();
};

bool SparseARAPSolver::InitialARAPMatrix(mesh* meshTargetModel){
	if(bIsInitialized) return false;
	
	meshDeformingModel = meshTargetModel;

	if(!(bIsInitialized = InitializeMatrices())) return false;

	return true;
};

bool SparseARAPSolver::SetModel(mesh* meshTargetModel){
	if(!TerminateMatrices())  return false;

	meshDeformingModel = meshTargetModel;

	if(!(bIsInitialized = InitializeMatrices())) return false;

	return true;
};

bool SparseARAPSolver::SetScaleVariant(bool bScaleVariantSetting){
	bIsScaleVariant = bScaleVariantSetting;

	return true;
};

bool SparseARAPSolver::UpdateConstraint(int iVertexID, double* dVertexPos){
	if(!bIsInitialized)        return false;
	if(dVertexPos == NULL)     return false;
	if(iVertexID < 0)          return false;
	if(iVertexID > iVertexNum) return false;

	if(!bIsConstraint[iVertexID]) iConstraintNum++;
	bIsConstraint[iVertexID] = true;

	spmtrxSclFreeEdgeConstr.coeffRef(iEdgeNum * 2 + iVertexID * 2,     iVertexID * 2)     = globCONSTWEIGHT;
	spmtrxSclFreeEdgeConstr.coeffRef(iEdgeNum * 2 + iVertexID * 2 + 1, iVertexID * 2 + 1) = globCONSTWEIGHT;

	spmtrxSclFreeErrorCoef.coeffRef(iEdgeNum * 2 + iVertexID * 2,     0) = globCONSTWEIGHT * dVertexPos[0];
	spmtrxSclFreeErrorCoef.coeffRef(iEdgeNum * 2 + iVertexID * 2 + 1, 0) = globCONSTWEIGHT * dVertexPos[1];

	spmtrxSclVarEdgeConstr.coeffRef(iEdgeNum * 2 + iVertexID * 2,     iVertexID * 2)     = globCONSTWEIGHT;
	spmtrxSclVarEdgeConstr.coeffRef(iEdgeNum * 2 + iVertexID * 2 + 1, iVertexID * 2 + 1) = globCONSTWEIGHT;
	
	spmtrxSclVarErrorCoef.coeffRef(iEdgeNum * 2 + iVertexID * 2,     0) = globCONSTWEIGHT * dVertexPos[0];
	spmtrxSclVarErrorCoef.coeffRef(iEdgeNum * 2 + iVertexID * 2 + 1, 0) = globCONSTWEIGHT * dVertexPos[1];

	return true;
};

bool SparseARAPSolver::RemoveConstraint(int iVertexID){
	if(!bIsInitialized)        return false;
	if(iVertexID < 0)          return false;
	if(iVertexID > iVertexNum) return false;

	if(bIsConstraint[iVertexID]) iConstraintNum--;
	bIsConstraint[iVertexID] = false;
	
	int iVertexNum = int(meshDeformingModel->vTotal);

	spmtrxSclFreeEdgeConstr.coeffRef(iVertexNum * 2 + iVertexID * 2,     iVertexID * 2)     = 0;
	spmtrxSclFreeEdgeConstr.coeffRef(iVertexNum * 2 + iVertexID * 2 + 1, iVertexID * 2 + 1) = 0;

	spmtrxSclFreeErrorCoef.coeffRef(iVertexNum * 2 + iVertexID * 2,     0) = 0;
	spmtrxSclFreeErrorCoef.coeffRef(iVertexNum * 2 + iVertexID * 2 + 1, 0) = 0;

	spmtrxSclVarEdgeConstr.coeffRef(iVertexNum * 2 + iVertexID * 2,     iVertexID * 2)     = 0;
	spmtrxSclVarEdgeConstr.coeffRef(iVertexNum * 2 + iVertexID * 2 + 1, iVertexID * 2 + 1) = 0;
	
	spmtrxSclVarErrorCoef.coeffRef(iVertexNum * 2 + iVertexID * 2,     0) = 0;
	spmtrxSclVarErrorCoef.coeffRef(iVertexNum * 2 + iVertexID * 2 + 1, 0) = 0;

	return true;
};

bool SparseARAPSolver::ResetConstraint(){
	if(!bIsInitialized) return false;

	iConstraintNum = 0;

	for(int iVertexCount = 0;iVertexCount < iVertexNum;iVertexCount++){
		bIsConstraint[iVertexCount] = false;
		for(int iTmp = 0;iTmp < 3;iTmp++) meshDeformingModel->vList[iVertexCount].ptr[iTmp] = dInitVertexPos[iVertexCount][iTmp];
	}

	spmtrxSclFreeEdgeConstr = spmtrxSclFreeEdgeVector;
	spmtrxSclFreeErrorCoef.setZero();

	spmtrxSclVarEdgeConstr  = spmtrxSclVarEdgeVector;
	spmtrxSclVarErrorCoef.setZero();

	return true;
};

bool SparseARAPSolver::DeriveResult(){
	if(iConstraintNum < 2)   return false;
	if(!bIsInitialized)      return false;

	int iCurrentTime[3];
	int iTimeTmp[2];

	iTimeTmp[0] = glutGet(GLUT_ELAPSED_TIME);

	spmtrxSclFreeError = spmtrxSclFreeEdgeConstr;
	spmtrxSclFreeError.makeCompressed();
	spmtrxSclFreeErrorCoef.makeCompressed();

	spmtrxSclFreeResult.setZero();
	spqrSclFreeSolver.analyzePattern(spmtrxSclFreeError);
	spqrSclFreeSolver.factorize(spmtrxSclFreeError);
	spqrSclFreeSolver.compute(spmtrxSclFreeError);
	spmtrxSclFreeResult = spqrSclFreeSolver.solve(spmtrxSclFreeErrorCoef);
	
	iTimeTmp[1] = glutGet(GLUT_ELAPSED_TIME);
	iCurrentTime[0] = iTimeTmp[1] - iTimeTmp[0];

#ifdef GLOB_DEBUG
	//cout<<spmtrxSclFreeResult.topRows(2 * iVertexNum)<<endl;
	//cout<<spmtrxRotation.topRows(2 * iEdgeNum)<<endl;
#endif
	
	iTimeTmp[0] = glutGet(GLUT_ELAPSED_TIME);

	SparseMatrix<double, ColMajor> spmtrxSclFrRsltCompressed = spmtrxSclFreeResult.topRows(iVertexNum * 2);
	spmtrxRotationAngle.setZero();
	spmtrxRotationAngle = spmtrxRotation * spmtrxSclFrRsltCompressed;
	
	double dTotalRotationAngle = 0.0;

	for(int iEdgeCount = 0;iEdgeCount < iEdgeNum;iEdgeCount++){

		Matrix<double, 2, 2> mtrxEdge;
		mtrxEdge.coeffRef(0, 0) =   dInitVertexPos[iEdgeRelation[iEdgeCount][1]][0] - dInitVertexPos[iEdgeRelation[iEdgeCount][0]][0];
		mtrxEdge.coeffRef(0, 1) =   dInitVertexPos[iEdgeRelation[iEdgeCount][1]][1] - dInitVertexPos[iEdgeRelation[iEdgeCount][0]][1];
		mtrxEdge.coeffRef(1, 0) =   dInitVertexPos[iEdgeRelation[iEdgeCount][1]][1] - dInitVertexPos[iEdgeRelation[iEdgeCount][0]][1];
		mtrxEdge.coeffRef(1, 1) = -(dInitVertexPos[iEdgeRelation[iEdgeCount][1]][0] - dInitVertexPos[iEdgeRelation[iEdgeCount][0]][0]);

		Matrix<double, 2, 1> mtrxRot;
		mtrxRot.coeffRef(0, 0) = spmtrxRotationAngle.coeff(2 * iEdgeCount    , 0);
		mtrxRot.coeffRef(1, 0) = spmtrxRotationAngle.coeff(2 * iEdgeCount + 1, 0);
		//cout<<"Edge #"<<iEdgeCount<<" : "<<endl;
		//cout<<mtrxEdge<<endl;
		//cout<<"Before Scale : "<<mtrxRot.transpose()<<endl;
		mtrxRot.normalize();
		//cout<<"After Scale  : "<<mtrxRot.transpose()<<endl;
		//cout<<"Rotation Angle : "<<asin(mtrxRot.coeff(1, 0)) / 3.1415926 * 180<<endl;
		dTotalRotationAngle += asin(mtrxRot.coeff(1, 0)) / 3.1415926 * 180;

		Matrix<double, 2, 1> mtrxRotatedEdge = mtrxEdge * mtrxRot;
		
		spmtrxSclVarErrorCoef.coeffRef(2 * iEdgeCount    , 0) = mtrxRotatedEdge.coeff(0, 0);
		spmtrxSclVarErrorCoef.coeffRef(2 * iEdgeCount + 1, 0) = mtrxRotatedEdge.coeff(1, 0);

		/*
		double dEdgeVec[2];
		double dEdgeRot[2];
		for(int iTmp = 0;iTmp < 2;iTmp++) dEdgeVec[iTmp] = dInitVertexPos[iEdgeRelation[iEdgeCount][1]][iTmp] - dInitVertexPos[iEdgeRelation[iEdgeCount][0]][iTmp];
		for(int iTmp = 0;iTmp < 2;iTmp++) dEdgeRot[iTmp] = spmtrxRotationAngle.coeff(2 * iEdgeCount + iTmp, 0);
		//cout<<"Edge #"<<iEdgeCount<<" : "<<endl;
		//cout<<"Before Scale : ("<<dEdgeRot[0]<<", "<<dEdgeRot[1]<<")"<<endl;
		double dEdgeScl = sqrt(dEdgeRot[0] * dEdgeRot[0] + dEdgeRot[1] * dEdgeRot[1]);
		for(int iTmp = 0;iTmp < 2;iTmp++) dEdgeRot[iTmp] /= dEdgeScl;
		//cout<<"After Scale  : ("<<dEdgeRot[0]<<", "<<dEdgeRot[1]<<")"<<endl;
		//cout<<"Rotation Angle : "<<double(acos(dEdgeRot[0])) / 3.1415926 * 180<<" "<<double(asin(dEdgeRot[1])) / 3.1415926 * 180<<endl;
		dTotalRotationAngle += double(asin(dEdgeRot[1])) / 3.1415926 * 180;

		spmtrxSclVarErrorCoef.coeffRef(iEdgeCount * 2,     0) = dEdgeVec[0] *  dEdgeRot[0] + dEdgeVec[1] * dEdgeRot[1];
		spmtrxSclVarErrorCoef.coeffRef(iEdgeCount * 2 + 1, 0) = dEdgeVec[0] * -dEdgeRot[1] + dEdgeVec[1] * dEdgeRot[0];
		*/
	}

	iTimeTmp[1] = glutGet(GLUT_ELAPSED_TIME);
	iCurrentTime[1] = iTimeTmp[1] - iTimeTmp[0];
	
#ifdef GLOB_DEBUG
	//cout<<spmtrxRotationAngle.topRows(2 * iVertexNum)<<endl;
#endif
	
	iTimeTmp[0] = glutGet(GLUT_ELAPSED_TIME);

	spmtrxSclVarError = spmtrxSclVarEdgeConstr;
	spmtrxSclVarError.makeCompressed();
	spmtrxSclVarErrorCoef.makeCompressed();

	spmtrxSclVarResult.setZero();
	spqrSclVarSolver.analyzePattern(spmtrxSclVarError);
	spqrSclVarSolver.factorize(spmtrxSclVarError);
	spqrSclVarSolver.compute(spmtrxSclVarError);
	spmtrxSclVarResult = spqrSclVarSolver.solve(spmtrxSclVarErrorCoef);
	
	iTimeTmp[1] = glutGet(GLUT_ELAPSED_TIME);
	iCurrentTime[2] = iTimeTmp[1] - iTimeTmp[0];

#ifdef GLOB_DEBUG
	//cout<<"spmtrxSclFreeError : "<<endl<<spmtrxSclFreeError.bottomRows(2 * iVertexNum)<<endl;
	//cout<<"spmtrxSclVarError : "<<endl<<spmtrxSclVarError.bottomRows(2 * iVertexNum)<<endl;

	//cout<<"spmtrxSclFreeErrorCoef : "<<endl<<spmtrxSclFreeErrorCoef.bottomRows(2 * iVertexNum)<<endl;
	//cout<<"spmtrxSclVarErrorCoef : "<<endl<<spmtrxSclVarErrorCoef.bottomRows(2 * iVertexNum)<<endl;

	//cout<<"spmtrxSclFreeResult : "<<endl<<spmtrxSclFreeResult.topRows(2 * iVertexNum)<<endl;
	//cout<<"spmtrxSclVarResult  : "<<endl<<spmtrxSclVarResult.topRows(2 * iVertexNum)<<endl;
	
	cout<<"Average Rotation Angle(Scale Free) = "<<dTotalRotationAngle / iEdgeNum<<endl;
	cout<<"Execution Time(Scale Free) : "<<iCurrentTime[0]<<" ms"<<endl;
	cout<<"Execution Time(Rotation) : "<<iCurrentTime[1]<<" ms"<<endl;
	cout<<"Execution Time(Scale Variant) : "<<iCurrentTime[2]<<" ms"<<endl;
#endif

	return true;
};

bool SparseARAPSolver::ApplyDeform(){
	if(bIsScaleVariant){
		for(int iVertexCount = 1;iVertexCount < iVertexNum;iVertexCount++){
			meshDeformingModel->vList[iVertexCount].ptr[0] = spmtrxSclVarResult.coeff(iVertexCount * 2,     0);
			meshDeformingModel->vList[iVertexCount].ptr[1] = spmtrxSclVarResult.coeff(iVertexCount * 2 + 1, 0);
#ifdef GLOB_DEBUG
			//cout<<meshDeformingModel->vList[iVertexCount].ptr[0]<<"  "<<meshDeformingModel->vList[iVertexCount].ptr[1]<<"  "<<meshDeformingModel->vList[iVertexCount].ptr[2]<<endl;
#endif
		}
	}
	else{
		for(int iVertexCount = 1;iVertexCount < iVertexNum;iVertexCount++){
			meshDeformingModel->vList[iVertexCount].ptr[0] = spmtrxSclFreeResult.coeff(iVertexCount * 2,     0);
			meshDeformingModel->vList[iVertexCount].ptr[1] = spmtrxSclFreeResult.coeff(iVertexCount * 2 + 1, 0);
#ifdef GLOB_DEBUG
			//cout<<meshDeformingModel->vList[iVertexCount].ptr[0]<<"  "<<meshDeformingModel->vList[iVertexCount].ptr[1]<<"  "<<meshDeformingModel->vList[iVertexCount].ptr[2]<<endl;
#endif
		}
	}
	return true;
};

bool SparseARAPSolver::ApplyDeform(bool bScaleVariantSetting){
	if(bScaleVariantSetting){
		for(int iVertexCount = 1;iVertexCount < iVertexNum;iVertexCount++){
			meshDeformingModel->vList[iVertexCount].ptr[0] = spmtrxSclVarResult.coeff(iVertexCount * 2,     0);
			meshDeformingModel->vList[iVertexCount].ptr[1] = spmtrxSclVarResult.coeff(iVertexCount * 2 + 1, 0);
#ifdef GLOB_DEBUG
			//cout<<meshDeformingModel->vList[iVertexCount].ptr[0]<<"  "<<meshDeformingModel->vList[iVertexCount].ptr[1]<<"  "<<meshDeformingModel->vList[iVertexCount].ptr[2]<<endl;
#endif
		}
	}
	else{
		for(int iVertexCount = 1;iVertexCount < iVertexNum;iVertexCount++){
			meshDeformingModel->vList[iVertexCount].ptr[0] = spmtrxSclFreeResult.coeff(iVertexCount * 2,     0);
			meshDeformingModel->vList[iVertexCount].ptr[1] = spmtrxSclFreeResult.coeff(iVertexCount * 2 + 1, 0);
#ifdef GLOB_DEBUG
			//cout<<meshDeformingModel->vList[iVertexCount].ptr[0]<<"  "<<meshDeformingModel->vList[iVertexCount].ptr[1]<<"  "<<meshDeformingModel->vList[iVertexCount].ptr[2]<<endl;
#endif
		}
	}
	return true;
};

bool SparseARAPSolver::GetResult(double** dResultPos){
	if(!bIsInitialized) return false;
	if(bIsScaleVariant){
		for(int iVertexCount = 0;iVertexCount < iVertexNum;iVertexCount++){
			dResultPos[iVertexCount][0] = spmtrxSclVarResult.coeff(iVertexCount * 2,     0);
			dResultPos[iVertexCount][1] = spmtrxSclVarResult.coeff(iVertexCount * 2 + 1, 0);
		}
	}
	else{
		for(int iVertexCount = 0;iVertexCount < iVertexNum;iVertexCount++){
			dResultPos[iVertexCount][0] = spmtrxSclFreeResult.coeff(iVertexCount * 2,     0);
			dResultPos[iVertexCount][1] = spmtrxSclFreeResult.coeff(iVertexCount * 2 + 1, 0);
		}
	}
	return true;
};

bool SparseARAPSolver::GetResult(double** dResultPos, bool bScaleVariantSetting){
	if(!bIsInitialized) return false;
	if(bScaleVariantSetting){
		for(int iVertexCount = 0;iVertexCount < iVertexNum;iVertexCount++){
			dResultPos[iVertexCount][0] = spmtrxSclVarResult.coeff(iVertexCount * 2,     0);
			dResultPos[iVertexCount][1] = spmtrxSclVarResult.coeff(iVertexCount * 2 + 1, 0);
		}
	}
	else{
		for(int iVertexCount = 0;iVertexCount < iVertexNum;iVertexCount++){
			dResultPos[iVertexCount][0] = spmtrxSclFreeResult.coeff(iVertexCount * 2,     0);
			dResultPos[iVertexCount][1] = spmtrxSclFreeResult.coeff(iVertexCount * 2 + 1, 0);
		}
	}
	return true;
};

bool SparseARAPSolver::GetVertexPosition(double** dVertexPos){
	if(!bIsInitialized) return false;
	for(int iVertexCount = 0;iVertexCount < iVertexNum;iVertexCount++){
		dVertexPos[iVertexCount][0] = double(meshDeformingModel->vList[iVertexCount].ptr[0]);
		dVertexPos[iVertexCount][1] = double(meshDeformingModel->vList[iVertexCount].ptr[1]);
	}
	return true;
};

bool SparseARAPSolver::InitializeMatrices(){
	if(!AnalyzeEdgeRelation()) return false;

#ifdef GLOB_DEBUG
	//cout<<"iVertexNum:"<<iVertexNum<<endl;
	//cout<<"iEdgeNum  :"<<iEdgeNum<<endl;
#endif
	
	iConstraintNum = 0;

	bIsConstraint  = new bool[iVertexNum];
	for(int iTmp = 0;iTmp < iVertexNum;iTmp++)bIsConstraint[iTmp] = false; 

	dInitVertexPos = new double*[iVertexNum];
	for(int iVertexCount = 0;iVertexCount < iVertexNum;iVertexCount++){
		dInitVertexPos[iVertexCount] = new double[3];
		for(int iTmp = 0;iTmp < 3;iTmp++) dInitVertexPos[iVertexCount][iTmp] = meshDeformingModel->vList[iVertexCount].ptr[iTmp];
	}

	spmtrxRotation          = SparseMatrix<double, ColMajor>(iEdgeNum * 2, iVertexNum * 2);
	spmtrxRotationAngle     = SparseMatrix<double, ColMajor>(iEdgeNum * 2, 1);

	spmtrxSclFreeErrorCoef  = SparseMatrix<double, ColMajor>(iEdgeNum * 2 + iVertexNum * 2, 1);
	spmtrxSclFreeResult     = SparseMatrix<double, ColMajor>(iVertexNum * 2, 1);

	spmtrxSclFreeEdgeVector = SparseMatrix<double, ColMajor>(iEdgeNum * 2 + iVertexNum * 2, iVertexNum * 2);
	spmtrxSclFreeError      = SparseMatrix<double, ColMajor>(iEdgeNum * 2 + iVertexNum * 2, iVertexNum * 2);

	spmtrxSclVarErrorCoef   = SparseMatrix<double, ColMajor>(iEdgeNum * 2 + iVertexNum * 2, 1);
	spmtrxSclVarResult      = SparseMatrix<double, ColMajor>(iVertexNum * 2, 1);

	spmtrxSclVarError       = SparseMatrix<double, ColMajor>(iEdgeNum * 2 + iVertexNum * 2, iVertexNum * 2);
	spmtrxSclVarEdgeVector  = SparseMatrix<double, ColMajor>(iEdgeNum * 2 + iVertexNum * 2, iVertexNum * 2);

	if(!InitializeEdgeVector()) return false;
	
#ifdef GLOB_DEBUG
	//system("pause");
#endif

	spmtrxRotation.makeCompressed();

	spmtrxSclFreeErrorCoef.setZero();
	spmtrxSclFreeEdgeConstr = SparseMatrix<double, ColMajor>(spmtrxSclFreeEdgeVector);
	spqrSclFreeSolver  = SparseQR<SparseMatrix<double, ColMajor>, COLAMDOrdering<int>>(spmtrxSclFreeError);

	spmtrxSclVarErrorCoef.setZero();
	spmtrxSclVarEdgeConstr = SparseMatrix<double, ColMajor>(spmtrxSclVarEdgeVector);
	spqrSclVarSolver   = SparseQR<SparseMatrix<double, ColMajor>, COLAMDOrdering<int>>(spmtrxSclVarError);

#ifdef GLOB_DEBUG
	//cout<<spmtrxSclFreeError.topRows(iEdgeNum * 2)<<endl;
	//cout<<spmtrxSclVarError.topRows(iEdgeNum * 2)<<endl;
	//cout<<spmtrxRotation.topRows(iEdgeNum * 2)<<endl;
#endif

	return true;
};

bool SparseARAPSolver::TerminateMatrices(){
	if(bIsInitialized){
		iConstraintNum = 0;

		for(int iEdgeCount = 0;iEdgeCount < iEdgeNum;iEdgeCount++) delete iEdgeRelation[iEdgeCount];
		delete iEdgeRelation;
		iEdgeNum   = 0;
		iVertexNum = 0;

		delete bIsConstraint;

		for(int iVertexCount = 0;iVertexCount < iVertexNum;iVertexCount++) delete dInitVertexPos[iVertexCount];
		delete dInitVertexPos;

		meshDeformingModel = NULL;

		bIsInitialized = false;
	}
	return true;
};

bool SparseARAPSolver::AnalyzeEdgeRelation(){
	iVertexNum = int(meshDeformingModel->vTotal);

	bool** bEdgeMap = new bool*[iVertexNum];
	for(int iTmpA = 0;iTmpA < iVertexNum;iTmpA++){
		bEdgeMap[iTmpA] = new bool[iVertexNum];
		for(int iTmpB = 0;iTmpB < iVertexNum;iTmpB++) bEdgeMap[iTmpA][iTmpB] = false;
	}

	int iFaceNum = meshDeformingModel->fTotal;

	for(int iFaceCount = 0;iFaceCount < iFaceNum;iFaceCount++){
		int iFaceVertex[3];
		for(int iTmp = 0;iTmp < 3;iTmp++) iFaceVertex[iTmp] = meshDeformingModel->faceList[iFaceCount].v[iTmp].v;
		bEdgeMap[iFaceVertex[0]][iFaceVertex[1]] = true;
		bEdgeMap[iFaceVertex[1]][iFaceVertex[0]] = true;
		bEdgeMap[iFaceVertex[0]][iFaceVertex[2]] = true;
		bEdgeMap[iFaceVertex[2]][iFaceVertex[0]] = true;
		bEdgeMap[iFaceVertex[1]][iFaceVertex[2]] = true;
		bEdgeMap[iFaceVertex[2]][iFaceVertex[1]] = true;
	}

	iEdgeNum = 0;
	for(int iRowCount = 0;iRowCount < iVertexNum;iRowCount++){
		for(int iColCount = iRowCount + 1;iColCount < iVertexNum;iColCount++){
			if(bEdgeMap[iRowCount][iColCount]) iEdgeNum++;
		}
	}
	
	iEdgeRelation = new int*[iEdgeNum];
	for(int iEdgeCount = 0;iEdgeCount < iEdgeNum;iEdgeCount++){
		iEdgeRelation[iEdgeCount] = new int[4];
		for(int iTmp = 0;iTmp < 4;iTmp++) iEdgeRelation[iEdgeCount][iTmp] = -1;
	}

	{
		int iEdgeCount = 0;
		for(int iRowCount = 0;iRowCount < iVertexNum;iRowCount++){
			for(int iColCount = iRowCount + 1;iColCount < iVertexNum;iColCount++){
				if(bEdgeMap[iRowCount][iColCount]){
					iEdgeRelation[iEdgeCount][0] = iRowCount;
					iEdgeRelation[iEdgeCount][1] = iColCount;
					for(int iFaceCount = 0;iFaceCount < iFaceNum;iFaceCount++){
						int iFaceVertex[3];
						for(int iTmp = 0;iTmp < 3;iTmp++) iFaceVertex[iTmp] = meshDeformingModel->faceList[iFaceCount].v[iTmp].v;
						
						bool bIsMatchRow = false;
						for(int iTmp = 0;iTmp < 3;iTmp++) bIsMatchRow = (iRowCount == iFaceVertex[iTmp]) || bIsMatchRow;
						
						bool bIsMatchCol = false;
						for(int iTmp = 0;iTmp < 3;iTmp++) bIsMatchCol = (iColCount == iFaceVertex[iTmp]) || bIsMatchCol;
						
						if(bIsMatchRow && bIsMatchCol){
							switch(iEdgeRelation[iEdgeCount][2]){
							case(-1):
								iEdgeRelation[iEdgeCount][2] = iFaceVertex[0] + iFaceVertex[1] + iFaceVertex[2] - (iRowCount + iColCount);
								break;
							default:
								iEdgeRelation[iEdgeCount][3] = iFaceVertex[0] + iFaceVertex[1] + iFaceVertex[2] - (iRowCount + iColCount);
							}
						}
					}
					iEdgeCount++;
				}
			}
		}
	}
#ifdef GLOB_DEBUG
	//for(int iEdgeCount = 0;iEdgeCount < iEdgeNum;iEdgeCount++){
	//	for(int iTmp = 0;iTmp < 4;iTmp++) cout<<iEdgeRelation[iEdgeCount][iTmp]<<" ";
	//	cout<<endl;
	//}
#endif
	
	for(int iVertexCount = 0;iVertexCount < iVertexNum;iVertexCount++) delete bEdgeMap[iVertexCount];
	delete bEdgeMap;

	return true;
};

bool SparseARAPSolver::InitializeEdgeVector(){
	for(int iEdgeCount = 0;iEdgeCount < iEdgeNum;iEdgeCount++){
		
		Matrix<double, 8, 2>  mtrxG;
		mtrxG.setZero();
		for(int iTmp = 0;iTmp < 4 && iEdgeRelation[iEdgeCount][iTmp] > 0;iTmp++){
			mtrxG.coeffRef(2 * iTmp, 0)     =  dInitVertexPos[iEdgeRelation[iEdgeCount][iTmp]][0];
			mtrxG.coeffRef(2 * iTmp, 1)     =  dInitVertexPos[iEdgeRelation[iEdgeCount][iTmp]][1];
			mtrxG.coeffRef(2 * iTmp + 1, 0) =  dInitVertexPos[iEdgeRelation[iEdgeCount][iTmp]][1];
			mtrxG.coeffRef(2 * iTmp + 1, 1) = -dInitVertexPos[iEdgeRelation[iEdgeCount][iTmp]][0];
		}

		double dEdge[2];
		for(int iTmp = 0;iTmp < 2;iTmp++) dEdge[iTmp] = dInitVertexPos[iEdgeRelation[iEdgeCount][1]][iTmp] - dInitVertexPos[iEdgeRelation[iEdgeCount][0]][iTmp];

		Matrix<double, 2, 2> mtrxEdge;
		mtrxEdge.coeffRef(0, 0) =  dEdge[0];
		mtrxEdge.coeffRef(0, 1) =  dEdge[1];
		mtrxEdge.coeffRef(1, 0) =  dEdge[1];
		mtrxEdge.coeffRef(1, 1) = -dEdge[0];

		Matrix<double, 2, 8> mtrxPreCoef;
		mtrxPreCoef.setZero();
		mtrxPreCoef.coeffRef(0, 0) = -1;
		mtrxPreCoef.coeffRef(0, 2) =  1;
		mtrxPreCoef.coeffRef(1, 1) = -1;
		mtrxPreCoef.coeffRef(1, 3) =  1;

		Matrix<double, 2, 8> mtrxGt       = mtrxG.transpose();
		Matrix<double, 2, 2> mtrxGtG      = mtrxGt * mtrxG;
		Matrix<double, 2, 2> mtrxInvGtG   = mtrxGtG.inverse();
		Matrix<double, 2, 8> mtrxRot      = mtrxInvGtG * mtrxGt;
		Matrix<double, 2, 8> mtrxEdgeCoef = mtrxPreCoef - (mtrxEdge * mtrxRot);
		
#ifdef GLOB_DEBUG
		/*
		for(int iTmp = 0;iTmp < 4;iTmp++) cout<<iEdgeRelation[iEdgeCount][iTmp] - 1<<" ";
		cout<<endl;
		cout<<"mtrxG       "<<endl<<mtrxG<<endl;
		cout<<"mtrxGt      "<<endl<<mtrxGt<<endl;
		cout<<"mtrxGtG     "<<endl<<mtrxGtG<<endl;
		cout<<"mtrxInvGtG  "<<endl<<mtrxInvGtG<<endl;
		cout<<"mtrxRot     "<<endl<<mtrxRot<<endl;
		cout<<"mtrxEdge    "<<endl<<mtrxEdge<<endl;
		cout<<"mtrxEdgeCoef"<<endl<<mtrxEdgeCoef<<endl;
		cout<<"-----------------------------------"<<endl;
		system("pause");
		*/
#endif

		for(int iTmp = 0;iTmp < 4 && iEdgeRelation[iEdgeCount][iTmp] > 0;iTmp++){
			spmtrxRotation.coeffRef(iEdgeCount * 2    , 2 * iEdgeRelation[iEdgeCount][iTmp]    ) = mtrxRot.coeff(0, iTmp * 2);
			spmtrxRotation.coeffRef(iEdgeCount * 2 + 1, 2 * iEdgeRelation[iEdgeCount][iTmp]    ) = mtrxRot.coeff(1, iTmp * 2);
			spmtrxRotation.coeffRef(iEdgeCount * 2    , 2 * iEdgeRelation[iEdgeCount][iTmp] + 1) = mtrxRot.coeff(0, iTmp * 2 + 1);
			spmtrxRotation.coeffRef(iEdgeCount * 2 + 1, 2 * iEdgeRelation[iEdgeCount][iTmp] + 1) = mtrxRot.coeff(1, iTmp * 2 + 1);

			spmtrxSclFreeEdgeVector.coeffRef(iEdgeCount * 2    , 2 * iEdgeRelation[iEdgeCount][iTmp]    ) = mtrxEdgeCoef.coeff(0, iTmp * 2);
			spmtrxSclFreeEdgeVector.coeffRef(iEdgeCount * 2 + 1, 2 * iEdgeRelation[iEdgeCount][iTmp]    ) = mtrxEdgeCoef.coeff(1, iTmp * 2);
			spmtrxSclFreeEdgeVector.coeffRef(iEdgeCount * 2    , 2 * iEdgeRelation[iEdgeCount][iTmp] + 1) = mtrxEdgeCoef.coeff(0, iTmp * 2 + 1);
			spmtrxSclFreeEdgeVector.coeffRef(iEdgeCount * 2 + 1, 2 * iEdgeRelation[iEdgeCount][iTmp] + 1) = mtrxEdgeCoef.coeff(1, iTmp * 2 + 1);
		}

		spmtrxSclVarEdgeVector.coeffRef(iEdgeCount * 2    , 2 * iEdgeRelation[iEdgeCount][0]    ) = -1;
		spmtrxSclVarEdgeVector.coeffRef(iEdgeCount * 2    , 2 * iEdgeRelation[iEdgeCount][1]    ) =  1;
		spmtrxSclVarEdgeVector.coeffRef(iEdgeCount * 2 + 1, 2 * iEdgeRelation[iEdgeCount][0] + 1) = -1;
		spmtrxSclVarEdgeVector.coeffRef(iEdgeCount * 2 + 1, 2 * iEdgeRelation[iEdgeCount][1] + 1) =  1;
		
	}

#ifdef GLOB_DEBUG
	//cout<<spmtrxRotation.topRows(2 * iEdgeNum)<<endl;
	//cout<<spmtrxSclFreeEdgeVector.topRows(2 * iEdgeNum)<<endl;
	//cout<<spmtrxSclVarEdgeVector.topRows(2 * iEdgeNum)<<endl;
#endif
	return true;
};