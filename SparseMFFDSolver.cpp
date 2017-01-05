/*Other inclusion*/
#include "SparseMFFDSolver.h"
#include "Debug.h"

/*Global variable*/
#define globCONSTWEIGHT    100
#define globMAXITERATION   10
#define globERRORTHRESHOLD 0.001

#define globXAXIS 0
#define globYAXIS 1
#define globZAXIS 2

SparseMFFDSolver::SparseMFFDSolver(){
	bIsInitialized = false;

	iConstraintNum = 0;

	bIsConstraint  = NULL;
	dConstraintPos = NULL;

	iVertexNum    = 0;
	iEdgeNum	  = 0;
	iEdgeRelation = NULL;

	meshDeformingModel = NULL;
};

SparseMFFDSolver::~SparseMFFDSolver(){
	TerminateModelVariance();
};


bool SparseMFFDSolver::InitialMFFDMatrix(mesh* meshTargetModel, SparseARAPSolver* ARAPmtrxPtr){
	if(bIsInitialized) return false;
	
	ARAPmtrxSolver     = ARAPmtrxPtr;
	meshDeformingModel = meshTargetModel;

	if(!(bIsInitialized = InitializeModelVariance())) return false;

	return true;
};

bool SparseMFFDSolver::SetModel(mesh* meshTargetModel){
	if(!bIsInitialized)       return false;
	if(!TerminateModelVariance())  return false;

	meshDeformingModel = meshTargetModel;

	if(!(bIsInitialized = InitializeModelVariance())) return false;

	return true;
};

bool SparseMFFDSolver::SetConstraintList(bool* bConstraintID){
	if(!bIsInitialized)      return false;

	iConstraintNum = 0;
	for(int iVertexCount = 0;iVertexCount < iVertexNum;iVertexCount++){
		bIsConstraint[iVertexCount] = bConstraintID[iVertexCount];
		if(bIsConstraint[iVertexCount]) iConstraintNum++;
	}

	return true;
};

bool SparseMFFDSolver::UpdateConstraint(){
	if(!bIsInitialized)                                    return false;
	cout<<"Update Constraint..."<<endl;
	if(!ARAPmtrxSolver->GetVertexPosition(dConstraintPos)) return false;
	return true;
};

bool SparseMFFDSolver::ResetConstraint(){
	if(!bIsInitialized)      return false;
	return true;
};

bool SparseMFFDSolver::DeriveResult(){
	if(!bIsInitialized)      return false;
	if(iConstraintNum < 2)   return false;

	for(int iDeformLevel = 1; iDeformLevel <= globMAXITERATION && ErrorCalculate(iDeformLevel) > globERRORTHRESHOLD; iDeformLevel++){
		cout<<"Iteration #"<<iDeformLevel<<"==========================================================================="<<endl;
		if(!InitializeMatrices(iDeformLevel)){
			cout<<"Iteration Break"<<endl;
			break;
		}

		if(!ApplyDeform(iDeformLevel))        return false;
		if(!TerminateMatrices())              return false;
#ifdef GLOB_DEBUG
#endif
	}

	return true;
};

bool SparseMFFDSolver::DeriveResult(int iStartIter = 1, int EndIter = globMAXITERATION){
	if(!bIsInitialized)      return false;
	if(iConstraintNum < 2)   return false;
	if(iStartIter > EndIter) return false;

	for(int iDeformLevel = iStartIter; iDeformLevel <= EndIter && ErrorCalculate(iDeformLevel) > globERRORTHRESHOLD; iDeformLevel++){
		cout<<"Iteration #"<<iDeformLevel<<"==========================================================================="<<endl;
		if(!InitializeMatrices(iDeformLevel)){
			cout<<"Iteration Break"<<endl;
			break;
		}

		if(!ApplyDeform(iDeformLevel))        return false;
		if(!TerminateMatrices())              return false;
#ifdef GLOB_DEBUG
#endif
	}

	return true;
};

bool SparseMFFDSolver::ApplyDeform(int iDeformLevel){
	if(!bIsInitialized)      return false;

	int iLatticeSize = LatticeSize(iDeformLevel);

	double** dPosDiff = new double*[iVertexNum];
	for(int iVertexCount = 0;iVertexCount < iVertexNum;iVertexCount++){
		dPosDiff[iVertexCount] = new double[3];
		for(int iTmp = 0;iTmp < 3;iTmp++) dPosDiff[iVertexCount][iTmp] = 0;
	}

	for(int iVertexCount = 1;iVertexCount < iVertexNum;iVertexCount++){
		
#ifdef GLOB_DEBUG
		//cout<<"Vertex #"<<iVertexCount<<"  "<<meshDeformingModel->vList[iVertexCount].ptr[0]<<"  "<<meshDeformingModel->vList[iVertexCount].ptr[1]<<"  "<<meshDeformingModel->vList[iVertexCount].ptr[2]<<" ----> ";
#endif

		for(int iLatX = 0;iLatX < iLatticeSize;iLatX++){
			for(int iLatY = 0;iLatY < iLatticeSize;iLatY++){
				for(int iLatZ = 0;iLatZ < 2;iLatZ++){
					double dCoeff = BernPoly(dNormalizedPos[iVertexCount][0], iLatticeSize, iLatX) * BernPoly(dNormalizedPos[iVertexCount][1], iLatticeSize, iLatY) * BernPoly(dNormalizedPos[iVertexCount][2], 2, iLatZ);
					for(int iAxisCount = 0;iAxisCount < 3;iAxisCount++) dPosDiff[iVertexCount][iAxisCount] += dCoeff * spmtrxResult[iAxisCount].coeff(((iLatX * iLatticeSize) + iLatY) * 2 + iLatZ, 0);
				}
			}
		}
		for(int iTmp = 0;iTmp < 3;iTmp++) meshDeformingModel->vList[iVertexCount].ptr[iTmp] += dPosDiff[iVertexCount][iTmp] * AxisLength(iTmp);

#ifdef GLOB_DEBUG
		//cout<<meshDeformingModel->vList[iVertexCount].ptr[0]<<"  "<<meshDeformingModel->vList[iVertexCount].ptr[1]<<"  "<<meshDeformingModel->vList[iVertexCount].ptr[2]<<endl;
#endif
	}

	for(int iVertexCount = 0;iVertexCount < iVertexNum;iVertexCount++) delete dPosDiff[iVertexCount];
	delete dPosDiff;

	return true;
};

bool SparseMFFDSolver::InitializeModelVariance(){
	if(!AnalyzeEdgeRelation()) return false;

	TerminateMatrices();

	iConstraintNum = 0;

	bIsConstraint  = new bool[iVertexNum];
	for(int iVertexCount = 0;iVertexCount < iVertexNum;iVertexCount++) bIsConstraint[iVertexCount] = false;

	dConstraintPos = new double*[iVertexNum];
	for(int iVertexCount = 0;iVertexCount < iVertexNum;iVertexCount++){
		dConstraintPos[iVertexCount] = new double[2];
		dConstraintPos[iVertexCount][0] = dConstraintPos[iVertexCount][1] = 0.0; 
	}

	dNormalizedPos = new double*[iVertexNum];
	for(int iVertexCount = 0;iVertexCount < iVertexNum;iVertexCount++){
		dNormalizedPos[iVertexCount] = new double[3];
		dNormalizedPos[iVertexCount][0] = dNormalizedPos[iVertexCount][1] = dNormalizedPos[iVertexCount][2] = 0.0;
	}

	for(int iTmp = 0;iTmp < 3;iTmp++){
		dAxisRange[iTmp][0] = -9999999999;
		dAxisRange[iTmp][1] =  9999999999;
	}
	dMaxAxisLength = 0;

	for(int iTmp = 0;iTmp < 3;iTmp++) spmtrxResult[iTmp] = SparseMatrix<double, ColMajor>(2, 1);
	
	if(!SimpleContourDetect()) return false;

	return true;
};

bool SparseMFFDSolver::TerminateModelVariance(){
	if(bIsInitialized){
		iConstraintNum = 0;

		for(int iEdgeCount = 0;iEdgeCount< iEdgeNum;iEdgeCount++) delete iEdgeRelation[iEdgeCount];
		delete iEdgeRelation;
		iEdgeNum   = 0;
		iVertexNum = 0;

		delete bIsConstraint;

		for(int iVertexCount = 0;iVertexCount < iVertexNum;iVertexCount++) delete dConstraintPos[iVertexCount];
		delete dConstraintPos;
		
		for(int iVertexCount = 0;iVertexCount < iVertexNum;iVertexCount++) delete dNormalizedPos[iVertexCount];
		delete dNormalizedPos;

		for(int iTmp = 0;iTmp < 3;iTmp++){
			dAxisRange[iTmp][0] = -9999999999;
			dAxisRange[iTmp][1] =  9999999999;
		}
		dMaxAxisLength = 0;

		meshDeformingModel = NULL;
		
		for(int iTmp = 0;iTmp < 3;iTmp++) spmtrxResult[iTmp].~SparseMatrix();

		bIsInitialized = false;
	}
	return true;
};

bool SparseMFFDSolver::InitializeMatrices(int iDeformLevel){
	int iLatticeSize = LatticeSize(iDeformLevel);
	int iResultSize  = iLatticeSize * iLatticeSize * 2;
	int iErrorSize   = iConstraintNum + iLatticeSize * iLatticeSize * 3 + iEdgeNum;

	if(!ModelNormalize())        return false;
	if(iResultSize > iErrorSize) return false;

	int iCurrentTime[2];
	int iTimeTmp[2];

	iTimeTmp[0] = glutGet(GLUT_ELAPSED_TIME);

	SparseMatrix<double, ColMajor> spmtrxError = SparseMatrix<double, ColMajor>(iErrorSize, iResultSize);
	SparseMatrix<double, ColMajor> spmtrxConstraintError = SparseMatrix<double, ColMajor>(iConstraintNum, iResultSize);
	SparseMatrix<double, ColMajor> spmtrxErrorCoef[3];
	for(int iTmp = 0;iTmp < 3;iTmp++) spmtrxErrorCoef[iTmp] = SparseMatrix<double, ColMajor>(iErrorSize, 1);
	
	for(int iTmp = 0;iTmp < 3;iTmp++){
		spmtrxResult[iTmp].resize(iResultSize, 1);
		spmtrxResult[iTmp].reserve(iResultSize);
		spmtrxResult[iTmp].setZero();
	}
	
	{
		int iConstraintCount = 0;
		for(int iVertexCount = 1;iVertexCount < iVertexNum;iVertexCount++){
			if(bIsConstraint[iVertexCount]){
				for(int iLatX = 0;iLatX < iLatticeSize;iLatX++){
					for(int iLatY = 0;iLatY < iLatticeSize;iLatY++){
						for(int iLatZ = 0;iLatZ < 2;iLatZ++){
							double dCoeff = BernPoly(dNormalizedPos[iVertexCount][0], iLatticeSize, iLatX) * BernPoly(dNormalizedPos[iVertexCount][1], iLatticeSize, iLatY) * BernPoly(dNormalizedPos[iVertexCount][2], 2, iLatZ);
							//cout<<iLatX<<" "<<iLatY<<" "<<iLatZ<<" ("<<iLatticeSize<<") "<<dNormalizedPos[iVertexCount][0]<<" "<<dNormalizedPos[iVertexCount][1]<<" "<<dNormalizedPos[iVertexCount][2]<<" -> "<<BernPoly(dNormalizedPos[iVertexCount][0], iLatticeSize, iLatX)<<"*"<<BernPoly(dNormalizedPos[iVertexCount][1], iLatticeSize, iLatY)<<"*"<<BernPoly(dNormalizedPos[iVertexCount][2], 2, iLatZ)<<"="<<dCoeff<<endl;
							spmtrxError.coeffRef(iConstraintCount, ((iLatX * iLatticeSize) + iLatY) * 2 + iLatZ) = globCONSTWEIGHT * dCoeff;
							spmtrxConstraintError.coeffRef(iConstraintCount, ((iLatX * iLatticeSize) + iLatY) * 2 + iLatZ) = globCONSTWEIGHT * dCoeff;
						}
					}
				}
				spmtrxErrorCoef[0].coeffRef(iConstraintCount, 0) = globCONSTWEIGHT * (dConstraintPos[iVertexCount][0] - double(meshDeformingModel->vList[iVertexCount].ptr[0])) / AxisLength(globXAXIS);
				spmtrxErrorCoef[1].coeffRef(iConstraintCount, 0) = globCONSTWEIGHT * (dConstraintPos[iVertexCount][1] - double(meshDeformingModel->vList[iVertexCount].ptr[1])) / AxisLength(globYAXIS);
				spmtrxErrorCoef[2].coeffRef(iConstraintCount, 0) = globCONSTWEIGHT * 0;
				iConstraintCount++;
				//cout<<"Vertex #"<<iVertexCount<<" ("<<double(meshDeformingModel->vList[iVertexCount].ptr[0])<<" "<<double(meshDeformingModel->vList[iVertexCount].ptr[1])<<") - ("<<dConstraintPos[iVertexCount][0]<<" "<<dConstraintPos[iVertexCount][1]<<"}"<<endl;
			}
		}
	}
	for(int iLatticeCount = 0;iLatticeCount < iLatticeSize * iLatticeSize * 2;iLatticeCount++){
		spmtrxError.coeffRef(iConstraintNum + iLatticeCount, iLatticeCount) = 1;
	}
	for(int iLatticeCount = 0;iLatticeCount < iLatticeSize * iLatticeSize;iLatticeCount++){
		spmtrxError.coeffRef(iConstraintNum + iLatticeSize * iLatticeSize * 2 + iLatticeCount, iLatticeCount * 2)     =  1;
		spmtrxError.coeffRef(iConstraintNum + iLatticeSize * iLatticeSize * 2 + iLatticeCount, iLatticeCount * 2 + 1) = -1;
	}
	for(int iEdgeCount = 0;iEdgeCount < iEdgeNum;iEdgeCount++){
		for(int iLatX = 0;iLatX < iLatticeSize;iLatX++){
			for(int iLatY = 0;iLatY < iLatticeSize;iLatY++){
				for(int iLatZ = 0;iLatZ < 2;iLatZ++){
					double dCoeffA = BernPoly(dNormalizedPos[iEdgeRelation[iEdgeCount][0]][0], iLatticeSize, iLatX) * BernPoly(dNormalizedPos[iEdgeRelation[iEdgeCount][0]][1], iLatticeSize, iLatY) * BernPoly(dNormalizedPos[iEdgeRelation[iEdgeCount][0]][2], 2, iLatZ);
					double dCoeffB = BernPoly(dNormalizedPos[iEdgeRelation[iEdgeCount][1]][0], iLatticeSize, iLatX) * BernPoly(dNormalizedPos[iEdgeRelation[iEdgeCount][1]][1], iLatticeSize, iLatY) * BernPoly(dNormalizedPos[iEdgeRelation[iEdgeCount][1]][2], 2, iLatZ);
					//cout<<iLatX<<" "<<iLatY<<" "<<iLatZ<<" "<<"-"<<iLatticeSize<<"-"<<dNormalizedPos[iEdgeRelation[iEdgeCount][0]][0]<<" "<<dNormalizedPos[iEdgeRelation[iEdgeCount][0]][1]<<" "<<dNormalizedPos[iEdgeRelation[iEdgeCount][0]][2]<<" -> "<<dCoeffA<<" "<<dCoeffB<<endl;
					spmtrxError.coeffRef(iConstraintNum + iLatticeSize * iLatticeSize * 3 + iEdgeCount, ((iLatX * iLatticeSize) + iLatY) * 2 + iLatZ) = dCoeffB - dCoeffA;
				}
			}
		}
	}
	spmtrxError.makeCompressed();
	for(int iTmp = 0;iTmp < 3;iTmp++){
		spmtrxErrorCoef[iTmp].makeCompressed();
		spmtrxResult[iTmp].setZero();
	}
	
	iTimeTmp[1] = glutGet(GLUT_ELAPSED_TIME);
	iCurrentTime[0] = iTimeTmp[1] - iTimeTmp[0];

#ifdef GLOB_DEBUG
	//cout<<spmtrxError.topRows(iErrorSize)<<endl;
	//cout<<spmtrxErrorCoef[0].topRows(iErrorSize)<<endl;
	//cout<<spmtrxErrorCoef[1].topRows(iErrorSize)<<endl;
	//cout<<spmtrxErrorCoef[2].topRows(iErrorSize)<<endl;
#endif
	
	iTimeTmp[0] = glutGet(GLUT_ELAPSED_TIME);

	SparseQR<SparseMatrix<double, ColMajor>, COLAMDOrdering<int>> spqrSolver = SparseQR<SparseMatrix<double, ColMajor>, COLAMDOrdering<int>>(spmtrxError);
	//SparseQR<SparseMatrix<double, ColMajor>, NaturalOrdering<int>> spqrSolver = SparseQR<SparseMatrix<double, ColMajor>, NaturalOrdering<int>>(spmtrxError);
	//spqrSolver.analyzePattern(spmtrxError);
	//spqrSolver.factorize(spmtrxError);
	spqrSolver.compute(spmtrxError);
	for(int iTmp = 0;iTmp < 3;iTmp++) spmtrxResult[iTmp] = spqrSolver.solve(spmtrxErrorCoef[iTmp]).topRows(iResultSize);
	
	iTimeTmp[1] = glutGet(GLUT_ELAPSED_TIME);
	iCurrentTime[1] = iTimeTmp[1] - iTimeTmp[0];

#ifdef GLOB_DEBUG
	/*
	cout<<"Result #"<<iDeformLevel<<endl;
	cout<<spmtrxResult[0].topRows(iResultSize)<<endl;
	cout<<spmtrxResult[1].topRows(iResultSize)<<endl;
	cout<<spmtrxResult[2].topRows(iResultSize)<<endl;
	SparseMatrix<double, ColMajor> spmtrxResultX = spmtrxResult[0].topRows(iResultSize);
	SparseMatrix<double, ColMajor> spmtrxResultY = spmtrxResult[1].topRows(iResultSize);
	SparseMatrix<double, ColMajor> spmtrxConstraintResultX = spmtrxConstraintError * spmtrxResultX;
	SparseMatrix<double, ColMajor> spmtrxConstraintResultY = spmtrxConstraintError * spmtrxResultY;
	cout<<"-----------------------------------------------------"<<endl;
	cout<<spmtrxConstraintResultX.topRows(iConstraintNum)<<endl;
	cout<<spmtrxErrorCoef[0].topRows(iConstraintNum)<<endl;
	cout<<"-----------------------------------------------------"<<endl;
	cout<<spmtrxConstraintResultY.topRows(iConstraintNum)<<endl;
	cout<<spmtrxErrorCoef[1].topRows(iConstraintNum)<<endl;
	cout<<"-----------------------------------------------------"<<endl;
	*/
	cout<<"Matrix Size :"<<iErrorSize<<" * "<<iResultSize<<endl;
	cout<<"Execution Time(Construct Matrix) : "<<iCurrentTime[0]<<" ms"<<endl;
	cout<<"Execution Time(Solve Matrix)     : "<<iCurrentTime[1]<<" ms"<<endl;
	cout<<"Construct Time : Solve Time = "<<double(iCurrentTime[0]) / double(iCurrentTime[1])<<" : 1"<<endl;

#endif

	return true;
};

bool SparseMFFDSolver::TerminateMatrices(){
	
	for(int iTmp = 0;iTmp < 3;iTmp++){
		dAxisRange[iTmp][0] = -9999999999;
		dAxisRange[iTmp][1] =  9999999999;
	}
	dMaxAxisLength = 0;

	return true;
};

bool SparseMFFDSolver::AnalyzeEdgeRelation(){
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
/*
bool SparseMFFDSolver::SimpleContourDetect(){
	bIsConstraint[6]   = true;
	bIsConstraint[7]   = true;
	bIsConstraint[9]   = true;
	bIsConstraint[13]  = true;
	bIsConstraint[28]  = true;
	bIsConstraint[42]  = true;
	bIsConstraint[44]  = true;
	bIsConstraint[47]  = true;
	bIsConstraint[51]  = true;
	bIsConstraint[74]  = true;
	bIsConstraint[78]  = true;
	bIsConstraint[91]  = true;
	bIsConstraint[92]  = true;
	bIsConstraint[93]  = true;
	bIsConstraint[95]  = true;
	bIsConstraint[96]  = true;
	bIsConstraint[99]  = true;
	bIsConstraint[100] = true;
	bIsConstraint[101] = true;
	bIsConstraint[112] = true;
	iConstraintNum = 20;
	return true;
};
*/

bool SparseMFFDSolver::SimpleContourDetect(){
	iConstraintNum = 0;
	for(int iEdgeCount = 0;iEdgeCount < iEdgeNum;iEdgeCount++){
		if(iEdgeRelation[iEdgeCount][3] < 0){
			bIsConstraint[iEdgeRelation[iEdgeCount][0]] = true;
			bIsConstraint[iEdgeRelation[iEdgeCount][1]] = true;
		}
		else{
			double dEdgeVec[3][3];
			double dEyeDir[3] = {0, 0, 1};
			for(int iTmp = 0;iTmp < 3;iTmp++){
				for(int iAxis = 0;iAxis < 3;iAxis++){
					dEdgeVec[iTmp][iAxis] = meshDeformingModel->vList[iEdgeRelation[iEdgeCount][0]].ptr[iAxis] - meshDeformingModel->vList[iEdgeRelation[iEdgeCount][iTmp + 1]].ptr[iAxis];
				}
			}
			Vector3d V3dEdge[3], V3dFaceNormal[2];
			for(int iTmp = 0;iTmp < 3;iTmp++) V3dEdge[iTmp] = Vector3d(dEdgeVec[iTmp]);
			Vector3d V3EyeDir = Vector3d(dEyeDir);
			V3dFaceNormal[0] = V3dEdge[0].cross(V3dEdge[1]);
			V3dFaceNormal[1] = V3dEdge[2].cross(V3dEdge[0]);
			//cout<<iEdgeCount<<" :"<<endl<<V3dFaceNormal[0].transpose()<<endl<<V3dFaceNormal[1].transpose()<<endl;
			//cout<<V3EyeDir.dot(V3dFaceNormal[0])<<endl<<V3EyeDir.dot(V3dFaceNormal[1])<<endl;
			if(V3EyeDir.dot(V3dFaceNormal[0]) * V3EyeDir.dot(V3dFaceNormal[1]) <= 0){
				bIsConstraint[iEdgeRelation[iEdgeCount][0]] = true;
				bIsConstraint[iEdgeRelation[iEdgeCount][1]] = true;
			}
		}
	}
#ifdef GLOB_DEBUG
	for(int iVertexCount = 0;iVertexCount < iVertexNum;iVertexCount++){
		if(bIsConstraint[iVertexCount]){
			iConstraintNum++;
			cout<<iVertexCount<<" ";
		}
	}
	cout<<endl;
#endif

	return true;
};

bool SparseMFFDSolver::ModelNormalize(){
	for(int iVertexCount = 1;iVertexCount < iVertexNum;iVertexCount++){
		for(int iAxis = 0;iAxis < 3;iAxis++){
			if(meshDeformingModel->vList[iVertexCount].ptr[iAxis] > dAxisRange[iAxis][0]) dAxisRange[iAxis][0] = meshDeformingModel->vList[iVertexCount].ptr[iAxis];
			if(meshDeformingModel->vList[iVertexCount].ptr[iAxis] < dAxisRange[iAxis][1]) dAxisRange[iAxis][1] = meshDeformingModel->vList[iVertexCount].ptr[iAxis];
		}
	}

	dMaxAxisLength = 0.0001;
	for(int iAxis = 0;iAxis < 3;iAxis++){
		dAxisRange[iAxis][0] += 0.001;
		dAxisRange[iAxis][1] -= 0.001;
		if(dMaxAxisLength < (dAxisRange[iAxis][0] - dAxisRange[iAxis][1])) dMaxAxisLength = dAxisRange[iAxis][0] - dAxisRange[iAxis][1];
	}

#ifdef GLOB_DEBUG
	cout<<"dMaxAxisLength = "<<dMaxAxisLength<<endl;
	cout<<"dAxisRangeX = "<<dAxisRange[globXAXIS][0]<<"   "<<dAxisRange[globXAXIS][1]<<endl;
	cout<<"dAxisRangeY = "<<dAxisRange[globYAXIS][0]<<"   "<<dAxisRange[globYAXIS][1]<<endl;
	cout<<"dAxisRangeZ = "<<dAxisRange[globZAXIS][0]<<"   "<<dAxisRange[globZAXIS][1]<<endl;
#endif

	for(int iVertexCount = 0;iVertexCount < iVertexNum;iVertexCount++){
		for(int iAxis = 0;iAxis < 3;iAxis++){
			dNormalizedPos[iVertexCount][iAxis] = (double(meshDeformingModel->vList[iVertexCount].ptr[iAxis]) - dAxisRange[iAxis][1]) / AxisLength(iAxis);
		}
	}
	return true;
}

double SparseMFFDSolver::ErrorCalculate(int iDeformLevel){
	double dError = 99999999;
	return dError;
};

double SparseMFFDSolver::BernPoly(double dVar, int iDegree, int iLevel){
	double dResult = 1.0;

	for(int iTmp = iLevel;iTmp < iDegree - 1;iTmp++) dResult *= double(iTmp + 1) / double(iDegree - iTmp - 1);
	for(int iTmp = 0     ;iTmp < iDegree - 1;iTmp++) dResult *= (iTmp < iLevel) ? dVar : double(1) - dVar;

	return dResult;
};

inline int SparseMFFDSolver::LatticeSize(int iDeformLevel){
	return (2 * iDeformLevel + 1);
};

inline double SparseMFFDSolver::AxisLength(int iAxis){
	return dAxisRange[iAxis][0] - dAxisRange[iAxis][1];
	//return dMaxAxisLength;
};