#include "GL_ARAPControl.h"

/*Inclusion of C/C++ standard library*/
#include <cmath>
#include <ctime>
#include <iostream>
using namespace std;

/*Inclusion of OpenGL*/
#include "glew.h"
#include "glut.h"

/*Other inclusion*/
#include "mesh.h"
#include "ShaderLoader.h"
#include "ErrorOutput.h"

/*Global variable of window*/
static ARAPGLControl* CurrentState;

ARAPGLControl::ARAPGLControl(){
	CurrentState = this;

	bIsScaleVariant = true;

	ARAPmtrxSolver   = NULL;
	meshDisplayModel = NULL;

	iCurrentSelectedID = -1;
	iCurrentNearID = -1;
	GLfModelPos[0] =  0.0f;
	GLfModelPos[1] =  0.0f;
	GLfModelPos[2] =  0.0f;
	GLfModelDefaultDir[0] = GLfModelDir[0] =  0.0f;
	GLfModelDefaultDir[1] = GLfModelDir[1] =  0.0f;
	GLfModelDefaultDir[2] = GLfModelDir[2] = -1.0f;

	iWindowCurrent = 0;
	iWindowSize[0] = 600;
	iWindowSize[1] = 800;
	
	GLfCameraPos[0] = 0.0f;
	GLfCameraPos[1] = 0.0f;
	GLfCameraPos[2] = -0.5f;
	GLfCameraDir[0] = 0.0f;
	GLfCameraDir[1] = 0.0f;
	GLfCameraDir[2] = 1.0f;
	GLfCameraUp[0] = 1.0f;
	GLfCameraUp[1] = 0.0f;
	GLfCameraUp[2] = 0.0f;
	GLfCameraRate = 10.0f;

	iMouseButton = GLUT_LEFT_BUTTON;
	iMouseState = GLUT_UP;
	iMouseClickPos[0] = 0;
	iMouseClickPos[1] = 0;
	
	bIsEdgeDisplayed = true;
	bIsVertDisplayed = true;
};

ARAPGLControl::~ARAPGLControl(){
};

bool ARAPGLControl::SetModel(mesh* meshModel){
	glutSetWindow(CurrentState->iWindowCurrent);
	TerminateVAO();
	meshDisplayModel = meshModel;
	InitialVAO();
	return true;
};

bool ARAPGLControl::InitialGLWindow(int iParent, mesh* meshModel, SparseARAPSolver* ARAPmtrxPtr){
	ARAPmtrxSolver   = ARAPmtrxPtr;
	meshDisplayModel = meshModel;

	if(!AnalyzeEdgeRelation()) return false;

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	iWindowCurrent = glutCreateSubWindow(iParent, 0, 0, iWindowSize[0], iWindowSize[1]);
	glutSetWindow(CurrentState->iWindowCurrent);
	InitialShader();
	InitialVAO();
	CameraSet();

	glutReshapeFunc(NULL);
	glutMouseFunc(&ARAPGLControl::ARAPMouse);
	glutMotionFunc(&ARAPGLControl::ARAPMotion);
	glutDisplayFunc(&ARAPGLControl::ARAPDisplay);
 	glutKeyboardFunc(&ARAPGLControl::ARAPKeyboard);
	return true;
};

void ARAPGLControl::InitialVAO(){
	glutSetWindow(CurrentState->iWindowCurrent);

	glGenVertexArrays(1, &uiVAO);
	glBindVertexArray(uiVAO);
	
	glGenBuffers(1, &uiVBO);
	glBindBuffer(GL_ARRAY_BUFFER, uiVBO);

	int iVertexNum  = int(meshDisplayModel->vTotal);
	int iNormalNum  = int(meshDisplayModel->nTotal);
	int iTextureNum = int(meshDisplayModel->tTotal);
	int iFaceNum    = int(meshDisplayModel->fTotal);
	int iBufferSize = iVertexNum + iNormalNum + iTextureNum;

	glBufferData(GL_ARRAY_BUFFER, (4 * iBufferSize * sizeof(GLfloat)), NULL, GL_STREAM_DRAW);
	
	Model2VAO();
	
	unsigned int* uiIndex = (unsigned int*)malloc((3 * iFaceNum + 2 * iEdgeNum + iVertexNum - 1) * sizeof(unsigned int));
	for(int iFaceCount = 0;iFaceCount < iFaceNum;iFaceCount++){
		for(int iTmp = 0;iTmp < 3;iTmp++){
			uiIndex[3 * iFaceCount + iTmp] = meshDisplayModel->faceList[iFaceCount].v[iTmp].v;
		}
	}
	for(int iEdgeCount = 0;iEdgeCount < iEdgeNum;iEdgeCount++){
		for(int iTmp = 0;iTmp < 2;iTmp++){
			uiIndex[3 * iFaceNum + 2 * iEdgeCount + iTmp] = iEdgeRelation[iEdgeCount][iTmp];
		}
	}
	for(int iVertexCount = 1;iVertexCount < iVertexNum;iVertexCount++){
		uiIndex[3 * iFaceNum + 2 * iEdgeNum + iVertexCount - 1] = iVertexCount;
	}

	glGenBuffers(1, &uiIBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, uiIBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, (3 * iFaceNum + 2 * iEdgeNum + iVertexNum - 1) * sizeof(unsigned int), NULL, GL_STATIC_DRAW);
	glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, (3 * iFaceNum + 2 * iEdgeNum + iVertexNum - 1) * sizeof(unsigned int), uiIndex);
    
	glBindVertexArray(0);
};

bool ARAPGLControl::InitialShader(){
	glutSetWindow(CurrentState->iWindowCurrent);

	GLArbFaceShader = glCreateProgramObjectARB();
	if(GLArbFaceShader == 0) return false;

	ShaderLoad(GLArbFaceShader, "ARAPFaceDisplayVert.txt", GL_VERTEX_SHADER_ARB);
	ShaderLoad(GLArbFaceShader, "ARAPFaceDisplayFrag.txt", GL_FRAGMENT_SHADER_ARB);
	
	GLArbEdgeShader = glCreateProgramObjectARB();
	if(GLArbEdgeShader == 0) return false;

	ShaderLoad(GLArbEdgeShader, "ARAPEdgeDisplayVert.txt", GL_VERTEX_SHADER_ARB);
	ShaderLoad(GLArbEdgeShader, "ARAPEdgeDisplayFrag.txt", GL_FRAGMENT_SHADER_ARB);

	GLArbVertexShader = glCreateProgramObjectARB();
	if(GLArbVertexShader == 0) return false;

	ShaderLoad(GLArbVertexShader, "ARAPVertexDisplayVert.txt", GL_VERTEX_SHADER_ARB);
	ShaderLoad(GLArbVertexShader, "ARAPVertexDisplayFrag.txt", GL_FRAGMENT_SHADER_ARB);

	GLArbVertexSelectShader = glCreateProgramObjectARB();
	if(GLArbVertexSelectShader == 0) return false;

	ShaderLoad(GLArbVertexSelectShader, "ARAPVertexSelectDisplayVert.txt", GL_VERTEX_SHADER_ARB);
	ShaderLoad(GLArbVertexSelectShader, "ARAPVertexSelectDisplayFrag.txt", GL_FRAGMENT_SHADER_ARB);
	return true;
};

void ARAPGLControl::TerminateVAO(){
	glutSetWindow(CurrentState->iWindowCurrent);

	glBindVertexArray(uiVAO);
	
	glDeleteBuffers(1, &uiVBO);
	glDeleteBuffers(1, &uiIBO);
	
	glBindVertexArray(0);
	
	glDeleteVertexArrays(1, &uiVAO);
};

void ARAPGLControl::Model2VAO(){
	glutSetWindow(CurrentState->iWindowCurrent);

	int iVertexNum  = int(meshDisplayModel->vTotal);
	int iNormalNum  = int(meshDisplayModel->nTotal);
	int iTextureNum = int(meshDisplayModel->tTotal);

	GLfloat* GLfVertex   = (GLfloat*)malloc(4 * iVertexNum  * sizeof(GLfloat));
	GLfloat* GLfNormal   = (GLfloat*)malloc(4 * iNormalNum  * sizeof(GLfloat));
	GLfloat* GLfTexCoord = (GLfloat*)malloc(4 * iTextureNum * sizeof(GLfloat));
	
	for(int iVertexCount = 0;iVertexCount < iVertexNum;iVertexCount++){
		for(int iTemp = 0;iTemp < 3;iTemp++){
			GLfVertex[iVertexCount * 4 + iTemp] = meshDisplayModel->vList[iVertexCount].ptr[iTemp];
			GLfVertex[iVertexCount * 4 + 3] = 1.0f;
		}
	}
	for(int iNorCount = 0;iNorCount < iNormalNum;iNorCount++){
		for(int iTemp = 0;iTemp < 3;iTemp++){
			GLfNormal[iNorCount * 4 + iTemp] = meshDisplayModel->nList[iNorCount].ptr[iTemp];
			GLfNormal[iNorCount * 4 + 3] = 0.0f;
		}
	}
	for(int iTexCount = 0;iTexCount < iTextureNum;iTexCount++){
		for(int iTemp = 0;iTemp < 3;iTemp++){
			GLfTexCoord[iTexCount * 4 + iTemp] = meshDisplayModel->tList[iTexCount].ptr[iTemp];
			GLfTexCoord[iTexCount * 4 + 3] = 1.0f;
		}
	}
	
	glBufferSubData(GL_ARRAY_BUFFER, 0                                              , 4 * iVertexNum  * sizeof(GLfloat), GLfVertex);
	glBufferSubData(GL_ARRAY_BUFFER, 4 * iVertexNum                * sizeof(GLfloat), 4 * iNormalNum  * sizeof(GLfloat), GLfNormal);
	glBufferSubData(GL_ARRAY_BUFFER, 4 * (iVertexNum + iNormalNum) * sizeof(GLfloat), 4 * iTextureNum * sizeof(GLfloat), GLfTexCoord);
	
	unsigned int iPositionAttrLoc = glGetAttribLocation(GLArbFaceShader, "v_V4Position");
	glEnableVertexAttribArray(iPositionAttrLoc);
	glVertexAttribPointer(iPositionAttrLoc, 4, GL_FLOAT, GL_FALSE, 0, 0);

	unsigned int iNormalAttrLoc   = glGetAttribLocation(GLArbFaceShader, "v_V4Normal");
	glEnableVertexAttribArray(iNormalAttrLoc);
	glVertexAttribPointer(iNormalAttrLoc,   4, GL_FLOAT, GL_FALSE, 0, reinterpret_cast<void*>(4 * iVertexNum * sizeof(GLfloat)));

	unsigned int iTexCoordAttrLoc = glGetAttribLocation(GLArbFaceShader, "v_V4TexCoord");
	glEnableVertexAttribArray(iTexCoordAttrLoc);
	glVertexAttribPointer(iTexCoordAttrLoc, 4, GL_FLOAT, GL_FALSE, 0, reinterpret_cast<void*>(4 * (iVertexNum + iNormalNum) * sizeof(GLfloat)));

	delete GLfVertex;
	delete GLfNormal;
	delete GLfTexCoord;
};

int ARAPGLControl::VertexSelect(int iPosX, int iPosY){
	glutSetWindow(CurrentState->iWindowCurrent);
	
	double dMousePos[2];
	Mouse2SpacePos(iPosX, iPosY, dMousePos);

	static int iSelectRange = 10;

	double dPixelDis = 0.5 / double(iWindowSize[0]) * double(GLfCameraRate);
	double dMinDis[2] = {dPixelDis * (iSelectRange * 2 - 1), dPixelDis * (iSelectRange * 2 - 1)};
	int iMinDisVertexID = -1;

	for(int iVertexCount = 0;iVertexCount < int(meshDisplayModel->vTotal);iVertexCount++){
		double dVertexPos[2] = {double(meshDisplayModel->vList[iVertexCount].ptr[0]), double(meshDisplayModel->vList[iVertexCount].ptr[1])};
		double dVertexDis[2];
		for(int i = 0;i < 2;i++){
			dVertexDis[i] = abs(dVertexPos[i] - dMousePos[i]);
		}
		if(dVertexDis[0] < dMinDis[0] && dVertexDis[1] < dMinDis[1]){
			for(int i = 0;i < 2;i++){
				dMinDis[i] = dVertexDis[i];
				iMinDisVertexID = iVertexCount;
			}
		}
	}
	cout<<"("<<dMousePos[0]<<" "<<dMousePos[1]<<")"<<iMinDisVertexID<<endl;;
	return iMinDisVertexID;
};

void ARAPGLControl::Mouse2SpacePos(int iPosX, int iPosY, double* dSpacePos){
	dSpacePos[0] = double(iPosX);
	dSpacePos[1] = iWindowSize[1] - double(iPosY);

	for(int i = 0;i < 2;i++){
		dSpacePos[i] -= double(iWindowSize[i]) / 2;
		dSpacePos[i] /= double(iWindowSize[0]);
		dSpacePos[i] *= double(GLfCameraRate) * 2;
	}
};

void ARAPGLControl::CameraSet(){
	glutSetWindow(CurrentState->iWindowCurrent);

	//Simple Orthogonal Projection
	GLfloat GLfCameraMat[16];
	for(int i = 0;i < 16;i++){
		GLfCameraMat[i] = 0.0f;
	}
	GLfCameraMat[0]  = 1.0f / GLfCameraRate;
	GLfCameraMat[5]  = 1.0f / GLfCameraRate * iWindowSize[0] / iWindowSize[1];
	GLfCameraMat[10] = 1.0f / GLfCameraRate;

	GLfCameraMat[12]  = -GLfCameraPos[0];
	GLfCameraMat[13]  = -GLfCameraPos[1];
	GLfCameraMat[14] = -GLfCameraPos[2];
	GLfCameraMat[15] = 1.0f;

	GLfloat GLfModelMat[16];
	for(int i = 0;i < 16;i++){
		GLfModelMat[i] = 0.0f;
	}
	GLfModelMat[0]  = 1.0f;
	GLfModelMat[5]  = 1.0f;
	GLfModelMat[10] = 1.0f;
	
	GLfModelMat[3]  = GLfModelPos[0];
	GLfModelMat[7]  = GLfModelPos[1];
	GLfModelMat[11] = GLfModelPos[2];
	GLfModelMat[15] = 1.0f;

	glProgramUniformMatrix4fv(GLArbFaceShader, glGetUniformLocation(GLArbFaceShader, "v_MatCamera"), 1, GL_FALSE, GLfCameraMat);
	glProgramUniformMatrix4fv(GLArbFaceShader, glGetUniformLocation(GLArbFaceShader, "v_MatModel"),  1, GL_FALSE, GLfModelMat);
	glProgramUniformMatrix4fv(GLArbEdgeShader, glGetUniformLocation(GLArbEdgeShader, "v_MatCamera"), 1, GL_FALSE, GLfCameraMat);
	glProgramUniformMatrix4fv(GLArbEdgeShader, glGetUniformLocation(GLArbEdgeShader, "v_MatModel"),  1, GL_FALSE, GLfModelMat);
	glProgramUniformMatrix4fv(GLArbVertexShader, glGetUniformLocation(GLArbVertexShader, "v_MatCamera"), 1, GL_FALSE, GLfCameraMat);
	glProgramUniformMatrix4fv(GLArbVertexShader, glGetUniformLocation(GLArbVertexShader, "v_MatModel"),  1, GL_FALSE, GLfModelMat);
	glProgramUniformMatrix4fv(GLArbVertexSelectShader, glGetUniformLocation(GLArbVertexSelectShader, "v_MatCamera"), 1, GL_FALSE, GLfCameraMat);
	glProgramUniformMatrix4fv(GLArbVertexSelectShader, glGetUniformLocation(GLArbVertexSelectShader, "v_MatModel"),  1, GL_FALSE, GLfModelMat);
};

bool ARAPGLControl::AnalyzeEdgeRelation(){
	iVertexNum = int(meshDisplayModel->vTotal);

	bool** bEdgeMap = new bool*[iVertexNum];
	for(int iTmpA = 0;iTmpA < iVertexNum;iTmpA++){
		bEdgeMap[iTmpA] = new bool[iVertexNum];
		for(int iTmpB = 0;iTmpB < iVertexNum;iTmpB++) bEdgeMap[iTmpA][iTmpB] = false;
	}

	int iFaceNum = meshDisplayModel->fTotal;

	for(int iFaceCount = 0;iFaceCount < iFaceNum;iFaceCount++){
		int iFaceVertex[3];
		for(int iTmp = 0;iTmp < 3;iTmp++) iFaceVertex[iTmp] = meshDisplayModel->faceList[iFaceCount].v[iTmp].v;
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
						for(int iTmp = 0;iTmp < 3;iTmp++) iFaceVertex[iTmp] = meshDisplayModel->faceList[iFaceCount].v[iTmp].v;
						
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
	for(int iEdgeCount = 0;iEdgeCount < iEdgeNum;iEdgeCount++){
		for(int iTmp = 0;iTmp < 4;iTmp++) cout<<iEdgeRelation[iEdgeCount][iTmp]<<" ";
		cout<<endl;
	}
#endif
	
	for(int iVertexCount = 0;iVertexCount < iVertexNum;iVertexCount++) delete bEdgeMap[iVertexCount];
	delete bEdgeMap;

	return true;
};
//GLUT Window Function

void ARAPGLControl::ARAPReshape(GLsizei GLsW, GLsizei GLsH){
 };

void ARAPGLControl::ARAPDisplay(){
	int iDrawedIndexNum = 0;

    glBindVertexArray(CurrentState->uiVAO);
	
	glClearColor(0.05f, 0.0f, 0.0f, 1.0f);
	glClearDepth(0.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	glUseProgram(CurrentState->GLArbFaceShader);
	glDrawElements(GL_TRIANGLES, 3 * GLsizei(CurrentState->meshDisplayModel->fTotal), GL_UNSIGNED_INT, (void*)(4 * iDrawedIndexNum));
	iDrawedIndexNum += 3 * GLsizei(CurrentState->meshDisplayModel->fTotal);

	if(CurrentState->bIsEdgeDisplayed){
		glUseProgram(CurrentState->GLArbEdgeShader);
		glDrawElements(GL_LINES, 2 * CurrentState->iEdgeNum, GL_UNSIGNED_INT,  (void*)(4 * iDrawedIndexNum));
	}
	iDrawedIndexNum += 2 * CurrentState->iEdgeNum;
	
	if(CurrentState->bIsVertDisplayed){
		glUseProgram(CurrentState->GLArbVertexShader);
		glDrawElements(GL_POINTS, GLsizei(CurrentState->meshDisplayModel->vTotal - 1), GL_UNSIGNED_INT,  (void*)(4 * iDrawedIndexNum));
	}
	iDrawedIndexNum += GLsizei(CurrentState->meshDisplayModel->vTotal - 1);

	if(CurrentState->iCurrentNearID != -1){
		//glUseProgram(CurrentState->GLArbVertexSelectShader);
	}

	glUseProgram(0);
	glBindVertexArray(0);

	glutSwapBuffers();
};

void ARAPGLControl::ARAPKeyboard(unsigned char ucKey, int iPosX, int iPosY){
	//double dSetPos1[2]  = {CurrentState->meshDisplayModel->vList[1].ptr[0], CurrentState->meshDisplayModel->vList[1].ptr[1] - 2.0};
	//double dSetPos4[2]  = {CurrentState->meshDisplayModel->vList[4].ptr[0], CurrentState->meshDisplayModel->vList[4].ptr[1] + 2.0};
	double dSetPos54[2] = {CurrentState->meshDisplayModel->vList[54].ptr[0], CurrentState->meshDisplayModel->vList[54].ptr[1] - 1.0};
	double dSetPos24[2] = {CurrentState->meshDisplayModel->vList[24].ptr[0], CurrentState->meshDisplayModel->vList[24].ptr[1] + 1.0};
	//double dSetPos10[2] = {0.0, -1.0};
	//double dSetPos59[2] = {0.0,  3.0};

	switch(ucKey){
		case('s'):
			CurrentState->bIsScaleVariant = bool(1 - CurrentState->bIsScaleVariant);
			CurrentState->ARAPmtrxSolver->SetScaleVariant(CurrentState->bIsScaleVariant);
			cout<<"bIsScaleVariant = "<<CurrentState->ARAPmtrxSolver->IsScaleVariant()<<endl;
			if(CurrentState->ARAPmtrxSolver->DeriveResult()){
				CurrentState->ARAPmtrxSolver->ApplyDeform();
				glBindBuffer(GL_ARRAY_BUFFER, CurrentState->uiVBO);
				CurrentState->Model2VAO();
			}
			break;
		case('1'):
			CurrentState->ARAPmtrxSolver->UpdateConstraint(24, dSetPos24);
			CurrentState->ARAPmtrxSolver->UpdateConstraint(54, dSetPos54);
			//CurrentState->ARAPmtrxSolver->UpdateConstraint(4, dSetPos4);
			//CurrentState->ARAPmtrxSolver->UpdateConstraint(1, dSetPos1);
			if(CurrentState->ARAPmtrxSolver->DeriveResult()){
				CurrentState->ARAPmtrxSolver->ApplyDeform();
				glBindBuffer(GL_ARRAY_BUFFER, CurrentState->uiVBO);
				CurrentState->Model2VAO();
			}
			break;
		case('e'):
			CurrentState->bIsEdgeDisplayed = 1 - CurrentState->bIsEdgeDisplayed;
			break;
		case('v'):
			CurrentState->bIsVertDisplayed = 1 - CurrentState->bIsVertDisplayed;
			break;
		case(' '):
			CurrentState->ARAPmtrxSolver->ResetConstraint();
			glBindBuffer(GL_ARRAY_BUFFER, CurrentState->uiVBO);
			CurrentState->Model2VAO();
	}

	glutSetWindow(CurrentState->iWindowCurrent);
	glutPostRedisplay();
};

void ARAPGLControl::ARAPMouse(int iButton, int iState, int iPosX, int iPosY){
	glutSetWindow(CurrentState->iWindowCurrent);

	CurrentState->iMouseButton = iButton;
	CurrentState->iMouseState  = iState;

	if(iState == GLUT_DOWN){
		CurrentState->iMouseClickPos[0] = iPosX;
		CurrentState->iMouseClickPos[1] = iPosY;
		CurrentState->iCurrentSelectedID = CurrentState->VertexSelect(iPosX, iPosY);
	}
	else{CurrentState->iCurrentSelectedID = -1;}
	glutPostRedisplay();
};

void ARAPGLControl::ARAPMotion(int iPosX, int iPosY){
	glutSetWindow(CurrentState->iWindowCurrent);
	int    iCroppedPos[2] = {iPosX, iPosY};
	double dSpacePos[2];
	if(iPosX < 0)							iCroppedPos[0] = double(0);
	if(iPosX > CurrentState->iWindowSize[0])iCroppedPos[0] = double(CurrentState->iWindowSize[0]);
	if(iPosY < 0)							iCroppedPos[1] = double(0);
	if(iPosY > CurrentState->iWindowSize[1])iCroppedPos[1] = double(CurrentState->iWindowSize[1]);
	
	CurrentState->Mouse2SpacePos(iPosX, iPosY, dSpacePos);

	//cout<<iPosX<<"  "<<iPosY<<" > "<<dSpacePos[0]<<"  "<<dSpacePos[1]<<endl;

	if(CurrentState->ARAPmtrxSolver->UpdateConstraint(CurrentState->iCurrentSelectedID, dSpacePos)){
		if(CurrentState->ARAPmtrxSolver->DeriveResult()){
			CurrentState->ARAPmtrxSolver->ApplyDeform();
			glBindBuffer(GL_ARRAY_BUFFER, CurrentState->uiVBO);
			CurrentState->Model2VAO();
		}
	}
	
	glutSetWindow(CurrentState->iWindowCurrent);
	glutPostRedisplay();
};

void ARAPGLControl::ARAPPassiveMotion(int iPosX, int iPosY){
	glutSetWindow(CurrentState->iWindowCurrent);
	CurrentState->iCurrentNearID = CurrentState->VertexSelect(iPosX, iPosY);
};