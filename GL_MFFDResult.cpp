#include "GL_MFFDResult.h"

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
static MFFDGLResult* CurrentState;

MFFDGLResult::MFFDGLResult(){
	CurrentState = this;

	MFFDmtrxSolver   = NULL;
	meshDisplayModel = NULL;

	iCurrentSelectedID = -1;
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
	
	dInitVertexPos = NULL;
};

MFFDGLResult::~MFFDGLResult(){
};

bool MFFDGLResult::SetModel(mesh* meshModel){
	glutSetWindow(CurrentState->iWindowCurrent);
	TerminateVAO();
	meshDisplayModel = meshModel;
	InitialVAO();
	return true;
};

bool MFFDGLResult::InitialGLWindow(int iParent, mesh* meshModel, SparseMFFDSolver* MFFDmtrxPtr){
	MFFDmtrxSolver   = MFFDmtrxPtr;
	meshDisplayModel = meshModel;

	if(!AnalyzeEdgeRelation()) return false;

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	iWindowCurrent = glutCreateSubWindow(iParent, iWindowSize[0], 0, iWindowSize[0], iWindowSize[1]);
	glutSetWindow(CurrentState->iWindowCurrent);
	InitialShader();
	InitialVAO();
	CameraSet();

	glutReshapeFunc(NULL);
	glutMouseFunc(&MFFDGLResult::MFFDMouse);
	glutMotionFunc(&MFFDGLResult::MFFDMotion);
	glutDisplayFunc(&MFFDGLResult::MFFDDisplay);
 	glutKeyboardFunc(&MFFDGLResult::MFFDKeyboard);
	return true;
};

void MFFDGLResult::InitialVAO(){
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
	
	unsigned int iPositionAttrLoc = glGetAttribLocation(GLArbFaceShader, "v_V4Position");
	glEnableVertexAttribArray(iPositionAttrLoc);
	glVertexAttribPointer(iPositionAttrLoc, 4, GL_FLOAT, GL_FALSE, 0, 0);

	unsigned int iNormalAttrLoc   = glGetAttribLocation(GLArbFaceShader, "v_V4Normal");
	glEnableVertexAttribArray(iNormalAttrLoc);
	glVertexAttribPointer(iNormalAttrLoc,   4, GL_FLOAT, GL_FALSE, 0, reinterpret_cast<void*>(4 * iVertexNum * sizeof(GLfloat)));

	unsigned int iTexCoordAttrLoc = glGetAttribLocation(GLArbFaceShader, "v_V4TexCoord");
	glEnableVertexAttribArray(iTexCoordAttrLoc);
	glVertexAttribPointer(iTexCoordAttrLoc, 4, GL_FLOAT, GL_FALSE, 0, reinterpret_cast<void*>(4 * (iVertexNum + iNormalNum) * sizeof(GLfloat)));
    
	glBindVertexArray(0);
	
	dInitVertexPos = new double*[iVertexNum];
	for(int iVertexCount = 0;iVertexCount < iVertexNum;iVertexCount++){
		dInitVertexPos[iVertexCount] = new double[3];
		for(int iTmp = 0;iTmp < 3;iTmp++) dInitVertexPos[iVertexCount][iTmp] = meshDisplayModel->vList[iVertexCount].ptr[iTmp];
	}
};

bool MFFDGLResult::InitialShader(){
	glutSetWindow(CurrentState->iWindowCurrent);

	GLArbFaceShader = glCreateProgramObjectARB();
	if(GLArbFaceShader == 0) return false;

	ShaderLoad(GLArbFaceShader, "MFFDFaceDisplayVert.txt", GL_VERTEX_SHADER_ARB);
	ShaderLoad(GLArbFaceShader, "MFFDFaceDisplayFrag.txt", GL_FRAGMENT_SHADER_ARB);
	
	GLArbVertShader = glCreateProgramObjectARB();
	if(GLArbVertShader == 0) return false;

	ShaderLoad(GLArbVertShader, "MFFDVertDisplayVert.txt", GL_VERTEX_SHADER_ARB);
	ShaderLoad(GLArbVertShader, "MFFDVertDisplayFrag.txt", GL_FRAGMENT_SHADER_ARB);
	
	GLArbEdgeShader = glCreateProgramObjectARB();
	if(GLArbEdgeShader == 0) return false;

	ShaderLoad(GLArbEdgeShader, "MFFDEdgeDisplayVert.txt", GL_VERTEX_SHADER_ARB);
	ShaderLoad(GLArbEdgeShader, "MFFDEdgeDisplayFrag.txt", GL_FRAGMENT_SHADER_ARB);
	return true;
};

void MFFDGLResult::TerminateVAO(){
	glutSetWindow(CurrentState->iWindowCurrent);

	glBindVertexArray(uiVAO);
	
	glDeleteBuffers(1, &uiVBO);
	glDeleteBuffers(1, &uiIBO);
	
	glBindVertexArray(0);
	
	glDeleteVertexArrays(1, &uiVAO);
};

void MFFDGLResult::Model2VAO(){
	glutSetWindow(CurrentState->iWindowCurrent);

	int iVertexNum  = int(meshDisplayModel->vTotal);
	int iNormalNum  = int(meshDisplayModel->nTotal);
	int iTextureNum = int(meshDisplayModel->tTotal);

	GLfloat* GLfVertex   = (GLfloat*)malloc(4 * iVertexNum  * sizeof(GLfloat));
	GLfloat* GLfNormal   = (GLfloat*)malloc(4 * iNormalNum  * sizeof(GLfloat));
	GLfloat* GLfTexCoord = (GLfloat*)malloc(4 * iTextureNum * sizeof(GLfloat));
	
	for(int iVerCount = 0;iVerCount < iVertexNum;iVerCount++){
		for(int iTemp = 0;iTemp < 3;iTemp++){
			GLfVertex[iVerCount * 4 + iTemp] = meshDisplayModel->vList[iVerCount].ptr[iTemp];
			GLfVertex[iVerCount * 4 + 3] = 1.0f;
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
};

void MFFDGLResult::CameraSet(){
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
	glProgramUniformMatrix4fv(GLArbVertShader, glGetUniformLocation(GLArbVertShader, "v_MatCamera"), 1, GL_FALSE, GLfCameraMat);
	glProgramUniformMatrix4fv(GLArbVertShader, glGetUniformLocation(GLArbVertShader, "v_MatModel"),  1, GL_FALSE, GLfModelMat);
	glProgramUniformMatrix4fv(GLArbEdgeShader, glGetUniformLocation(GLArbEdgeShader, "v_MatCamera"), 1, GL_FALSE, GLfCameraMat);
	glProgramUniformMatrix4fv(GLArbEdgeShader, glGetUniformLocation(GLArbEdgeShader, "v_MatModel"),  1, GL_FALSE, GLfModelMat);
};

bool MFFDGLResult::AnalyzeEdgeRelation(){
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

 void MFFDGLResult::MFFDReshape(GLsizei GLsW, GLsizei GLsH){
	//glutSetWindow(CurrentState->iWindowCurrent);
 };

void MFFDGLResult::MFFDDisplay(){
	int iDrawedIndexNum = 0;

    glBindVertexArray(CurrentState->uiVAO);
	
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
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
		glUseProgram(CurrentState->GLArbVertShader);
		glDrawElements(GL_POINTS, GLsizei(CurrentState->meshDisplayModel->vTotal - 1), GL_UNSIGNED_INT,  (void*)(4 * iDrawedIndexNum));
	}
	iDrawedIndexNum += GLsizei(CurrentState->meshDisplayModel->vTotal - 1);

    glUseProgram(0);

    glBindVertexArray(0);
	
	glutSwapBuffers();
};

void MFFDGLResult::MFFDKeyboard(unsigned char ucKey, int iPosX, int iPosY){
	static int iIterCount = 0;
	glutSetWindow(CurrentState->iWindowCurrent);
	cout<<ucKey<<endl;
	switch(ucKey){
		case('c'):
			if(!CurrentState->MFFDmtrxSolver->UpdateConstraint()) cout<<"Error : No Constraint Info"<<endl;
			if(!CurrentState->MFFDmtrxSolver->DeriveResult())     cout<<"Error : Derivation Failure"<<endl;
			glBindBuffer(GL_ARRAY_BUFFER, CurrentState->uiVBO);
			CurrentState->Model2VAO();
			break;
		case('e'):
			CurrentState->bIsEdgeDisplayed = 1 - CurrentState->bIsEdgeDisplayed;
			break;
		case('v'):
			CurrentState->bIsVertDisplayed = 1 - CurrentState->bIsVertDisplayed;
			break;
		case('`'):
			if(!CurrentState->MFFDmtrxSolver->UpdateConstraint()) cout<<"Error : No Constraint Info"<<endl;
			iIterCount++;
			if(!CurrentState->MFFDmtrxSolver->DeriveResult(iIterCount, iIterCount)){
				cout<<"Error : Derivation Failure"<<endl;
				iIterCount = 0;
				break;
			}
			glBindBuffer(GL_ARRAY_BUFFER, CurrentState->uiVBO);
			CurrentState->Model2VAO();
			break;
		case(' '):
			iIterCount = 0;
			CurrentState->ResetModel();
			glBindBuffer(GL_ARRAY_BUFFER, CurrentState->uiVBO);
			CurrentState->Model2VAO();
			break;
	}
	glutPostRedisplay();
};

void MFFDGLResult::MFFDMouse(int iButton, int iState, int iPosX, int iPosY){
	glutSetWindow(CurrentState->iWindowCurrent);
	/*
	CurrentState->iMouseButton = iButton;
	CurrentState->iMouseState  = iState;
	if(iState == GLUT_DOWN){
		CurrentState->iMouseClickPos[0] = iPosX;
		CurrentState->iMouseClickPos[1] = iPosY;
	}
	else{
		CurrentState->iCurrentSelectedID = -1;
	}
	glutPostRedisplay();
	*/
};

void MFFDGLResult::MFFDMotion(int iPosX, int iPosY){
	glutSetWindow(CurrentState->iWindowCurrent);
	/*
	glBindBuffer(GL_ARRAY_BUFFER, CurrentState->uiVBO);
	CurrentState->Model2VAO();
	glutPostRedisplay();
	*/
};

void MFFDGLResult::ResetModel(){
	for(int iVertexCount = 0;iVertexCount < iVertexNum;iVertexCount++){
		for(int iTmp = 0;iTmp < 3;iTmp++){
			meshDisplayModel->vList[iVertexCount].ptr[iTmp] = dInitVertexPos[iVertexCount][iTmp];
		}
	}
};
