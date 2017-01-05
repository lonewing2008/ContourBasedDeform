#include "GL_ParentWindow.h"

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

/*Global variable of window*/
static MainGLParentWindow* CurrentState;
//char* cMeshName = "TestPlane.obj";
char* cMeshName = "LuluOBJExport5.obj";

MainGLParentWindow::MainGLParentWindow(){
	CurrentState = this;

	iWindowCurrent    = 0;
	iWindowChildLeft  = 0;
	iWindowChildRight = 0;

	iWindowSize[0] = 1200;
	iWindowSize[1] = 800;

	ARAPLeftChild  = new ARAPGLControl();
	ARAPmtrxSolver = new SparseARAPSolver();

	MFFDRightChild = new MFFDGLResult();
	MFFDmtrxSolver = new SparseMFFDSolver();

	meshModel = NULL;
	meshModelFlatten = NULL;
};

MainGLParentWindow::~MainGLParentWindow(){
	delete ARAPLeftChild;
	delete ARAPmtrxSolver;

	delete MFFDRightChild;
	delete MFFDmtrxSolver;

	delete meshModel;
	delete meshModelFlatten;
};

bool MainGLParentWindow::InitialGLWindow(int argc, char** argv){
	this->MeshLoad(cMeshName);
	if(!(this->InitialParentWindow(argc, argv))) return false;
	return true;
};

void MainGLParentWindow::MainReshape(GLsizei GLsW, GLsizei GLsH){
	glutSetWindow(CurrentState->iWindowCurrent);
	glutReshapeWindow(CurrentState->iWindowSize[0], CurrentState->iWindowSize[1]);
};

void MainGLParentWindow::MainDisplay(){
	glutSetWindow(CurrentState->iWindowCurrent);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glutSwapBuffers();
};

void MainGLParentWindow::MeshLoad(char* cMeshName){
	meshModel        = NULL;
	meshModelFlatten = NULL;

	meshModel        = new mesh(cMeshName);
	meshModelFlatten = new mesh(cMeshName);
};

bool MainGLParentWindow::InitialParentWindow(int argc, char** argv){
	ARAPmtrxSolver->InitialARAPMatrix(meshModelFlatten);
	MFFDmtrxSolver->InitialMFFDMatrix(meshModel, ARAPmtrxSolver);

	bool* dConstraintList = new bool[meshModel->vTotal];
	for(int iTmp = 0;iTmp < meshModel->vTotal;iTmp++) dConstraintList[iTmp] = false;
	
	//TesterConstraintSetup(dConstraintList);
	//MFFDmtrxSolver->SetConstraintList(dConstraintList);

	glutInit(&argc, argv);

	glutInitWindowSize(iWindowSize[0], iWindowSize[1]);
	glutInitWindowPosition(600, 100);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	iWindowCurrent = glutCreateWindow("Deformation Simple Test");
	glutDisplayFunc(MainDisplay);
	glutReshapeFunc(MainReshape);
	
	glewExperimental = GL_TRUE;
	int result = glewInit();
	if(result != GLEW_OK){
		cout << "glew initialize error\n";
		return false;
	}

	glutSetWindow(iWindowCurrent);
	ARAPLeftChild->InitialGLWindow(iWindowCurrent, meshModelFlatten, ARAPmtrxSolver);

	glutSetWindow(iWindowCurrent);
	MFFDRightChild->InitialGLWindow(iWindowCurrent, meshModel, MFFDmtrxSolver);

	iWindowChildLeft  = ARAPLeftChild->GetWindowID();
	iWindowChildRight = MFFDRightChild->GetWindowID();
	
	glutMainLoop();

	return true;
};
/*
void MainGLParentWindow::TesterConstraintSetup(bool* dConstraintList){
	dConstraintList[1]  = true;
	dConstraintList[2]  = true;
	dConstraintList[3]  = true;
	dConstraintList[4]  = true;
	dConstraintList[6]  = true;
	dConstraintList[8]  = true;
	dConstraintList[10]  = true;
	dConstraintList[11] = true;
	dConstraintList[12] = true;
	dConstraintList[13] = true;
	dConstraintList[14] = true;
	dConstraintList[18] = true;
	dConstraintList[19] = true;
	dConstraintList[23] = true;
	dConstraintList[24] = true;
	dConstraintList[27] = true;
	dConstraintList[29] = true;
	dConstraintList[31] = true;
	dConstraintList[32] = true;
	dConstraintList[33] = true;
};
*/