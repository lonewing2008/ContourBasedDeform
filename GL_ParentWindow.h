#ifndef MAIN_GL_PARENTWINDOW
#define MAIN_GL_PARENTWINDOW

/*Inclusion of OpenGL*/
#include "glew.h"
#include "wglew.h"
#include "glut.h"
#include "glaux.h"

/*Other inclusion*/
#include "mesh.h"
#include "GL_ARAPControl.h"
#include "GL_MFFDResult.h"

/*Global variable of window*/

class MainGLParentWindow{
public:
	//Initialize Function
	MainGLParentWindow();
	~MainGLParentWindow();
	
	//Initialize Subfunction
	bool InitialGLWindow(int argc, char** argv);

	//Member Function
	int GetWindowID(){return iWindowCurrent;};
	int GetLeftChildID(){return iWindowChildLeft;};
	int GetRightChildID(){return iWindowChildRight;};

private:
	//Member Variable
	int iWindowCurrent;
	int iWindowChildLeft;
	int iWindowChildRight;

	int iWindowSize[2];
	
	ARAPGLControl* ARAPLeftChild;
	MFFDGLResult*  MFFDRightChild;
	SparseARAPSolver*   ARAPmtrxSolver;
	SparseMFFDSolver*   MFFDmtrxSolver;
	
	mesh* meshModel;
	mesh* meshModelFlatten;

	//Member Function
		//GLUT Function of Control Window
	static void MainReshape(GLsizei GLsW, GLsizei GLsH);
	static void MainDisplay();
	
		//GLUT Subfunction
	bool InitialParentWindow(int argc, char** argv);
	void MeshLoad(char* cMeshName);

	void TesterConstraintSetup(bool* dConstraintList);
};

#endif