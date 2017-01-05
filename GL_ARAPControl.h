#ifndef ARAP_GL_CONTROL
#define ARAP_GL_CONTROL

/*Inclusion of OpenGL*/
#include "glew.h"
#include "wglew.h"
#include "glut.h"
#include "glaux.h"

/*Other inclusion*/
#include "mesh.h"
#include "SparseARAPSolver.h"

/*Global variable of window*/

class ARAPGLControl{
public:
	//Initialize Function
	ARAPGLControl();
	~ARAPGLControl();

	//Initialize Subfunction
	bool InitialGLWindow(int iParent, mesh* meshModel, SparseARAPSolver* ARAPmtrxPtr);
	bool SetModel(mesh* meshModel);

	//Member Function
	int GetWindowID(){return iWindowCurrent;};

private:
	//Member Variable
	SparseARAPSolver* ARAPmtrxSolver; 

	mesh* meshDisplayModel;

	bool bIsScaleVariant;

	int iCurrentSelectedID;
	int iCurrentNearID;
	GLfloat GLfModelPos[3];
	GLfloat GLfModelDir[3];
	GLfloat GLfModelDefaultDir[3];
	
	GLhandleARB GLArbFaceShader;
	GLhandleARB GLArbEdgeShader;
	GLhandleARB GLArbVertexShader;
	GLhandleARB GLArbVertexSelectShader;
	bool bIsEdgeDisplayed;
	bool bIsVertDisplayed;

	unsigned int uiVAO;
	unsigned int uiVBO;
	unsigned int uiIBO;

	GLuint GLuiModel;
	
	int iWindowCurrent;
	int iWindowSize[2];

	int iMouseButton;
	int iMouseState;
	int iMouseClickPos[2];

	GLfloat GLfCameraPos[3];
	GLfloat GLfCameraDir[3];
	GLfloat GLfCameraUp[3];
	GLfloat GLfCameraRate;
	
	int   iVertexNum;
	int   iEdgeNum;
	int** iEdgeRelation;

	//Member Function
		//GLUT Function of Control Window
	static void ARAPReshape(GLsizei GLsW, GLsizei GLsH);
	static void ARAPDisplay();
	static void ARAPKeyboard(unsigned char ucKey, int iPosX, int iPosY);
	static void ARAPMouse(int iButton, int iState, int iPosX, int iPosY);
	static void ARAPMotion(int iPosX, int iPosY);
	static void ARAPPassiveMotion(int iPosX, int iPosY);

		//GLUT Subfunction
	bool InitialShader();

	void InitialVAO();
	void TerminateVAO();
	void Model2VAO();

	void CameraSet();

	bool AnalyzeEdgeRelation();

		//Vertex Selection
	int VertexSelect(int iPosX, int iPosY);
	void Mouse2SpacePos(int iPosX, int iPosY, double* dSpacePos);
	
};

#endif