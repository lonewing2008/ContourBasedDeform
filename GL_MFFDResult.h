#ifndef MFFD_GL_RESULT
#define MFFD_GL_RESULT

/*Inclusion of OpenGL*/
#include "glew.h"
#include "wglew.h"
#include "glut.h"
#include "glaux.h"

/*Other inclusion*/
#include "mesh.h"
#include "SparseMFFDSolver.h"

/*Global variable of window*/

class MFFDGLResult{
public:
	//Initialize Function
	MFFDGLResult();
	~MFFDGLResult();
	
	//Initialize Subfunction
	bool InitialGLWindow(int iParent, mesh* meshModel, SparseMFFDSolver* MFFDmtrxPtr);
	bool SetModel(mesh* meshModel);
	
	//Member Function
	int GetWindowID(){return iWindowCurrent;};

private:
	//Member Variable
	SparseMFFDSolver* MFFDmtrxSolver;

	mesh* meshDisplayModel;

	int iCurrentSelectedID;
	GLfloat GLfModelPos[3];
	GLfloat GLfModelDir[3];
	GLfloat GLfModelDefaultDir[3];
	
	GLhandleARB GLArbFaceShader;
	GLhandleARB GLArbVertShader;
	GLhandleARB GLArbEdgeShader;
	bool bIsEdgeDisplayed;
	bool bIsVertDisplayed;

	unsigned int uiVAO;
	unsigned int uiVBO;
	unsigned int uiIBO;

	GLuint GLuiModel;
	
	int iWindowCurrent;
	int iWindowSize[2];
	
	GLfloat GLfCameraPos[3];
	GLfloat GLfCameraDir[3];
	GLfloat GLfCameraUp[3];
	GLfloat GLfCameraRate;

	int iMouseButton;
	int iMouseState;
	int iMouseClickPos[2];
	
	int   iVertexNum;
	int   iEdgeNum;
	int** iEdgeRelation;

	double** dInitVertexPos;

	//Member Function
		//GLUT Function of Control Window
	static void MFFDReshape(GLsizei GLsW, GLsizei GLsH);
	static void MFFDDisplay();
	static void MFFDKeyboard(unsigned char ucKey, int iPosX, int iPosY);
	static void MFFDMouse(int iButton, int iState, int iPosX, int iPosY);
	static void MFFDMotion(int iPosX, int iPosY);
	
		//GLUT Subfunction
	bool InitialShader();

	void InitialVAO();
	void TerminateVAO();
	void Model2VAO();

	void CameraSet();

	void ResetModel();
	
	bool AnalyzeEdgeRelation();
};

#endif