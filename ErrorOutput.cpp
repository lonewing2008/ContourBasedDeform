#include "ErrorOutput.h"

/*Inclusion of C/C++ standard library*/
#include <cmath>
#include <ctime>
#include <iostream>
using namespace std;

/*Inclusion of OpenGL*/
#include "glew.h"
#include "wglew.h"
#include "glut.h"
#include "glaux.h"

/*Other inclusion*/

void ErrorOutput(const char* ucFrom, int iMark){
	GLenum GLeError = glGetError();
	if(GLeError != GL_NO_ERROR){
		cout<<ucFrom<<" Mark "<<iMark<<" Error : ";
		switch(GLeError){
			case GL_INVALID_ENUM:
				cout<<"GL_INVALID_ENUM > An unacceptable value is specified for an enumerated argument."<<endl;
				break;
			case GL_INVALID_VALUE:
				cout<<"GL_INVALID_VALUE > A numeric argument is out of range."<<endl;
				break;
			case GL_INVALID_OPERATION:
				cout<<"GL_INVALID_OPERATION > The specified operation is not allowed in the current state."<<endl;
				break;
			case GL_INVALID_FRAMEBUFFER_OPERATION:
				cout<<"GL_INVALID_FRAMEBUFFER_OPERATION > The framebuffer object is not complete."<<endl;
				break;
			case GL_OUT_OF_MEMORY:
				cout<<"GL_OUT_OF_MEMORY > There is not enough memory left to execute the command."<<endl;
				break;
			case GL_STACK_UNDERFLOW:
				cout<<"GL_STACK_UNDERFLOW > An attempt has been made to perform an operation that would cause an internal stack to underflow."<<endl;
				break;
			case GL_STACK_OVERFLOW:
				cout<<"GL_STACK_OVERFLOW > An attempt has been made to perform an operation that would cause an internal stack to overflow."<<endl;
				break;
		}
	}
};