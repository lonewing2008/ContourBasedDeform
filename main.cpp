/*Other inclusion*/
#include "GL_ParentWindow.h"

int main(int argc, char** argv){

	MainGLParentWindow* GLMainWindow = new MainGLParentWindow();
	GLMainWindow->InitialGLWindow(argc, argv);

	delete GLMainWindow;
	
	return 0;
}