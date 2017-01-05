#include "ShaderLoader.h"

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include "glew.h"
#include "wglew.h"
#include "glut.h"

bool ShaderLoad(GLuint h_program, char* shader_file, GLenum shader_type)
{
	FILE *fp;
	GLuint h_shader;
	GLchar *shader_string;
	GLint str_length;
	GLint isCompiled = GL_FALSE, isLinked = GL_FALSE;

	// open the file of shader source code
	if((fp=fopen(shader_file, "r")) == NULL)
	{
		fprintf(stderr, "Error : Failed to read the OpenGL shader source \"%s\".\n", shader_file);
		return false;
	}

	// allocate memory for program string and load it.
	shader_string = (GLchar*) malloc(sizeof(GLchar) * 65536);
	str_length = (GLint)fread(shader_string, 1, 65536, fp);
	fclose(fp);

	// Create and load shader string.
	h_shader = glCreateShader(shader_type);
	if(h_shader == 0)
	{
		fprintf(stderr, "Error : Failed to create OpenGL shader object \"%s\".\n", shader_file);
		return false;
	}
	glShaderSource(h_shader, 1, (const GLchar**)&shader_string, (const GLint*)&str_length);
	free(shader_string);

	// Compile the vertex shader, print out the compiler log message.
	glCompileShader(h_shader);

	// get compile state information
    {
        int result;
        glGetShaderiv( h_shader, GL_COMPILE_STATUS, &result );
        if ( result == GL_FALSE ) {
            glGetShaderiv( h_shader, GL_INFO_LOG_LENGTH, &result );
            char* buf = new char [ result ];
            glGetShaderInfoLog( h_shader, result, 0, buf );
            std::fprintf( stderr, "Vertex Shader Compile Error: %s\n", buf );
            delete [] buf;
        }
    }
	glAttachShader(h_program, h_shader);

	// delete the shader object, since we have attached it with the program object.
	glDeleteShader(h_shader);

	// Link the program and print out the linker log message
	glLinkProgram(h_program);

    {
        int result;
        glGetProgramiv( h_program, GL_LINK_STATUS, &result );
        if ( result == GL_FALSE ) {
            glGetProgramiv( h_program, GL_INFO_LOG_LENGTH, &result );
            char* buf = new char [ result ];
            glGetProgramInfoLog( h_program, result, 0, buf );
            std::fprintf( stderr, "Program Link Error: %s\n", buf );
            delete [] buf;
        }
    }

	return true;
}