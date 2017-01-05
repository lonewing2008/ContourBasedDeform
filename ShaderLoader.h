#ifndef GL_SHADER_LOADER
#define GL_SHADER_LOADER

#pragma once

#include "glew.h"
#include "wglew.h"
#include "glut.h"

bool ShaderLoad(GLuint h_program, char* shader_file, GLenum shader_type);

#endif