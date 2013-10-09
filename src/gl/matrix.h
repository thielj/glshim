#ifndef GL_MATRIX_H
#define GL_MATRIX_H

#include "gl.h"

void glLoadIdentity();
void glLoadMatrixf(const GLfloat *m);
void glMatrixMode(GLenum mode);
void glMultMatrixf(const GLfloat *m);
void glPopMatrix();
void glPushMatrix();

#endif
