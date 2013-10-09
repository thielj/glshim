#ifndef GL_MATRIX_H
#define GL_MATRIX_H

#include "gl.h"

#ifdef __cplusplus
extern "C" {
#endif

void glLoadIdentity();
void glLoadMatrixf(const GLfloat *m);
void glMatrixMode(GLenum mode);
void glMultMatrixf(const GLfloat *m);
void glPopMatrix();
void glPushMatrix();
void gl_transform_vertex(GLfloat v[4]);

#ifdef __cplusplus
}
#endif

#endif
