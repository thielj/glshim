#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "gl.h"
#include "matrix.h"

using Eigen::AngleAxis;
using Eigen::Map;
using Eigen::Matrix4f;
using Eigen::Transform;
using Eigen::Translation3f;
using Eigen::Vector3f;
using Eigen::Vector4f;

extern "C" {

#define CURRENT_MATRIX_MODE state.matrix.mode ? state.matrix.mode : GL_MODELVIEW

// helper functions
static matrix_state_t *get_matrix_state(GLenum mode) {
    matrix_state_t *m;
    switch (mode) {
        case GL_MODELVIEW:
            m = &state.matrix.model;
            break;
        case GL_PROJECTION:
            m = &state.matrix.projection;
            break;
        case GL_TEXTURE:
            m = &state.matrix.texture;
            break;
    /* defined in ARB_imaging extension
        case GL_COLOR:
            m = &state.matrix.color;
            break;
    */
    }

    if (! m->matrix) {
        Matrix4f *matrix = new Matrix4f;
        matrix->setIdentity();
        m->matrix = static_cast<void *>(matrix);
    }
    return m;
}

static Matrix4f *get_matrix(GLenum mode) {
    matrix_state_t *state = get_matrix_state(mode);
    return static_cast<Matrix4f *>(state->matrix);
}

static Matrix4f *get_current_matrix() {
    return get_matrix(CURRENT_MATRIX_MODE);
}

static matrix_state_t *get_current_state() {
    return get_matrix_state(CURRENT_MATRIX_MODE);
}

// GL matrix functions
void glLoadIdentity() {
    get_current_matrix()->setIdentity();
}

void glLoadMatrixf(const GLfloat *load) {
    matrix_state_t *m = get_current_state();
    Matrix4f tmp = Map<Matrix4f>((GLfloat *)load);
    m->matrix = new Matrix4f(tmp);
}

void glMatrixMode(GLenum mode) {
    state.matrix.mode = mode;
}

void glMultMatrixf(const GLfloat *mult) {
    Matrix4f *m = get_current_matrix();
    Matrix4f tmp = Map<Matrix4f>((GLfloat *)mult);
    *m *= tmp;
}

void glPopMatrix() {
    matrix_state_t *m = get_current_state();
    if (m->stack.len > 0) {
        void *state = m->stack.list[--m->stack.len];
        delete static_cast<Matrix4f *>(m->matrix);
        m->matrix = state;
    }
}

void glPushMatrix() {
    matrix_state_t *m = get_current_state();
    if (m->stack.len + 1 > m->stack.cap) {
        m->stack.cap += 5;
        m->stack.list = (void **)realloc(m->stack.list, sizeof(void *) * m->stack.cap);
    }
    Matrix4f *matrix = new Matrix4f(*static_cast<Matrix4f *>(m->matrix));
    m->stack.list[m->stack.len++] = static_cast<void *>(matrix);
}

// GL transform functions
void glRotatef(GLfloat angle, GLfloat x, GLfloat y, GLfloat z) {
    Matrix4f *matrix = get_current_matrix();
    AngleAxis<float> rotate(angle, Vector3f(x, y, z));
    Transform<float, 3, 3> t;
    t = rotate.toRotationMatrix();
    *matrix *= t.matrix();
}

void glScalef(GLfloat x, GLfloat y, GLfloat z) {
    Matrix4f *matrix = get_current_matrix();
    Eigen::DiagonalMatrix<float, 4> scale(Vector4f(x, y, z, 1));
    *matrix *= scale;
}

void glTransformf(GLfloat x, GLfloat y, GLfloat z) {
    Matrix4f *matrix = get_current_matrix();
    Translation3f t(x, y, z);
    Transform<float, 3, 3> m;
    m = t;
    *matrix *= m.matrix();
}

void gl_transform_vertex(GLfloat v[4]) {
    Matrix4f *model = get_matrix(GL_MODELVIEW);
    Matrix4f *projection = get_matrix(GL_PROJECTION);
    Map<Vector4f> vert(v);
    vert = *model * vert;
    vert = *projection * vert;
    memcpy(v, vert.data(), sizeof(GLfloat) * 4);
}

} // extern "C"
