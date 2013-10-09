#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "gl.h"
#include "matrix.h"

using Eigen::Affine3f;
using Eigen::AngleAxisf;
using Eigen::Map;
using Eigen::Matrix4f;
using Eigen::Vector3f;

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
        Affine3f *matrix = new Affine3f;
        matrix->setIdentity();
        m->matrix = static_cast<void *>(matrix);
    }
    return m;
}

static Affine3f *get_matrix(GLenum mode) {
    matrix_state_t *state = get_matrix_state(mode);
    return static_cast<Affine3f *>(state->matrix);
}

static Affine3f *get_current_matrix() {
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
    Matrix4f *m = &get_current_matrix()->matrix();
    Matrix4f tmp = Map<Matrix4f>((GLfloat *)load);
    *m = tmp;
}

void glMatrixMode(GLenum mode) {
    state.matrix.mode = mode;
}

void glMultMatrixf(const GLfloat *mult) {
    Matrix4f *m = &get_current_matrix()->matrix();
    Matrix4f tmp = Map<Matrix4f>((GLfloat *)mult);
    *m *= tmp;
}

void glPopMatrix() {
    matrix_state_t *m = get_current_state();
    if (m->stack.len > 0) {
        void *state = m->stack.list[--m->stack.len];
        delete static_cast<Affine3f *>(m->matrix);
        m->matrix = state;
    }
}

void glPushMatrix() {
    matrix_state_t *m = get_current_state();
    if (m->stack.len + 1 > m->stack.cap) {
        m->stack.cap += 5;
        m->stack.list = (void **)realloc(m->stack.list, sizeof(void *) * m->stack.cap);
    }
    Affine3f *matrix = new Affine3f(*static_cast<Affine3f *>(m->matrix));
    m->stack.list[m->stack.len++] = static_cast<void *>(matrix);
}

// GL transform functions
void glRotatef(GLfloat angle, GLfloat x, GLfloat y, GLfloat z) {
    angle *= (M_PI / 180.0f);
    get_current_matrix()->rotate(AngleAxisf(angle, Vector3f(x, y, z)));
}

void glScalef(GLfloat x, GLfloat y, GLfloat z) {
    get_current_matrix()->scale(Vector3f(x, y, z));
}

void glTransformf(GLfloat x, GLfloat y, GLfloat z) {
    get_current_matrix()->translate(Vector3f(x, y, z));
}

void glOrthof(GLfloat left, GLfloat right,
              GLfloat bottom, GLfloat top,
              GLfloat near, GLfloat far) {
    GLfloat tx, ty, tz;
    tx = -(right + left) / (right - left);
    ty = -(top + bottom) / (top - bottom);
    tz = -(far + near) / (far - near);
    GLfloat tmp[] = {
        2 / (right - left), 0, tx,
        0, 2 / (top - bottom), ty,
        0, 0, -2 / (far - near), tz,
        0, 0, 0, 1,
    };
    glMultMatrixf(tmp);
}

void glFrustumf(GLfloat left, GLfloat right,
                GLfloat bottom, GLfloat top,
                GLfloat near, GLfloat far) {
    GLfloat A, B, C, D;
    A = (right + left) / (right - left);
    B = (top + bottom) / (top - bottom);
    C = -(far + near) / (far - near);
    D = -(2 * far * near) / (far - near);
    GLfloat tmp[] = {
        (2 * near) / (right - left), 0, A, 0,
        0, (2 * near) / (top - bottom), B, 0,
        0, 0, C, D,
        0, 0, -1, 0,
    };
    glMultMatrixf(tmp);
}

void gl_transform_vertex(GLfloat v[3]) {
    Affine3f *model = get_matrix(GL_MODELVIEW);
    Affine3f *projection = get_matrix(GL_PROJECTION);
    Map<Vector3f> vert(v);
    vert = *model * vert;
    vert = *projection * vert;
    memcpy(v, vert.data(), sizeof(GLfloat) * 3);
}

} // extern "C"
