#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "gl.h"
#include "matrix.h"

using namespace Eigen;

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
    Matrix4f tmp = Map<Matrix4f>((GLfloat *)load);
    *get_current_matrix() = tmp;
}

void glMatrixMode(GLenum mode) {
    state.matrix.mode = mode;
}

void glMultMatrixf(const GLfloat *mult) {
    Matrix4f tmp = Map<Matrix4f>((GLfloat *)mult);
    *get_current_matrix() *= tmp;
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
    angle *= (M_PI / 180.0f);
    Affine3f rotate(AngleAxisf(angle, Vector3f(x, y, z)));
    *get_current_matrix() *= rotate.matrix();
}

void glScalef(GLfloat x, GLfloat y, GLfloat z) {
    DiagonalMatrix<float, 4> scale(Vector4f(x, y, z, 1));
    *get_current_matrix() *= scale;
}

void glTransformf(GLfloat x, GLfloat y, GLfloat z) {
    Transform<float, 3, 3> translate;
    translate = Translation3f(x, y, z);
    *get_current_matrix() *= translate.matrix();
}

void glOrthof(GLfloat left, GLfloat right,
              GLfloat bottom, GLfloat top,
              GLfloat near, GLfloat far) {
    GLfloat tx, ty, tz;
    tx = -(right + left) / (right - left);
    ty = -(top + bottom) / (top - bottom);
    tz = -(far + near) / (far - near);

    Matrix4f tmp;
    tmp << 2 / (right - left), 0, 0, tx,
           0, 2 / (top - bottom), 0, ty,
           0, 0, -2 / (far - near), tz,
           0, 0, 0, 1;

    *get_current_matrix() *= tmp;
}

void glFrustumf(GLfloat left, GLfloat right,
                GLfloat bottom, GLfloat top,
                GLfloat near, GLfloat far) {
    GLfloat A, B, C, D;
    A = (right + left) / (right - left);
    B = (top + bottom) / (top - bottom);
    C = -(far + near) / (far - near);
    D = -(2 * far * near) / (far - near);

    Matrix4f tmp;
    tmp << (2 * near) / (right - left), 0, A, 0,
           0, (2 * near) / (top - bottom), B, 0,
           0, 0, C, D,
           0, 0, -1, 0;

    *get_current_matrix() *= tmp;
}

void gl_get_matrix(GLenum mode, GLfloat *out) {
    memcpy(out, get_matrix(mode)->data(), sizeof(GLfloat) * 16);
}

void gl_transform_vertex(GLfloat v[3]) {
    Matrix4f *model = get_matrix(GL_MODELVIEW);
    Matrix4f *projection = get_matrix(GL_PROJECTION);
    Matrix4f MVP = (*projection) * (*model);

    Map<Vector3f> vert(v);
    Vector3f out;
    out.noalias() = (MVP * vert.homogeneous()).colwise().hnormalized();
    memcpy(v, out.data(), sizeof(GLfloat) * 3);
}

} // extern "C"
