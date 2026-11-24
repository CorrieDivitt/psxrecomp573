/* android_gl_stubs.c - Android / Meta Quest (psxrecomp573, 2026-11-23).
 * gpu_gl_renderer.c is written for desktop OpenGL. GLES 3 has the rest of
 * what it links against, but not these two desktop calls; with them stubbed
 * the runtime links, and runs with the software renderer (--renderer software)
 * until the GL renderer is ported to GLES. Nothing here is reached by it. */
#if defined(__ANDROID__)
#include <stdio.h>

typedef unsigned int GLenum;
typedef int GLint;
typedef int GLsizei;
typedef double GLdouble;

void glGetTexImage(GLenum target, GLint level, GLenum format, GLenum type, void *pixels) {
    (void)target; (void)level; (void)format; (void)type; (void)pixels;
    fprintf(stderr, "psxrecomp: glGetTexImage is not available on GLES (Android)\n");
}

void glClearDepth(GLdouble depth) {
    (void)depth;   /* GLES has glClearDepthf; the desktop renderer is not used here */
}
#endif
