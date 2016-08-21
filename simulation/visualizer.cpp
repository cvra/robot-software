// OSX: g++ -g -Wno-deprecated -framework GLUT -framework OpenGL -framework Cocoa -lstdc++ visualizer.cpp -o test
// Linux: g++  -g -Wno-deprecated -lglut -lGL -lGLU -lm -L/usr/X11R6/lib -lX11 -lXext -lXmu -lXi  visualizer.cpp -o test

#include <math.h>
#include <stdio.h>

#ifdef __APPLE__
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else
#include <GL/glu.h>
#include <GL/glut.h>
#endif

#include "visualizer.h"

namespace {
    int main_window;
    int width, height;
    GLfloat aspect_ratio;
    const GLfloat table[] = {3000.0f, 2000.0f};
}

point_t *path = NULL;
size_t path_len = 0;

poly_t *obstacles = NULL;
size_t nb_obstacles = 0;

void visualizer_set_path(point_t* new_path, size_t new_len)
{
    path = new_path;
    path_len = new_len;
}

void visualizer_set_obstacles(poly_t* new_obstacles, size_t new_len)
{
    obstacles = new_obstacles;
    nb_obstacles = new_len;
}

static void draw_poly(poly_t *p)
{
    glBegin(GL_LINE_LOOP);
    int i;
    for (i = 0; i < p->l; i++) {
        glVertex2f(p->pts[i].x, p->pts[i].y);
    }
    glEnd();
}

static void vertex_circle(float px, float py, double radius, int points)
{
    int i;
    for (i = 0; i < points; i++) {
        GLfloat x, y;
        float phi = (float) i / points * 2 * M_PI;
        x = radius * cosf(phi);
        y = radius * sinf(phi);
        glVertex2f(px + x, py + y);
    }
}

void redraw_all()
{
    size_t i;

    glClear(GL_COLOR_BUFFER_BIT);

    glLoadIdentity();

    if (aspect_ratio <= table[0]/table[1]) {
        // window is narrower than table
        glOrtho(0,
                table[0],
                table[1] - table[0] / aspect_ratio,
                table[1],
                -1.0, 1.0);
    } else {
        glOrtho(0,
                table[1] * aspect_ratio,
                0,
                table[1],
                -1.0, 1.0);
    }

    // draw table
    glBegin(GL_LINE_LOOP);
    glColor3f(1.0,1.0,1.0);
    glVertex2f(0,0);
    glVertex2f(table[0],0);
    glVertex2f(table[0],table[1]);
    glVertex2f(0,table[1]);
    glEnd();

    // obstacles
    glColor3f(1.0,0.0,0.0);
    for (i = 0; i < nb_obstacles; i++) {
        draw_poly(&obstacles[i]);
    }

    // path
    glBegin(GL_LINES);
    glColor3f(0.0,1.0,0.0);
    for (i = 1; i < path_len; i++) {
        glVertex2f(path[i-1].x, path[i-1].y);
        glVertex2f(path[i].x, path[i].y);
    }
    glEnd();


    glutSwapBuffers();
}

void reshape_cb(int x, int y)
{
    aspect_ratio = (GLfloat) x / y;
    width = x;
    height = y;
    glViewport( 0, 0, width, height);
}

void display_cb()
{
    redraw_all();
}

void visualizer_run(void)
{
    int argc = 0;
    char **argv = NULL;
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowPosition(50, 50);
    aspect_ratio = table[0]/table[1];
    height = 400;
    width = aspect_ratio*height;
    glutInitWindowSize(width, height);

    main_window = glutCreateWindow("Obstacle avoidance");
    glutDisplayFunc(display_cb);
    glutReshapeFunc(reshape_cb);

    glutMainLoop();
}
