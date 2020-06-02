#include <error/error.h>
#include <cmath>
#include <box2d/box2d.h>
#include "viewer.h"
#include <thread>
#include <GL/glut.h>

PhysicsRobot* robot_ptr = nullptr;

void display()
{
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    glLoadIdentity();

    glOrtho(0., 3., 0., 2., -1., 1.);

    b2Vec2 pos = robot_ptr->GetPosition();
    float angle_rad = robot_ptr->GetAngle();
    NOTICE_EVERY_N(10, "draw at %.3f, %.3f", pos.x, pos.y);

    glPushMatrix();
    glTranslatef(pos.x, pos.y, 0.);

    glRotatef(angle_rad / M_PI * 180., 0., 0., 1.);

    glBegin(GL_QUADS);
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex2f(-0.12, 0.12);
    glVertex2f(-0.12, -0.12);
    glVertex2f(0.12, -0.12);
    glVertex2f(0.12, 0.12);
    glEnd();

    glColor3f(0.f, 0.f, 0.f);
    glBegin(GL_LINES);
    glVertex2f(0., 0.);
    glVertex2f(0.12, 0.);
    glEnd();

    glPopMatrix();

    glutSwapBuffers();
}

void on_timer(int value)
{
    glutPostRedisplay();
    glutTimerFunc(33, on_timer, 0);
}

void startRendering(int argc, char** argv, PhysicsRobot& robot)
{
    glutInit(&argc, argv);
    glutInitWindowSize(1500, 1000);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
    glutCreateWindow("CVRA");

    glutDisplayFunc(display);
    glutTimerFunc(33, on_timer, 0);

    robot_ptr = &robot;

    glutMainLoop();
}
