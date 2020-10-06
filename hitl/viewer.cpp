#include <error/error.h>
#include <cmath>
#include <box2d/box2d.h>
#include "viewer.h"
#include <thread>
#include "main.h"
#include <GL/glut.h>

std::vector<Renderable*>* renderables;

int window_width, window_height;

// Helper function to pass hex colors to OpenGL, as this is the most common
// format when taking color from screens
static void hexColor(int r, int g, int b)
{
    glColor3f(r / 256.f, g / 256.f, b / 256.f);
}

void RobotRenderer::render()
{
    b2Vec2 pos = robot.GetPosition();
    float angle_rad = robot.GetAngle();

    DEBUG_EVERY_N(10, "draw robot at %.3f, %.3f", pos.x, pos.y);

    glPushMatrix();
    glTranslatef(pos.x, pos.y, 0.);

    glRotatef(angle_rad / M_PI * 180., 0., 0., 1.);

    glBegin(GL_QUADS);

    hexColor(0xef, 0xa0, 0x18);

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
}

void OpponentRenderer::render()
{
    b2Vec2 pos = opponent.GetPosition();

    DEBUG_EVERY_N(10, "draw robot at %.3f, %.3f", pos.x, pos.y);

    glPushMatrix();

    glTranslatef(pos.x, pos.y, 0.);

    const int sides = 100;

    glBegin(GL_POLYGON);
    hexColor(0xc0, 0x39, 0x2b);

    for (int i = 0; i < sides; i++) {
        float angle = 2 * M_PI * (float)i / sides;
        float x = OpponentRobot::radius * cosf(angle);
        float y = OpponentRobot::radius * sinf(angle);
        glVertex2f(x, y);
    }

    glEnd();

    glPopMatrix();
}

void CupRenderer::render()
{
    b2Vec2 pos = cup->GetPosition();

    switch (color) {
        case CupColor::RED:
            hexColor(0xab, 0x3c, 0x37);
            break;

        case CupColor::GREEN:
            hexColor(0x30, 0x5b, 0x35);
            break;
    }

    glPushMatrix();
    glTranslatef(pos.x, pos.y, 0.);

    const int sides = 10;

    glBegin(GL_POLYGON);

    for (int i = 0; i < sides; i++) {
        float angle = 2 * M_PI * (float)i / sides;
        float x = PhysicsCup::radius * cosf(angle);
        float y = PhysicsCup::radius * sinf(angle);
        glVertex2f(x, y);
    }

    glEnd();

    glPopMatrix();
}

void TableRenderer::render()
{
    glEnable(GL_TEXTURE_2D);
    glColor3f(1., 1., 1.);

    glBindTexture(GL_TEXTURE_2D, texture_id);

    glBegin(GL_QUADS);

    glTexCoord2f(0., 1.);
    glVertex2f(0.f, 2.f);

    glTexCoord2f(0., 0.);
    glVertex2f(0.f, 0.f);

    glTexCoord2f(1., 0.);
    glVertex2f(3.f, 0.f);

    glTexCoord2f(1., 1.);
    glVertex2f(3.f, 2.f);

    glEnd();

    glDisable(GL_TEXTURE_2D);
}

void system_resize(int width, int height)
{
    float a = (float)width / (float)height;

    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    if (a < 3. / 2) {
        glOrtho(0, 3, 0, 3 / a, -1, 1);
    } else {
        glOrtho(0, 2 * a, 0, 2, -1, 1);
    }

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    window_width = width;
    window_height = height;
}

void display()
{
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Do a rotation around the table center, to convert OpenGL to CVRA convention
    glRotatef(180, 0., 0., 1.);
    glTranslatef(-3., -2., 0.);

    for (Renderable* r : *renderables) {
        r->render();
    }

    glutSwapBuffers();
}

void on_timer(int /*value*/)
{
    glutPostRedisplay();
    glutTimerFunc(33, on_timer, 0);
}

void on_mouse(int button, int state, int x, int y)
{
    if (button != GLUT_LEFT_BUTTON || state != GLUT_DOWN) {
        return;
    }

    float opp_x = 3. - 3. * x / window_width;
    float opp_y = 2. * y / window_height;
    opponent_set_position(opp_x, opp_y);
}

void viewer_init(int argc, char** argv)
{
    glutInit(&argc, argv);
    glutInitWindowSize(600, 400);
    window_width = 600;
    window_height = 400;
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
    glutCreateWindow("CVRA");

    glutDisplayFunc(display);
    glutMouseFunc(on_mouse);
    glutTimerFunc(33, on_timer, 0);
    glutReshapeFunc(system_resize);
}

void startRendering(std::vector<Renderable*>* r)
{
    renderables = r;

    glutMainLoop();
}
