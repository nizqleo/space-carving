#include "Viewer.hpp"
#include <math.h>
#include <QGLViewer/manipulatedFrame.h>

using namespace std;
using namespace qglviewer;
    // Draws a spiral
void Viewer::draw() {
    float pos[4] = {1.0, 0.5, 1.0, 0.0};
    // Directionnal light
    glLightfv(GL_LIGHT0, GL_POSITION, pos);

    pos[3] = 1.0;
    // Spot light
    Vec pos1 = light1->position();
    pos[0] = float(pos1.x);
    pos[1] = float(pos1.y);
    pos[2] = float(pos1.z);
    glLightfv(GL_LIGHT1, GL_POSITION, pos);
    glLightfv(GL_LIGHT1, GL_SPOT_DIRECTION,
            light1->inverseTransformOf(Vec(0, 0, 1)));

    // Point light
    Vec pos2 = light2->position();
    pos[0] = float(pos2.x);
    pos[1] = float(pos2.y);
    pos[2] = float(pos2.z);
    glLightfv(GL_LIGHT2, GL_POSITION, pos);


    content->draw();
        
    drawLight(GL_LIGHT0);

    if (light1->grabsMouse())
        drawLight(GL_LIGHT1, 1.2f);
    else
        drawLight(GL_LIGHT1);

    if (light2->grabsMouse())
        drawLight(GL_LIGHT2, 1.2f);
    else
        drawLight(GL_LIGHT2);

}

void Viewer::init() {
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Light0 is the default ambient light
    glEnable(GL_LIGHT0);

    // Light1 is a spot light
    glEnable(GL_LIGHT1);
    const GLfloat light_ambient[4] = {1.0f, 1.0f, 1.0f, 1.0};
    const GLfloat light_diffuse[4] = {1.0, 0.4f, 0.4f, 1.0};
    const GLfloat light_specular[4] = {1.0, 0.0, 0.0, 1.0};

    glLightf(GL_LIGHT1, GL_SPOT_EXPONENT, 3.0);
    glLightf(GL_LIGHT1, GL_SPOT_CUTOFF, 20.0);
    glLightf(GL_LIGHT1, GL_CONSTANT_ATTENUATION, 0.5);
    glLightf(GL_LIGHT1, GL_LINEAR_ATTENUATION, 1.0);
    glLightf(GL_LIGHT1, GL_QUADRATIC_ATTENUATION, 1.5);
    glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT1, GL_SPECULAR, light_specular);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, light_diffuse);

    // Light2 is a classical directionnal light
    glEnable(GL_LIGHT2);
    const GLfloat light_ambient2[4] = {0.2f, 0.2f, 2.0, 1.0};
    const GLfloat light_diffuse2[4] = {0.8f, 0.8f, 1.0, 1.0};
    const GLfloat light_specular2[4] = {0.0, 0.0, 1.0, 1.0};

    glLightfv(GL_LIGHT2, GL_AMBIENT, light_ambient2);
    glLightfv(GL_LIGHT2, GL_SPECULAR, light_specular2);
    glLightfv(GL_LIGHT2, GL_DIFFUSE, light_diffuse2);

    light1 = new ManipulatedFrame();
    light2 = new ManipulatedFrame();
    setMouseTracking(true);

    light1->setPosition(5, 5, 5);
    // Align z axis with -position direction : look at scene center
    light1->setOrientation(Quaternion(Vec(0, 0, 1), -light1->position()));

    light2->setPosition(-5, -5, -5);





    restoreStateFromFile();
    help();

    content = new Voxels(file_name);

}

QString Viewer::helpString() const {
  QString text("<h2>S i m p l e V i e w e r</h2>");
  text += "Use the mouse to move the camera around the object. ";
  text += "You can respectively revolve around, zoom and translate with the "
          "three mouse buttons. ";
  text += "Left and middle buttons pressed together rotate around the camera "
          "view direction axis<br><br>";
  text += "Pressing <b>Alt</b> and one of the function keys "
          "(<b>F1</b>..<b>F12</b>) defines a camera keyFrame. ";
  text += "Simply press the function key again to restore it. Several "
          "keyFrames define a ";
  text += "camera path. Paths are saved when you quit the application and "
          "restored at next start.<br><br>";
  text +=
      "Press <b>F</b> to display the frame rate, <b>A</b> for the world axis, ";
  text += "<b>Alt+Return</b> for full screen mode and <b>Control+S</b> to save "
          "a snapshot. ";
  text += "See the <b>Keyboard</b> tab in this window for a complete shortcut "
          "list.<br><br>";
  text += "Double clicks automates single click actions: A left button double "
          "click aligns the closer axis with the camera (if close enough). ";
  text += "A middle button double click fits the zoom of the camera and the "
          "right button re-centers the scene.<br><br>";
  text += "A left button double click while holding right button pressed "
          "defines the camera <i>Revolve Around Point</i>. ";
  text += "See the <b>Mouse</b> tab and the documentation web pages for "
          "details.<br><br>";
  text += "Press <b>Escape</b> to exit the viewer.";
  return text;
}
