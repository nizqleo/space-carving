# The simplest application example: 20 lines of code and yet all the power !

# A Viewer class is derived from QGLViewer and its <code>draw()</code> function is overloaded to
# specify the user defined OpenGL orders that describe the scene.

# This is the first example you should try, as it explains some of the default keyboard shortcuts
# and the mouse behavior of the viewer.

# This example can be cut and pasted to start the development of a new application.

TARGET = myViewer
CONFIG *= qt opengl release
QT *= opengl xml

HEADERS = Voxels.hpp Viewer.hpp
SOURCES = Voxels.cpp Viewer.cpp main.cpp

# Linux
INCLUDEPATH *= /usr/include
LIBS *= -L/usr/local/lib -lQGLViewer-qt5

DEFINES += "ENABLE_PRECOMPILED_HEADERS=OFF"

INCPATH -= -isystem /usr/include