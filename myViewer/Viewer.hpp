#include <QGLViewer/qglviewer.h>
#include "Voxels.hpp"
using namespace std;

class Viewer : public QGLViewer {
protected:
    virtual void draw();
    virtual void init();
    virtual QString helpString() const;
public:
    Voxels* content;

    void setFile(string name){ file_name = name;}
    
private:
    string file_name;
    qglviewer::ManipulatedFrame *light1;
    qglviewer::ManipulatedFrame *light2;
};
