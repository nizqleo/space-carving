#include <iostream>
#include <fstream>
#include <vector>
#include <memory.h>
#include <fstream>
#include <QGLViewer/qglviewer.h>
using namespace std;

class point{
public:
	float x,y,z;
	point(float x, float y, float z):x(x), y(y), z(z){
	}
};

class cube{
public:
    float x,y,z;//center
	float graity;
    bool occupied;
	float r,g,b;
	vector<point> points;

    cube(){}
    cube(float x, float y, float z, float graity, bool occ, float r = 1.0f, float g = 0.3f, float b = 0.0f):x(x), y(y), z(z), graity(graity), r(r), g(g), b(b){
        occupied = occ;
		float unit = 1.0f*graity;
		points.push_back(point(-unit + x, -unit+y, -unit+z));
		points.push_back(point(-unit+x, -unit+y, unit+z));
		points.push_back(point(-unit+x, unit+y, -unit+z));
		points.push_back(point(-unit+x, unit+y, unit+z));
		points.push_back(point(unit+x, -unit+y, -unit+z));
		points.push_back(point(unit+x, -unit+y, unit+z));
		points.push_back(point(unit+x, unit+y, -unit+z));
		points.push_back(point(unit+x, unit+y, unit+z));
    }


	void draw();
};

class Voxels{
public:
    float xmin, xmax, ymin, ymax, zmin, zmax;
    float graity;
	int xnum, ynum, znum;
	bool is_color;
	float scale;
	vector<vector<vector<cube>>> voxel;
    
	Voxels(){}
	Voxels(string file_name){
		ifstream  afile;
		afile.open(file_name, std::ios::in);
		afile>>graity>>xmin>>xmax>>ymin>>ymax>>zmin>>zmax>>is_color;
		cout<<graity<<' '<<xmin<<' '<<xmax<<' '<<ymin<<' '<<ymax<<' '<<zmin<<' '<<zmax<<' '<<is_color<<endl;

		float max_size = -1;
	    znum = int((zmax-zmin)/graity);if((zmax-zmin) > max_size) max_size = (zmax-zmin);
		ynum = int((ymax-ymin)/graity);if((ymax-ymin) > max_size) max_size = (ymax-ymin);
		xnum = int((xmax-xmin)/graity);if((xmax-xmin) > max_size) max_size = (xmax-xmin);

		if(max_size > 3){
			scale = 3/max_size;

			graity*= scale;
			xmin *= scale;
			xmax *= scale;
			ymin *= scale;
			ymax *= scale;
			zmin *= scale;
			zmax *= scale;
		}
		else scale = 1.0;

		vector<cube> vec_temp1;
		vec_temp1.resize(znum);
		vector<vector<cube>> vec_temp2(ynum, vec_temp1);
		voxel.resize(znum, vec_temp2);

		int cnt = 0;
		int bool_temp;
		int r,g,b;
		for(int i = 0; i < xnum; i++){
			for(int j = 0; j < ynum; j++){
				for(int k = 0; k < znum; k++){
					afile>>bool_temp;
					if (is_color){
						afile>>r>>g>>b;
						cube cube_instance(xmin + i*graity, ymin + j*graity, zmin + k*graity, graity, bool_temp, r,g,b);
						voxel[i][j][k] = cube_instance;
					}
					else {
						cube cube_instance(xmin + i*graity, ymin + j*graity, zmin + k*graity, graity, bool_temp);	
						voxel[i][j][k] = cube_instance;	
					}
					if(bool_temp)
						cnt++;
				}
			}
		}
		

		cout<<xnum<<' '<<ynum<<' '<<znum<<' '<<cnt<<endl;
	}

	void draw();
};