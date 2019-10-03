#include <iostream>
#include <fstream>
#include <vector>
#include <memory.h>

#include "Voxels.hpp"

using namespace std;


void cube::draw(){  
    for (int i = 0; i < 6; ++i) {
        glBegin(GL_QUADS);
        glColor3f(r, g, b);
        float norm[3];
        norm[0] =norm[1]= norm[2]= 0.0f;
        norm[i/2] = (i%2)? -1: 1;
        glNormal3f(norm[0], norm[1], norm[2]);

        int skip = 4>>(i/2); //0:4 1:2 2:1
        int p = (i%2)*skip;
        int cnt = 0;
        int mem_p = -1;
        while(p < 8){
            for(int j = 0; j < skip; j++, p++){
                if(cnt == 2){
                    mem_p = p;
                    cnt++;
                    continue;
                }
                glVertex3f(points[p].x, points[p].y, points[p].z);
                cnt++;
            }
            p += skip;
        }
        glVertex3f(points[mem_p].x, points[mem_p].y, points[mem_p].z);
        glEnd();
    }
}


void Voxels::draw(){

    for(int i = 0; i < xnum; i++){
        for(int j = 0; j < ynum; j++){
            for(int k = 0; k < znum; k++){
                if(voxel[i][j][k].occupied)
                    voxel[i][j][k].draw();
            }
        }
    }
}

