#include "../include/estimation.hh"
#include <stddef.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>

using std::cout;
using std::endl;


/* ESTIMATION */
void estimation::TRIAD(double vec1[3], double vec2[3], double frame_est[3][3]) {
    /* 
        This algorithm calculates a reference frame given two measurement vectors
        by establishing a primary vector, vec1 and a plane by crossing the two to 
        get a third vector normal. Then the second vector is crossed with the third
        to get the completed frame which is created as such
        vx = v1
        vz = v1xv2
        vy = vzxvx
        DCM = [vx'
               vy'
               vz']
    */

    double vx[3], vy[3], vz[3], vz_not_norm[3];

    vx[0] = vec1[0]/sqrt(vec1[0]*vec1[0] + vec1[1]*vec1[1] + vec1[2]*vec1[2]) ;
    vx[1] = vec1[1]/sqrt(vec1[0]*vec1[0] + vec1[1]*vec1[1] + vec1[2]*vec1[2]) ;
    vx[2] = vec1[2]/sqrt(vec1[0]*vec1[0] + vec1[1]*vec1[1] + vec1[2]*vec1[2]) ;

    utility.cross(vx, vec2, vz_not_norm);
    vz[0] = vz_not_norm[0]/sqrt(vz_not_norm[0]*vz_not_norm[0] + vz_not_norm[1]*vz_not_norm[1] + vz_not_norm[2]*vz_not_norm[2]) ;
    vz[1] = vz_not_norm[1]/sqrt(vz_not_norm[0]*vz_not_norm[0] + vz_not_norm[1]*vz_not_norm[1] + vz_not_norm[2]*vz_not_norm[2]) ;
    vz[2] = vz_not_norm[2]/sqrt(vz_not_norm[0]*vz_not_norm[0] + vz_not_norm[1]*vz_not_norm[1] + vz_not_norm[2]*vz_not_norm[2]) ;

    utility.cross(vz, vx, vy);
    frame_est[0][0] = vx[0]; frame_est[0][1] = vx[1]; frame_est[0][2] = vx[2];
    frame_est[1][0] = vy[0]; frame_est[1][1] = vy[1]; frame_est[1][2] = vy[2];
    frame_est[2][0] = vz[0]; frame_est[2][1] = vz[1]; frame_est[2][2] = vz[2];
}

void estimation::camera() {
    /*  
        This function uses a simulated camera to determine/estimate the unit vector pointing
        from the chaser to the target. In mathematical terms, we estimate the x_hat vector.
    */
}