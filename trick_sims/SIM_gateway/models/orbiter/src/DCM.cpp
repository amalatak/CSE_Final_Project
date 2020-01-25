#include "../include/DCM.hh"
#include <stddef.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>


using std::cout;
using std::endl;


void DCM::calculate_chaser_frame(double pos_rel[3], double velocity[3], double chaser_i[3][3]) {
    /*
        This function calculates the "chaser frame" from relative position,
        chaser position and velocity and has axes aligned such that:
        x points to the target
        y completes the right handed system
        z is perpendicular to the orbit plane along the LVLH z

    */

    estimator.TRIAD(pos_rel, velocity, chaser_i);

}

void DCM::calculate_body_chaser(double chaser_frame[3][3], double Ti2b[3][3], double body_chaser[3][3]) {
    /*
        Gets body relative to chaser frame
        T_chaser2b = Ti2b*transpose(Ti2chaser)
    */

    double Ti2chaser_transpose[3][3];
    utility.transpose(chaser_frame, Ti2chaser_transpose);
    utility.matmul(Ti2b, Ti2chaser_transpose, body_chaser);
}

void DCM::calculate_LVLH_i(double position[3], double velocity[3], double LVLH_i[3][3]) {

    /* 

    *** VERIFIED AGAINST MATLAB ***

    Algorithm to get LVLH DCM from velocity and position via TRIAD
    In this case, the LVLH frame has the axes aligned such that
    x is center pointing
    y completes the right handed system along the velocity component
    z is perpendicular to the orbit plane

    */

    double x_dir[3];

    x_dir[0] = -position[0];
    x_dir[1] = -position[1];
    x_dir[2] = -position[2];
    
    estimator.TRIAD(x_dir, velocity, LVLH_i);

}

void DCM::calculate_body_LVLH(double LVLH_i[3][3], double body_i[3][3], double body_lvlh[3][3]) {
    /* 

    *** VERIFIED AGAINST MATLAB ***

    Gets body relative to LVLH frame
    T_lvlh2b = Ti2b*transpose(Ti2lvlh)
    */

    double Ti2lvlh_transpose[3][3];
    utility.transpose(LVLH_i, Ti2lvlh_transpose);
    utility.matmul(body_i, Ti2lvlh_transpose, body_lvlh);

}

void DCM::T1(double angle, double T1_mat[3][3]) {
    /*
        T1 transformation around the x-axis

        T1 = [1      0      0
              0  cos(a) sin(a)
              0 -sin(a) cos(a)]
    */

    T1_mat[0][0] = 1.0; T1_mat[0][1] = 0.0;        T1_mat[0][2] = 0.0; 
    T1_mat[1][0] = 0.0; T1_mat[1][1] = cos(angle); T1_mat[1][2] = sin(angle); 
    T1_mat[2][0] = 0.0; T1_mat[2][1] =-sin(angle); T1_mat[2][2] = cos(angle);
}

void DCM::T2(double angle, double T2_mat[3][3]) {
    /*
        T2 transformation around the y-axis

        T2 = [cos(a) 0 -sin(a)
              0      1      0
              sin(a) 0  cos(a)]
    */
    T2_mat[0][0] = cos(angle); T2_mat[0][1] = 0.0; T2_mat[0][2] = -sin(angle); 
    T2_mat[1][0] = 0.0;        T2_mat[1][1] = 1.0; T2_mat[1][2] = 0.0; 
    T2_mat[2][0] = sin(angle); T2_mat[2][1] = 0.0; T2_mat[2][2] = cos(angle);
}

void DCM::T3(double angle, double T3_mat[3][3]) {
    /*
        T3 transformation around the z-axis

        T3 = [cos(a) sin(a) 0
             -sin(a) cos(a) 0
              0      0      1 ]        
    */
    T3_mat[0][0] = cos(angle); T3_mat[0][1] = sin(angle); T3_mat[0][2] = 0.0; 
    T3_mat[1][0] =-sin(angle); T3_mat[1][1] = cos(angle); T3_mat[1][2] = 0.0; 
    T3_mat[2][0] = 0.0;        T3_mat[2][1] = 0.0;        T3_mat[2][2] = 1.0; 
}