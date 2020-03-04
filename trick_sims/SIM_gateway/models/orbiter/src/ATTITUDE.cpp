#include "../include/orbiter.hh"
#include "../include/quaternion.hh"
#include "../include/ATTITUDE.hh"
#include "../include/estimation.hh"
#include <stddef.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>

using std::cout;
using std::endl;

void ATTITUDE::calculate_chaser_frame(double pos_rel[3], double velocity[3], double chaser_i[3][3]) {
    /*
        This function calculates the "chaser frame" from relative position,
        chaser position and velocity and has axes aligned such that:
        x points to the target
        y completes the right handed system, pointing away from the frame center
        z is perpendicular to the orbit plane along the LVLH z, perpendicular to
            the orbit plane

    */

    estimator.TRIAD(pos_rel, velocity, chaser_i);
}


void ATTITUDE::calculate_LVLH_i(double position[3], double velocity[3], double LVLH_i[3][3]) {
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

/********************************************************************
KINEMATICS
********************************************************************/


void ATTITUDE::calculate_wdot_body_bwrti(double torque[3], double J_mat[3][3], double J_inv[3][3], double w_b[3], double wdot_b[3]) {
    /*
    Calculate rate of change of angular velocity of a spacecraft body with respect to the inertial frame
    coordinatized in the body frame 

    wdot*J = sum(M), so
    wdot = J^(-1)*(sum(M) - cross(w_body_bwrti, Jmat*w_body_bwrti))

    */

    double w_cross[3], Jw[3], t_sub_wcross[3];

    utility.matvecmul(J_mat, w_b, Jw);
    utility.cross(w_b, Jw, w_cross);

    t_sub_wcross[0] = torque[0] - w_cross[0];
    t_sub_wcross[1] = torque[1] - w_cross[1];
    t_sub_wcross[2] = torque[2] - w_cross[2];

    utility.matvecmul(J_inv, t_sub_wcross, wdot_b);

}

