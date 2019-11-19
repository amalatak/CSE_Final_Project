#include "../include/control.hh"
#include <stddef.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>

using std::cout;
using std::endl;

void control::set_gains(double K_p, double K_i, double K_d) {
    Kp = K_p;
    Ki = K_i;
    Kd = K_d;
}

void control::set_qdes_center_pointing(double LVLH_i[3][3], double qdes[4]) {
    /* 
        This function sets the desired quaternion based on current position
        to be pointing at the center of the main orbiting body. 
    */
    quat_util.DCM2quat(LVLH_i, qdes);
}

void control::set_wdes_center_pointing(double position[3], double velocity[3], double quat_body[4], double wdes_b[3]) {
    /* 

        *** CALCULATES CORRECTLY, VALIDATE ALGORITHM

        This function sets the desired angular velocity based on current position
        and velocity in order to maintain pointing at the center of the orbiting body
    */
    double rsq, rxv[3], Ti2b[3][3];
    rsq = position[0]*position[0] + position[1]*position[1] + position[2]*position[2];
    utility.cross(position, velocity, rxv);

    w_ref_i[0] = rxv[0]/rsq;
    w_ref_i[1] = rxv[1]/rsq;
    w_ref_i[2] = rxv[2]/rsq;

    quat_util.quat2DCM(quat_body, Ti2b);  // should this be state or estimate

    utility.matvecmul(Ti2b, w_ref_i, wdes_b);

}

void control::set_qdes_target_pointing(double chaser_frame[3][3], double qdes[4]) {
    /* 
        This function sets the desired quaternion based on current position
        to be pointing at the center of the target. 
    */
    quat_util.DCM2quat(chaser_frame, qdes);
}

void control::set_wdes_target_pointing(double pos_rel[3], double vel_rel[3], double quat_body[4], double wdes_b[3]) {
    /* 
        This function sets the desired angular velocity as 

        rel_pos x rel_vel / |rel_pos|^2

        then transforms it into the body frame
    */
    double rsq, rxv_rel[3], Ti2b[3][3];

    rsq = pos_rel[0]*pos_rel[0] + pos_rel[1]*pos_rel[1] + pos_rel[2]*pos_rel[2];
    utility.cross(pos_rel, vel_rel, rxv_rel);

    w_ref_i[0] = rxv_rel[0]/rsq;
    w_ref_i[1] = rxv_rel[1]/rsq;
    w_ref_i[2] = rxv_rel[2]/rsq;

    quat_util.quat2DCM(quat_body, Ti2b);  // should this be state or estimate

    utility.matvecmul(Ti2b, w_ref_i, wdes_b);
}

void control::PD(double eul_err[3], double eul_err_rate[3], double desired_out[3]) {
    /*
        This function issues a desired control output for a PD controller
    */
    desired_out[0] = -Kp*eul_err[0] - Kd*eul_err_rate[0];
    desired_out[1] = -Kp*eul_err[1] - Kd*eul_err_rate[1];
    desired_out[2] = -Kp*eul_err[2] - Kd*eul_err_rate[2];
}

