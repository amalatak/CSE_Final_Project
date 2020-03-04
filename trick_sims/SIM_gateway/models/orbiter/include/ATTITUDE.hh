/*************************************************************************
PURPOSE: ( Orbiter Attitude Model )
**************************************************************************/
#ifndef ATTITUDE_H
#define ATTITUDE_H

#include "estimation.hh"
#include "control.hh"
#include "sensors.hh"
#include "quaternion.hh"
#include "UTILITIES.hh"
#include "mass_geometry.hh"
#include "DCM.hh"
#include <iostream>
#include <vector>
#include <math.h>


using namespace std;

#ifdef __cplusplus
extern "C" {
#endif


class ATTITUDE {
private:

    // QUATERNIONS
    double q_est[4];                  /* -- Estimated quaternion attitude of the satellite body with respect to the inertial frame */
    double q_des[4];                  /* -- Desired quaternion attitude of the satellite body with respect to the inertial frame */
    double q_err[4];                  /* -- Quaternion error relative to the desired quaternion */

    UTILITIES utility;
    

public:
    sensors sensor;
    control controller;
    mass_geometry physical_properties;
    estimation estimator;
    quaternion quat_util;

    double LVLH[3][3];                // -- LVLH frame of s/c
    double Chaser_Frame[3][3];        // -- Chaser frame composed of x-axis to target, z out of plane || to LVLH, y completes system
    double body_i[3][3];              // -- DCM attitude of the s/c
    
    double q_state[4];                /* -- Quaternion attitude of the satellite body with respect to the inertial frame */
    double q_dot[4];                  /* 1/s Quaternion attitude rate of the satellite body with respect to the inertial frame */

    double w_body_b[3];               /* rad/s angular velocity of the body frame in the inertial frame coordinatized in the body frame */
    double wdot_b_b[3];               /* rad/s^2 angular acceleration of body relative to inertial frame coordinatized in the body frame */

    double r_camera_to_dock[3];       // m position of the docking port relative to the camera
    double r_camera_to_dock_rate[3];  // m rate of change of the position of the docking port wrt the camera

    /* Functions */
    void calculate_chaser_frame(double pos_rel[3], double velocity[3], double chaser_i[3][3]);
    void calculate_LVLH_i(double position[3], double velocity[3], double LVLH_i[3][3]);
    void calculate_wdot_body_bwrti(double torque[3], double J_mat[3][3], double J_inv[3][3], double w_b[3], double wdot_b[3]);

};

#ifdef __cplusplus
}
#endif

#endif

