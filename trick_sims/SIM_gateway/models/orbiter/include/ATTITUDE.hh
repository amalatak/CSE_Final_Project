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
    // Mass properties
    double Jmat[3][3];                // -- Inertial tensor of orbiter
    double Jmat_inv[3][3];            // -- Inverse of the Inertial tensor of orbiter

    // Translational Variables
    double position[3];               // m position of spacecraft
    double velocity[3];               // m/s velocity of spacecraft 
    double rv[6];                     // -- position-velocity vector
    double pos_rel[3];                // -- relative position vector (for chaser use)
    double vel_rel[3];                // -- relative velocity vector (for chaser use)

    // DCM
    double body_i[3][3];              // -- body attitude relative to inertial frame
    double body_lvlh[3][3];           // -- body attitude relative to lvlh frame
    double LVLH_i[3][3];              // -- lvlh frame relative to intertial frame
    double body_to_LVLH[3][3];        // -- body to lvlh transformation (or rotation) matrix
    double body_chaser[3][3];         // -- body attitude relative to the chaser frame

    // QUATERNIONS
    double q_est[4];                  /* -- Estimated quaternion attitude of the satellite body with respect to the inertial frame */
    double q_des[4];                   /* -- Desired quaternion attitude of the satellite body with respect to the inertial frame */
    double q_err[4];                  /* -- Quaternion error relative to the desired quaternion */
    double eul_er_est[3];             /* -- Small angle approximation for euler angle error */
    double eul_er_rate_est[3];
    
    // KINEMATICS
    double torques[3];                // -- Summation of torques acting on body
    double w_body_i[3];               /* rad/s angular velocity of the body frame in the inertial frame coordinatized in the inertial frame */
    double wdot_b_i[3];               /* rad/s^2 angular acceleration of body relative to inertial frame coordinatized in the inertial frame */

    // CONTROL VALUES
    double w_ref_i[3];
    double w_ref_b[3];
    double att_ref[3][3]; 
    double control_out[3];            /* -- Controller output */
    double Kp;                        /* -- Proportional Gain */
    double Ki;                        /* -- Integral Gain */
    double Kd;                        /* -- Derivative Gain */

    double camera_sensor[3];
    double docking_port_location[3];

    estimation estimator;
    quaternion quat_util;
    DCM dcm;
    sensors sensor;
    UTILITIES utility;

public:
    double w_body_b[3];               /* rad/s angular velocity of the body frame in the inertial frame coordinatized in the body frame */
    double wdot_b_b[3];               /* rad/s^2 angular acceleration of body relative to inertial frame coordinatized in the body frame */
    double q_state[4];                /* -- Quaternion attitude of the satellite body with respect to the inertial frame */
    double q_dot[4];                  /* -- Quaternion attitude rate of the satellite body with respect to the inertial frame */
    double chaser_i[3][3];            // -- chaser attitude frame relative to inertial frame
    double target_i[3][3];            // -- target attitude frame relative to the inertial frame, used for chaser calculations
    control controller;

    /* PRINT UTILITY */
    void print_qerr();

    /* Set utilities */
    void set_Jmat(double J_00, double J_01, double J_02, 
                  double J_10, double J_11, double J_12, 
                  double J_20, double J_21, double J_22);
    void set_Jmat_inv();
    void set_body_i(double body_0_0, double body_0_1, double body_0_2, 
                    double body_1_0, double body_1_1, double body_1_2, 
                    double body_2_0, double body_2_1, double body_2_2);

    /* KINEMATICS */
    void set_rv(double pos[], double vel[]);
    void set_pos_vel_rel(double pos[], double vel[]);
    void calculate_wdot_body_bwrti();

    /* TOP LEVEL SCRIPTS */
    void estimate_attitude();
    // void attitude_control();
    void target_initialize();
    void target_dynamics_update(double pos[], double vel[]);
    void chaser_initialize();
    void chaser_dynamics_update(double pos[], double vel[], double target_pos[], double target_vel[], double target_attitude[3][3]);
};

#ifdef __cplusplus
}
#endif

#endif

