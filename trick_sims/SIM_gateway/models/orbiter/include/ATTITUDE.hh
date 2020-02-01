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

    // DCM
    double body_i[3][3];              // -- body attitude relative to inertial frame
    double LVLH_i[3][3];              // -- lvlh frame relative to intertial frame

    // QUATERNIONS
    double q_est[4];                  /* -- Estimated quaternion attitude of the satellite body with respect to the inertial frame */
    double q_des[4];                  /* -- Desired quaternion attitude of the satellite body with respect to the inertial frame */
    double q_err[4];                  /* -- Quaternion error relative to the desired quaternion */
    double eul_er_est[3];             /* -- Small angle approximation for euler angle error */
    double eul_er_rate_est[3];
    
    // KINEMATICS
    double target_w_b[3];
    double torques[3];                // -- Summation of torques acting on body

    // CONTROL VALUES
    double w_ref_b[3];
    
    // GEOMETRY and SENSORS
    double camera_location[3];        //
    double docking_port_location[3];  //
    double r_camera_to_dock[3];       //
    double r_camera_to_dock_rate[3];  //
    double camera_sensor[3];          //

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
    void set_target_w_b(double w_target_body_b[3]);
    void calculate_wdot_body_bwrti();

    /* GEOMETRY */
    void calculate_r_camera_to_dock(double target_attitude[3][3], double target_cg[3], 
                                    double chaser_attitude[3][3], double chaser_cg[3], 
                                    double r_camera_2_dock_i[3]);
    void calculate_r_camera_to_dock_rate(double target_vel[3], double target_attitude[3][3], double target_w_b[3], 
                                         double chaser_vel[3], double chaser_attitude[3][3], double chaser_w_b[3],
                                         double r_camera_to_dock_rate_i[3]);

    /* TOP LEVEL SCRIPTS */
    void estimate_attitude();
    // void attitude_control();
    void target_initialize();
    void target_dynamics_update(double pos[], double vel[]);
    void chaser_initialize();
    void chaser_dynamics_update(double pos[], double vel[], double target_pos[], double target_vel[], 
                                double target_attitude[3][3], double w_target_b[3]);
    void top_sol();
};

#ifdef __cplusplus
}
#endif

#endif

