/*************************************************************************
PURPOSE: ( Orbiter Attitude Model )
**************************************************************************/
#ifndef ATTITUDE_H
#define ATTITUDE_H

//#include "orbiter.hh"
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
    double pos_rel[3];                 // -- relative position vector (for chaser use)

    // DCM
    double body_i[3][3];              // -- body attitude relative to inertial frame
    double body_lvlh[3][3];           // -- body attitude relative to lvlh frame
    double LVLH_i[3][3];              // -- lvlh frame relative to intertial frame
    double body_to_LVLH[3][3];        // -- body to lvlh transformation (or rotation) matrix
    double chaser_i[3][3];            // -- chaser attitude frame relative to inertial frame
    double body_chaser[3][3];         // -- body attitude relative to the chaser frame

    // QUATERNIONS
    double qdes[4];                   /* -- Desired quaternion attitude of the satellite body with respect to the inertial frame */
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


public:
    double w_body_b[3];               /* rad/s angular velocity of the body frame in the inertial frame coordinatized in the body frame */
    double wdot_b_b[3];               /* rad/s^2 angular acceleration of body relative to inertial frame coordinatized in the body frame */
    double qest[4];                   /* -- Quaternion attitude of the satellite body with respect to the inertial frame */
    double q_dot[4];                  /* -- Quaternion attitude rate of the satellite body with respect to the inertial frame */

    /* UTILITIES */
    void print_mat(double mat[3][3]);
    void print_quat(double quat[4]);
    void print_vec(double vec[3]);
    void print_qerr();
    void set_vec(double vec1, double vec2, double vec3, double vec[3]);
    void quat2DCM(double quat[4], double DCM[3][3]); 
    void DCM2quat(double DCM[3][3], double quat[4]); // UNVARIFIED

    /* CONSTANTS -- VALIDATED */
    void set_Jmat(double J_00, double J_01, double J_02, 
                  double J_10, double J_11, double J_12, 
                  double J_20, double J_21, double J_22);
    void set_Jmat_inv();

    /* DCM OPERATIONS -- VALIDATED*/
    void set_body_i(double body_0_0, double body_0_1, double body_0_2, 
                    double body_1_0, double body_1_1, double body_1_2, 
                    double body_2_0, double body_2_1, double body_2_2);
    void calculate_chaser_frame();
    void calculate_body_chaser();
    void calculate_LVLH_i();
    void calculate_body_LVLH();

    /* QUATERNION OPERATIONS */
    void set_qest(double DCM[3][3], double angle_offset, double e_axis[3]);           // verified when DCM2quat if varified
    void qmult(double q1[], double q2[], double q3[]);              // VALIDATED
    void q_conjugate(double q[], double qconj[]);                   // VALIDATED
    void calculate_qdot();                                          // VALIDATED
    void calculate_quaternion_error();
    void calculate_euler_error();
    void calculate_euler_error_rate();

    /* KINEMATICS */
    void set_torques(double torque_set1, double torque_set2, double torque_set3);
    void set_rv(double pos[], double vel[]);
    void set_pos_rel(double pos[]);
    void set_w_body_b(double w_b[3]);
    void calculate_wdot_body_bwrti();

    /* CONTROLS */
    void set_gains(double K_p, double K_i, double K_d);
    void set_qdes_center_pointing();
    void set_wdes_center_pointing();
    void set_qdes_target_pointing();
    void set_wdes_target_pointing();
    void PD();

    /* ESTIMATION */
    void TRIAD();
    void camera();

    /* TOP LEVEL SCRIPTS */
    void target_initialize();
    void target_dynamics_update(double pos[], double vel[]);
    void chaser_initialize();
    void chaser_dynamics_update(double pos[], double vel[], double target_pos[]);
};

#ifdef __cplusplus
}
#endif

#endif

