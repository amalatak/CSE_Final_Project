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

    // DCM
    double body_i[3][3];              // -- body attitude relative to inertial frame
    double body_lvlh[3][3];           // -- body attitude relative to lvlh frame
    double LVLH_i[3][3];              // -- lvlh frame relative to intertial frame
    double body_to_LVLH[3][3];        // -- body to lvlh transformation (or rotation) matrix

    // QUATERNIONS
    double quat[4];                   /* -- Quaternion attitude of the rocket body with respect to the inertial frame */
    double q_dot[4];                  /* -- Quaternion attitude rate of the rocket body with respect to the inertial frame */
    double q_err[4];                  /* -- Quaternion error relative to the desired quaternion */
    double eul_er_est[4];             /* -- Small angle approximation for euler angle error */
    
    // KINEMATICS
    double torques[3];                // -- Summation of torques acting on body
    double w_body_i[3];               /* rad/s angular velocity of the body frame in the inertial frame coordinatized in the inertial frame */
    double w_body_b[3];               /* rad/s angular velocity of the body frame in the inertial frame coordinatized in the body frame */
    double wdot_b_i[3];               /* rad/s^2 angular acceleration of body relative to inertial frame coordinatized in the inertial frame */
    double wdot_b_b[3];               /* rad/s^2 angular acceleration of body relative to inertial frame coordinatized in the body frame */

    // CONTROL VALUES
    double w_ref[3];
    double att_ref[3][3];                               


public:
    /* CONSTANTS */
    void set_Jmat(double J_00, double J_01, double J_02, 
                  double J_10, double J_11, double J_12, 
                  double J_20, double J_21, double J_22);
    void set_Jmat_inv();

    /* DCM OPERATIONS */
    void set_body_i(double body_0_0, double body_0_1, double body_0_2, 
                    double body_1_0, double body_1_1, double body_1_2, 
                    double body_2_0, double body_2_1, double body_2_2);
    void calculate_body_LVLH();
    void calculate_LVLH_i();

    /* QUATERNION OPERATIONS */
    void qmult(double q1[], double q2[], double q3[]);
    void q_conjugate(double q[], double qconj[]);
    void calculate_qdot(double w[], double q[], double qdot[]);
    void calculate_quaternion_error(double q_est[], double q_des[], double dq[]);
    void calculate_euler_error(double dq[], double eul_er[]);

    /* KINEMATICS */
    void set_torques(double torque_set []);
    void set_rv(double pos[], double vel[]);
    void calculate_wdot_body_bwrti();
};

#ifdef __cplusplus
}
#endif

#endif

