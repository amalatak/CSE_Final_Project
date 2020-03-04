/*************************************************************************
PURPOSE: ( Orbiter Quaternion Library Model )
**************************************************************************/
#ifndef QUATERNION_H
#define QUATERNION_H

#include "UTILITIES.hh"
#include <iostream>
#include <vector>
#include <math.h>


#ifdef __cplusplus
extern "C" {
#endif


class quaternion {
private:

    UTILITIES utility;

public:

    /* UTILITIES */
    void quat2DCM(double quat[4], double DCM[3][3]); 
    void DCM2quat(double DCM[3][3], double quat[4]); 

    /* QUATERNION OPERATIONS */
    void set_q_from_DCM(double DCM[3][3], double e_axis1, double e_axis2, double e_axis3, double angle_offset_rad, double quat[4]);
    void qmult(double q1[], double q2[], double q3[]);
    void q_conjugate(double q[], double qconj[]);
    void calculate_qdot(double w_body[3], double quat[4], double qrate[4]);
    void calculate_quaternion_error(double quat[4], double qdes[4], double qerr[4]);
    void calculate_euler_error(double qerr[4], double eul_er[3]);
    void calculate_euler_error_rate(double w_body[3], double w_body_ref[3], double eul_er[3], double eul_er_rate[3]);

};

#ifdef __cplusplus
}
#endif

#endif

