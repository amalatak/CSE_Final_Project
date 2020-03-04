#include "../include/quaternion.hh"
#include <stddef.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>

#include "../../../../dependencies/Eigen/Dense"
#include "../../../../dependencies/Eigen/Eigen"
using std::cout;
using std::endl;

using namespace Eigen;


void quaternion::quat2DCM(double quat[4], double DCM[3][3]) {
    /*
        *** VERIFIED AGAINST MATLAB ***

        This function calculates the direction cosine matrix (DCM)
        from a given quaternion where q[3] is the scalar element.

        DCM = [q1^2-q2^2-q3^2+q4^2, 2*(q1*q2+q3*q4),      2*(q1*q3-q2*q4);
               2*(q1*q2-q3*q4),     -q1^2+q2^2-q3^2+q4^2, 2*(q2*q3+q1*q4);
               2*(q1*q3+q2*q4),     2*(q2*q3-q1*q4),      -q1^2-q2^2+q3^2+q4^2 ];

    */
    DCM[0][0] = quat[0]*quat[0] - quat[1]*quat[1] - quat[2]*quat[2] + quat[3]*quat[3];
    DCM[0][1] = 2*(quat[0]*quat[1] + quat[2]*quat[3]);
    DCM[0][2] = 2*(quat[0]*quat[2] - quat[1]*quat[3]);

    DCM[1][0] = 2*(quat[0]*quat[1] - quat[2]*quat[3]);
    DCM[1][1] = -quat[0]*quat[0] + quat[1]*quat[1] - quat[2]*quat[2] + quat[3]*quat[3];
    DCM[1][2] = 2*(quat[1]*quat[2] + quat[0]*quat[3]);

    DCM[2][0] = 2*(quat[0]*quat[2] + quat[1]*quat[3]);
    DCM[2][1] = 2*(quat[1]*quat[2] - quat[0]*quat[3]);
    DCM[2][2] = -quat[0]*quat[0] - quat[1]*quat[1] + quat[2]*quat[2] + quat[3]*quat[3];
}

void quaternion::DCM2quat(double DCM[3][3], double quat[4]) {
    /*

        *** VERIFIED AGAINST MATLAB (may be negative in absolute terms) ***

        This function calculates the quaternion attitude vector
        from a given DCM where q[3] is the scalar element.

        K3 = [Q(1,1)-Q(2,2)-Q(3,3), Q(2,1)+Q(1,2), Q(3,1)+Q(1,3), Q(2,3)-Q(3,2)
              Q(2,1)+Q(1,2), Q(2,2)-Q(1,1)-Q(3,3), Q(3,2)+Q(2,3), Q(3,1)-Q(1,3)
              Q(3,1)+Q(1,3), Q(3,2)+Q(2,3), Q(3,3)-Q(1,1)-Q(2,2), Q(1,2)-Q(2,1)
              Q(2,3)-Q(3,2), Q(3,1)-Q(1,3), Q(1,2)-Q(2,1), Q(1,1)+Q(2,2)+Q(3,3)]/3;
        [eigvec, eigval] = eig(K3)
        x,index = max(eigval)
        q = eigvec(:,i);
    */
    MatrixXd K3(4, 4);
    
    K3 << DCM[0][0]-DCM[1][1]-DCM[2][2], DCM[1][0]+DCM[0][1], DCM[2][0]+DCM[0][2], DCM[1][2]-DCM[2][1],
          DCM[1][0]+DCM[0][1], DCM[1][1]-DCM[0][0]-DCM[2][2], DCM[2][1]+DCM[1][2], DCM[2][0]-DCM[0][2],
          DCM[2][0]+DCM[0][2], DCM[2][1]+DCM[1][2], DCM[2][2]-DCM[0][0]-DCM[1][1], DCM[0][1]-DCM[1][0],
          DCM[1][2]-DCM[2][1], DCM[2][0]-DCM[0][2], DCM[0][1]-DCM[1][0], DCM[0][0]+DCM[1][1]+DCM[2][2];
    K3/=3;

    EigenSolver<MatrixXd> es(K3);

    int i, max_i;
    double max_eigenvalue = -1.0;
    for (i=0; i<4; i++) {
        if (es.eigenvalues()[i].real() > max_eigenvalue) {
            max_eigenvalue = es.eigenvalues()[i].real();
            max_i = i;
        }
    }

    quat[0] = es.eigenvectors()(0, max_i).real();
    quat[1] = es.eigenvectors()(1, max_i).real();
    quat[2] = es.eigenvectors()(2, max_i).real();
    quat[3] = es.eigenvectors()(3, max_i).real();

}


/********************************************************************
QUATERNION OPERATIONS

This library assumes that the scalar element of the quaternion is q[3]
and that the quaternion is defined as [e_vec*sin(theta/2); cos(theta/2)]
********************************************************************/

void quaternion::set_q_from_DCM(double DCM[3][3], double e_axis1, double e_axis2, double e_axis3, double angle_offset_rad, double quat[4]) {
    /*
        This function initializes the quaternion by taking an initial DCM
        then rotating it by some offset and then converting to 
        a quaternion
    */
    double q_set[4], q_offset[4];
    q_offset[0] = sin(angle_offset_rad/2)*e_axis1/(e_axis1*e_axis1 + e_axis2*e_axis2 + e_axis3*e_axis3);
    q_offset[1] = sin(angle_offset_rad/2)*e_axis2/(e_axis1*e_axis1 + e_axis2*e_axis2 + e_axis3*e_axis3);
    q_offset[2] = sin(angle_offset_rad/2)*e_axis3/(e_axis1*e_axis1 + e_axis2*e_axis2 + e_axis3*e_axis3);
    q_offset[3] = cos(angle_offset_rad/2);

    DCM2quat(DCM, q_set);
    qmult(q_set, q_offset, quat);
}

void quaternion::qmult(double q1[], double q2[], double q3[]) {
    /*  

        *** VALIDATED WITH MATLAB ***

        Performs quaternion multiplication on two length
        four vectors, with the fourth element being
        the scalar element.

        q3 = q1 x q2

        q3_s = q1_s*q2_s - dot(q1_v, q2_v)
        q3_v = q1_s*q2_v + q2_s*q1_v - cross(q1_v, q2_v)
        q3 = [q3_v; q3_s]
    */

    q3[0] = q1[3]*q2[0] + q2[3]*q1[0] - q1[1]*q2[2] + q1[2]*q2[1];
    q3[1] = q1[3]*q2[1] + q2[3]*q1[1] - q1[2]*q2[0] + q1[0]*q2[2];
    q3[2] = q1[3]*q2[2] + q2[3]*q1[2] - q1[0]*q2[1] + q1[1]*q2[0];
    q3[3] = q1[3]*q2[3] - q2[0]*q1[0] - q2[1]*q1[1] - q2[2]*q1[2];
}

void quaternion::q_conjugate(double q[], double qconj[]) {
    /*

        *** VALIDATED WITH INSPECTION ***

        Calculates the conjugate of a quaterion

        q* = [-q_v; q_s]

    */
    qconj[0] = -q[0];
    qconj[1] = -q[1];
    qconj[2] = -q[2];
    qconj[3] = q[3];
}

void quaternion::calculate_qdot(double w_body[3], double quat[4], double qrate[4]) {

    /*
        Finds the quaternion rate of change 

        q_dot = .5*[w; 0] x q

        where 'x' denotes quaternion multiplication.
        See function 'qmult'
    */

    double w_intermediate[4], qprod[4];
    w_intermediate[0] = w_body[0];
    w_intermediate[1] = w_body[1];
    w_intermediate[2] = w_body[2];
    w_intermediate[3] = 0;

    qmult(w_intermediate, quat, qprod);

    qrate[0] = .5*qprod[0];
    qrate[1] = .5*qprod[1];
    qrate[2] = .5*qprod[2];
    qrate[3] = .5*qprod[3];
}

void quaternion::calculate_quaternion_error(double quat[4], double qdes[4], double qerr[4]) {
    /*
        dq is a parameterization of the rotation from the actual (estimated) body frame 
        to the desired body from. 

        dq = q_est x qdes*

        where 'x' denotes quaternion multiplication (See function 'qmult') and qnom* 
        is the quaternion conjugate of the desired quaternion (See function 'q_conjugate').
    */
    double qstar[4];
    q_conjugate(qdes, qstar);
    qmult(quat, qstar, qerr);
}

void quaternion::calculate_euler_error(double qerr[4], double eul_er[3]) {
    /*
        We parameterize the Roll-Pitch-Yaw error angles according to 
        the following equation (using small angle approximation). 

        dE = 2*dq_v/dq_s

    */

    eul_er[0] = 2*qerr[0]/qerr[3];
    eul_er[1] = 2*qerr[1]/qerr[3];
    eul_er[2] = 2*qerr[2]/qerr[3];
}

void quaternion::calculate_euler_error_rate(double w_body[3], double w_body_ref[3], double eul_er[3], double eul_er_rate[3]) {
    /*
        edot = -[wx]dE + dw
    */

    double dw[3], wdesx[3][3];

    dw[0] = w_body[0] - w_body_ref[0];
    dw[1] = w_body[1] - w_body_ref[1];
    dw[2] = w_body[2] - w_body_ref[2];

    wdesx[0][0] = 0.0;            wdesx[0][1] = -w_body_ref[2]; wdesx[0][2] = w_body_ref[1]; 
    wdesx[1][0] = w_body_ref[2];  wdesx[1][1] = 0.0;            wdesx[1][2] = -w_body_ref[0]; 
    wdesx[2][0] = -w_body_ref[1]; wdesx[2][1] = w_body_ref[0];  wdesx[2][2] = 0.0; 

    eul_er_rate[0] = -(wdesx[0][0]*eul_er[0] + wdesx[0][1]*eul_er[1] + wdesx[0][2]*eul_er[2]) + dw[0];
    eul_er_rate[1] = -(wdesx[1][0]*eul_er[0] + wdesx[1][1]*eul_er[1] + wdesx[1][2]*eul_er[2]) + dw[1];
    eul_er_rate[2] = -(wdesx[2][0]*eul_er[0] + wdesx[2][1]*eul_er[1] + wdesx[2][2]*eul_er[2]) + dw[2];


}
