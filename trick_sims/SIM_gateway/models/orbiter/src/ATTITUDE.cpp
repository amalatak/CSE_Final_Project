#include "../include/orbiter.hh"
#include "../include/ATTITUDE.hh"
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

/********************************************************************
UTILITIES
********************************************************************/

double set_semi_major_axis(double mu, double pos[], double vel[]) {
    double r_mag = sqrt(pos[0]*pos[0] + pos[1]*pos[1] + pos[2]*pos[2]) ;
    double v_mag = sqrt(vel[0]*vel[0] + vel[1]*vel[1] + vel[2]*vel[2]) ;
    double semi_major_axis = -mu*r_mag/((r_mag*v_mag*v_mag - 2.0*mu));
    return semi_major_axis;
}

void cross(double u[], double v[], double cross_p[]) {
    // calculate u x v
    cross_p[0] = u[1]*v[2] - v[1]*u[2];
    cross_p[1] = v[0]*u[2] - u[0]*v[2];
    cross_p[2] = u[0]*v[1] - v[0]*u[1];
}

/********************************************************************
CONSTANTS
********************************************************************/

void ATTITUDE::set_Jmat(double J_00, double J_01, double J_02, 
                        double J_10, double J_11, double J_12, 
                        double J_20, double J_21, double J_22)
    {
    Jmat[0][0] = J_00; Jmat[0][1] = J_01; Jmat[0][2] = J_02;
    Jmat[1][0] = J_10; Jmat[1][1] = J_11; Jmat[1][2] = J_12;
    Jmat[2][0] = J_20; Jmat[2][1] = J_21; Jmat[2][2] = J_22;
}

void ATTITUDE::set_Jmat_inv() {
    Matrix3d J_inter, J_inter_inv;
    J_inter << Jmat[0][0], Jmat[0][1], Jmat[0][2],
               Jmat[1][0], Jmat[1][1], Jmat[1][2],
               Jmat[2][0], Jmat[2][1], Jmat[2][2];  

    if (J_inter.determinant() <= 0.0001) {
        // check if determinant of J matrix is less than some value to ensure proper inversion
        cout << "J matrix is not invertible" << endl;
        exit(1);
    }

    J_inter_inv = J_inter.inverse(); 

    Jmat_inv[0][0] = J_inter_inv(0, 0); Jmat_inv[0][1] = J_inter_inv(0, 1); Jmat_inv[0][2] = J_inter_inv(0, 2);
    Jmat_inv[1][0] = J_inter_inv(1, 0); Jmat_inv[1][1] = J_inter_inv(1, 1); Jmat_inv[1][2] = J_inter_inv(1, 2);
    Jmat_inv[2][0] = J_inter_inv(2, 0); Jmat_inv[2][1] = J_inter_inv(2, 1); Jmat_inv[2][2] = J_inter_inv(2, 2);
}

/********************************************************************
DCM OPERATIONS
********************************************************************/
void ATTITUDE::set_body_i(double body_0_0, double body_0_1, double body_0_2, 
                          double body_1_0, double body_1_1, double body_1_2, 
                          double body_2_0, double body_2_1, double body_2_2) 
    {
    body_i[0][0] = body_0_0; body_i[0][1] = body_0_1; body_i[0][2] = body_0_2;
    body_i[1][0] = body_1_0; body_i[1][1] = body_1_1; body_i[1][2] = body_1_2;
    body_i[2][0] = body_2_0; body_i[2][1] = body_2_1; body_i[2][2] = body_2_2; 
}

void ATTITUDE::calculate_body_LVLH() {
    /* 
    Gets body relative to LVLH frame
    T_lvlh2b = Ti2b*transpose(Ti2lvlh)
    */
    
    body_lvlh[0][0] = body_i[0][0]*LVLH_i[0][0] + body_i[0][1]*LVLH_i[0][1] + body_i[0][2]*LVLH_i[0][2];
    body_lvlh[0][1] = body_i[0][0]*LVLH_i[1][0] + body_i[0][1]*LVLH_i[1][1] + body_i[0][2]*LVLH_i[1][2];
    body_lvlh[0][2] = body_i[0][0]*LVLH_i[2][0] + body_i[0][1]*LVLH_i[2][1] + body_i[0][2]*LVLH_i[2][2];

    body_lvlh[1][0] = body_i[1][0]*LVLH_i[0][0] + body_i[1][1]*LVLH_i[0][1] + body_i[1][2]*LVLH_i[0][2];
    body_lvlh[1][1] = body_i[1][0]*LVLH_i[1][0] + body_i[1][1]*LVLH_i[1][1] + body_i[1][2]*LVLH_i[1][2];
    body_lvlh[1][2] = body_i[1][0]*LVLH_i[2][0] + body_i[1][1]*LVLH_i[2][1] + body_i[1][2]*LVLH_i[2][2];

    body_lvlh[2][0] = body_i[2][0]*LVLH_i[0][0] + body_i[2][1]*LVLH_i[0][1] + body_i[2][2]*LVLH_i[0][2];
    body_lvlh[2][1] = body_i[2][0]*LVLH_i[1][0] + body_i[2][1]*LVLH_i[1][1] + body_i[2][2]*LVLH_i[1][2];
    body_lvlh[2][2] = body_i[2][0]*LVLH_i[2][0] + body_i[2][1]*LVLH_i[2][1] + body_i[2][2]*LVLH_i[2][2];
}

void ATTITUDE::calculate_LVLH_i() {

    /* 
    Algorithm to get LVLH DCM from velocity and position 
    In this case, the LVLH frame has the axes aligned such that
    x is parallel to position
    y completes the right handed system along the velocity component
    z is perpendicular to the orbit plane

    xhat = pos/norm(pos)
    zhat = cross(pos, vel)/norm(cross(pos, vel))
    yhat = cross(zhat, xhat)
    
    */

    double x_dir[3], y_dir[3], z_dir[3];
    double cross_xy[3];

    cross(position, velocity, cross_xy);
    
    x_dir[0] = rv[0]/sqrt(rv[0]*rv[0] + rv[1]*rv[1] + rv[2]*rv[2]) ;
    x_dir[1] = rv[1]/sqrt(rv[0]*rv[0] + rv[1]*rv[1] + rv[2]*rv[2]) ;
    x_dir[2] = rv[2]/sqrt(rv[0]*rv[0] + rv[1]*rv[1] + rv[2]*rv[2]) ;

    z_dir[0] = cross_xy[0]/sqrt(cross_xy[0]*cross_xy[0] + cross_xy[1]*cross_xy[1] + cross_xy[2]*cross_xy[2]) ;
    z_dir[1] = cross_xy[1]/sqrt(cross_xy[0]*cross_xy[0] + cross_xy[1]*cross_xy[1] + cross_xy[2]*cross_xy[2]) ;
    z_dir[2] = cross_xy[2]/sqrt(cross_xy[0]*cross_xy[0] + cross_xy[1]*cross_xy[1] + cross_xy[2]*cross_xy[2]) ;

    cross(z_dir, x_dir, y_dir) ;

    LVLH_i[0][0] = x_dir[0]; LVLH_i[0][1] = x_dir[1]; LVLH_i[0][2] = x_dir[2];
    LVLH_i[1][0] = y_dir[0]; LVLH_i[1][1] = y_dir[1]; LVLH_i[1][2] = y_dir[2];
    LVLH_i[2][0] = z_dir[0]; LVLH_i[2][1] = z_dir[1]; LVLH_i[2][2] = z_dir[2];
}



/********************************************************************
QUATERNION OPERATIONS
********************************************************************/

void ATTITUDE::qmult(double q1[], double q2[], double q3[]) {
    /*  
        Performs quaternion multiplication on two length
        four vectors, with the fourth element being
        the scalar element.

        q3 = q1 x q2

        q3_s = q1_s*q2_s - dot(q1_v, q2_v)
        q3_v = q1_s*q2_v + q2_s*q1_v + cross(q1_v, q2_v)
        q3 = [q3_v; q3_s]
    */
    q3[0] = q1[3]*q2[0] + q2[3]*q1[0] + q1[1]*q2[2] - q1[2]*q2[1];
    q3[1] = q1[3]*q2[1] + q2[3]*q1[1] + q1[2]*q2[0] - q1[0]*q2[2];
    q3[2] = q1[3]*q2[2] + q2[3]*q1[2] + q1[0]*q2[1] - q1[1]*q2[0];
    q3[3] = q1[3]*q2[3] - q2[0]*q1[0] - q2[1]*q1[1] - q2[2]*q1[2];
}

void ATTITUDE::q_conjugate(double q[], double qconj[]) {
    /*
        Calculates the conjugate of a quaterion

        q* = [-q_v; q_s]

    */
    qconj[0] = -q[0];
    qconj[1] = -q[1];
    qconj[2] = -q[2];
    qconj[3] = q[3];
}

void ATTITUDE::calculate_qdot(double w[], double q[], double qdot[]) {

    /*
        Finds the quaternion rate of change 

        q_dot = .5*[w; 0] x q

        where 'x' denotes quaternion multiplication.
        See function 'qmult'
    */

    double w_intermediate[4], qprod[4];
    w_intermediate[0] = w[0];
    w_intermediate[1] = w[1];
    w_intermediate[2] = w[2];
    w_intermediate[3] = 0;

    qmult(w_intermediate, q, qprod);

    qdot[0] = .5*qprod[0];
    qdot[1] = .5*qprod[1];
    qdot[2] = .5*qprod[2];
    qdot[3] = .5*qprod[3];
}

void ATTITUDE::calculate_quaternion_error(double q_est[], double q_des[], double dq[]) {
    /*
        dq is a parameterization of the rotation from the actual (estimated) body frame 
        to the desired body from. 

        dq = q_est x qdes*

        where 'x' denotes quaternion multiplication (See function 'qmult') and qnom* 
        is the quaternion conjugate of the desired quaternion (See function 'q_conjugate').
    */
    double qstar[4];
    q_conjugate(q_des, qstar);
    qmult(q_est, qstar, dq);
}

void ATTITUDE::calculate_euler_error(double dq[], double eul_er[]) {
    /*
        We parameterize the Roll-Pitch-Yaw error angles according to 
        the following equation (using small angle approximation). 

        dE = 2*dq_v/dq_s

    */

    eul_er[0] = 2*dq[0]/dq[3];
    eul_er[1] = 2*dq[1]/dq[3];
    eul_er[2] = 2*dq[2]/dq[3];
}

/********************************************************************
KINEMATICS
********************************************************************/
void ATTITUDE::set_rv(double pos[], double vel[]) {
    rv[0] = pos[0];
    rv[1] = pos[1];
    rv[2] = pos[2];
    rv[3] = vel[0];
    rv[4] = vel[1];
    rv[5] = vel[2];
    position[0] = pos[0]; position[1] = pos[1]; position[2] = pos[2];
    velocity[0] = vel[0]; velocity[1] = vel[1]; velocity[2] = vel[2];
}

void ATTITUDE::set_torques(double torque_set []) {
    torques[0] = torque_set[0];
    torques[1] = torque_set[1];
    torques[2] = torque_set[2];
}

void ATTITUDE::calculate_wdot_body_bwrti() {
    /*
    Calculate rate of change of angular velocity of a spacecraft body with respect to the inertial frame
    coordinatized in the body frame 

    wdot*J = sum(M) => wdot = J^(-1)*(sum(M) - cross(w_body_bwrti, Jmat*w_body_bwrti))

    */

    double w_cross[3], Jw[3], t_sub_wcross[3];

    Jw[0] = Jmat[0][0]*w_body_b[0] + Jmat[0][1]*w_body_b[1] + Jmat[0][2]*w_body_b[2];
    Jw[1] = Jmat[1][0]*w_body_b[0] + Jmat[1][1]*w_body_b[1] + Jmat[1][2]*w_body_b[2];
    Jw[2] = Jmat[2][0]*w_body_b[0] + Jmat[2][1]*w_body_b[1] + Jmat[2][2]*w_body_b[2];

    cross(w_body_b, Jw, w_cross);

    t_sub_wcross[0] = torques[0] - w_cross[0];
    t_sub_wcross[1] = torques[1] - w_cross[1];
    t_sub_wcross[2] = torques[2] - w_cross[2];

    wdot_b_b[0] = Jmat_inv[0][0]*t_sub_wcross[0] + Jmat_inv[0][1]*t_sub_wcross[1] + Jmat_inv[0][2]*t_sub_wcross[2];
    wdot_b_b[1] = Jmat_inv[1][0]*t_sub_wcross[0] + Jmat_inv[1][1]*t_sub_wcross[1] + Jmat_inv[1][2]*t_sub_wcross[2];
    wdot_b_b[2] = Jmat_inv[2][0]*t_sub_wcross[0] + Jmat_inv[2][1]*t_sub_wcross[1] + Jmat_inv[2][2]*t_sub_wcross[2];

}

