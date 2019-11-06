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

/*
TO DO:
 * Add TRIAD to determine attitutde
 * Add Camera sensor on chaser
 * Correct error in chaser attitude (likely from desired values)
*/

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

void ATTITUDE::print_mat(double frame[3][3]) {
    cout << "[" << frame[0][0] << ", " << frame[0][1] << ", " << frame[0][2] << ", " << endl;
    cout << " " << frame[1][0] << ", " << frame[1][1] << ", " << frame[1][2] << ", " << endl;
    cout << " " << frame[2][0] << ", " << frame[2][1] << ", " << frame[2][2] << "] " << endl;
}

void ATTITUDE::print_quat(double quat[4]) {
    cout << "quat: [" << 
    quat[0] << ", " <<
    quat[1] << ", " <<
    quat[2] << ", " <<
    quat[3] << "] " <<endl;
}

void ATTITUDE::print_vec(double vec[3]) {
    cout << "some vec: [" << 
    vec[0] << ", " <<
    vec[1] << ", " <<
    vec[2] << "] " <<endl; 
}

void ATTITUDE::print_qerr() {
    printf( "qer = [%.9f, %.9f, %.9f]\n", eul_er_est[0], eul_er_est[1], eul_er_est[2]);
    printf( "dqr = [%.9f, %.9f, %.9f]\n", eul_er_rate_est[0], eul_er_rate_est[1], eul_er_rate_est[2]);
}

void ATTITUDE::set_vec(double vec1, double vec2, double vec3, double vec[3]) {
    vec[0] = vec1;
    vec[1] = vec2;
    vec[2] = vec3;
}

void ATTITUDE::quat2DCM(double quat[4], double DCM[3][3]) {
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

void ATTITUDE::DCM2quat(double DCM[3][3], double quat[4]) {
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
    /*
    Store the inverse of the J matrix for speed
    
    *** VERIFIED AGAINST MATLAB ***

    */

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
    /*
        Initialize or set i2b matrix
    */
    body_i[0][0] = body_0_0; body_i[0][1] = body_0_1; body_i[0][2] = body_0_2;
    body_i[1][0] = body_1_0; body_i[1][1] = body_1_1; body_i[1][2] = body_1_2;
    body_i[2][0] = body_2_0; body_i[2][1] = body_2_1; body_i[2][2] = body_2_2; 
}

void ATTITUDE::calculate_chaser_frame() {
    /*
        This function calculates the "chaser frame" from relative position,
        chaser position and velocity and has axes aligned such that:
        x points to the target
        y completes the right handed system
        z is perpendicular to the orbit plane along the LVLH z

        x_hat = norm(rel_pos)
        zhat = -cross(pos, vel)/norm(cross(pos, vel))
        yhat = cross(zhat, xhat)
    */
    double x_dir[3], y_dir[3], z_dir[3];
    double cross_xy[3];

    cross(position, velocity, cross_xy);

    x_dir[0] = pos_rel[0]/sqrt(pos_rel[0]*pos_rel[0] + pos_rel[1]*pos_rel[1] + pos_rel[2]*pos_rel[2]) ;
    x_dir[1] = pos_rel[1]/sqrt(pos_rel[0]*pos_rel[0] + pos_rel[1]*pos_rel[1] + pos_rel[2]*pos_rel[2]) ;
    x_dir[2] = pos_rel[2]/sqrt(pos_rel[0]*pos_rel[0] + pos_rel[1]*pos_rel[1] + pos_rel[2]*pos_rel[2]) ;

    z_dir[0] = -cross_xy[0]/sqrt(cross_xy[0]*cross_xy[0] + cross_xy[1]*cross_xy[1] + cross_xy[2]*cross_xy[2]) ;
    z_dir[1] = -cross_xy[1]/sqrt(cross_xy[0]*cross_xy[0] + cross_xy[1]*cross_xy[1] + cross_xy[2]*cross_xy[2]) ;
    z_dir[2] = -cross_xy[2]/sqrt(cross_xy[0]*cross_xy[0] + cross_xy[1]*cross_xy[1] + cross_xy[2]*cross_xy[2]) ;

    cross(z_dir, x_dir, y_dir) ;

    chaser_i[0][0] = x_dir[0]; chaser_i[0][1] = x_dir[1]; chaser_i[0][2] = x_dir[2];
    chaser_i[1][0] = y_dir[0]; chaser_i[1][1] = y_dir[1]; chaser_i[1][2] = y_dir[2];
    chaser_i[2][0] = z_dir[0]; chaser_i[2][1] = z_dir[1]; chaser_i[2][2] = z_dir[2];

}

void ATTITUDE::calculate_body_chaser() {
    /*
        Gets body relative to chaser frame
        T_chaser2b = Ti2b*transpose(Ti2chaser)
    */

    body_chaser[0][0] = body_i[0][0]*chaser_i[0][0] + body_i[0][1]*chaser_i[0][1] + body_i[0][2]*chaser_i[0][2];
    body_chaser[0][1] = body_i[0][0]*chaser_i[1][0] + body_i[0][1]*chaser_i[1][1] + body_i[0][2]*chaser_i[1][2];
    body_chaser[0][2] = body_i[0][0]*chaser_i[2][0] + body_i[0][1]*chaser_i[2][1] + body_i[0][2]*chaser_i[2][2];

    body_chaser[1][0] = body_i[1][0]*chaser_i[0][0] + body_i[1][1]*chaser_i[0][1] + body_i[1][2]*chaser_i[0][2];
    body_chaser[1][1] = body_i[1][0]*chaser_i[1][0] + body_i[1][1]*chaser_i[1][1] + body_i[1][2]*chaser_i[1][2];
    body_chaser[1][2] = body_i[1][0]*chaser_i[2][0] + body_i[1][1]*chaser_i[2][1] + body_i[1][2]*chaser_i[2][2];

    body_chaser[2][0] = body_i[2][0]*chaser_i[0][0] + body_i[2][1]*chaser_i[0][1] + body_i[2][2]*chaser_i[0][2];
    body_chaser[2][1] = body_i[2][0]*chaser_i[1][0] + body_i[2][1]*chaser_i[1][1] + body_i[2][2]*chaser_i[1][2];
    body_chaser[2][2] = body_i[2][0]*chaser_i[2][0] + body_i[2][1]*chaser_i[2][1] + body_i[2][2]*chaser_i[2][2];

}

void ATTITUDE::calculate_LVLH_i() {

    /* 

    *** VERIFIED AGAINST MATLAB ***

    Algorithm to get LVLH DCM from velocity and position 
    In this case, the LVLH frame has the axes aligned such that
    x is center pointing
    y completes the right handed system along the velocity component
    z is perpendicular to the orbit plane

    xhat = -pos/norm(pos)
    zhat = -cross(pos, vel)/norm(cross(pos, vel))
    yhat = cross(zhat, xhat)
    
    */

    double x_dir[3], y_dir[3], z_dir[3];
    double cross_xy[3];

    cross(position, velocity, cross_xy);
    
    x_dir[0] = -rv[0]/sqrt(rv[0]*rv[0] + rv[1]*rv[1] + rv[2]*rv[2]) ;
    x_dir[1] = -rv[1]/sqrt(rv[0]*rv[0] + rv[1]*rv[1] + rv[2]*rv[2]) ;
    x_dir[2] = -rv[2]/sqrt(rv[0]*rv[0] + rv[1]*rv[1] + rv[2]*rv[2]) ;

    z_dir[0] = -cross_xy[0]/sqrt(cross_xy[0]*cross_xy[0] + cross_xy[1]*cross_xy[1] + cross_xy[2]*cross_xy[2]) ;
    z_dir[1] = -cross_xy[1]/sqrt(cross_xy[0]*cross_xy[0] + cross_xy[1]*cross_xy[1] + cross_xy[2]*cross_xy[2]) ;
    z_dir[2] = -cross_xy[2]/sqrt(cross_xy[0]*cross_xy[0] + cross_xy[1]*cross_xy[1] + cross_xy[2]*cross_xy[2]) ;

    cross(z_dir, x_dir, y_dir) ;

    LVLH_i[0][0] = x_dir[0]; LVLH_i[0][1] = x_dir[1]; LVLH_i[0][2] = x_dir[2];
    LVLH_i[1][0] = y_dir[0]; LVLH_i[1][1] = y_dir[1]; LVLH_i[1][2] = y_dir[2];
    LVLH_i[2][0] = z_dir[0]; LVLH_i[2][1] = z_dir[1]; LVLH_i[2][2] = z_dir[2];
}

void ATTITUDE::calculate_body_LVLH() {
    /* 

    *** VERIFIED AGAINST MATLAB ***

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


/********************************************************************
QUATERNION OPERATIONS

This library assumes that the scalar element of the quaternion is q[3]
and that the quaternion is defined as [e_vec*sin(theta/2); cos(theta/2)]
********************************************************************/

void ATTITUDE::set_qest(double DCM[3][3], double angle_offset, double e_axis[3]) {
    /*
        This function initializes the quaternion by taking an initial DCM
        then rotating it by some offset and then converting to 
        a quaternion
    */
    double q_set[4], q_offset[4];
    q_offset[0] = sin(angle_offset/2)*e_axis[0];
    q_offset[1] = sin(angle_offset/2)*e_axis[1];
    q_offset[2] = sin(angle_offset/2)*e_axis[2];
    q_offset[3] = cos(angle_offset/2);

    DCM2quat(DCM, q_set);
    qmult(q_set, q_offset, qest);
}

void ATTITUDE::qmult(double q1[], double q2[], double q3[]) {
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

void ATTITUDE::q_conjugate(double q[], double qconj[]) {
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

void ATTITUDE::calculate_qdot() {

    /*
        Finds the quaternion rate of change 

        q_dot = .5*[w; 0] x q

        where 'x' denotes quaternion multiplication.
        See function 'qmult'
    */

    double w_intermediate[4], qprod[4];
    w_intermediate[0] = w_body_b[0];
    w_intermediate[1] = w_body_b[1];
    w_intermediate[2] = w_body_b[2];
    w_intermediate[3] = 0;

    qmult(w_intermediate, qest, qprod);

    q_dot[0] = .5*qprod[0];
    q_dot[1] = .5*qprod[1];
    q_dot[2] = .5*qprod[2];
    q_dot[3] = .5*qprod[3];
}

void ATTITUDE::calculate_quaternion_error() {
    /*
        dq is a parameterization of the rotation from the actual (estimated) body frame 
        to the desired body from. 

        dq = q_est x qdes*

        where 'x' denotes quaternion multiplication (See function 'qmult') and qnom* 
        is the quaternion conjugate of the desired quaternion (See function 'q_conjugate').
    */
    double qstar[4];
    q_conjugate(qdes, qstar);
    qmult(qest, qstar, q_err);
}

void ATTITUDE::calculate_euler_error() {
    /*
        We parameterize the Roll-Pitch-Yaw error angles according to 
        the following equation (using small angle approximation). 

        dE = 2*dq_v/dq_s

    */
    calculate_quaternion_error();

    eul_er_est[0] = 2*q_err[0]/q_err[3];
    eul_er_est[1] = 2*q_err[1]/q_err[3];
    eul_er_est[2] = 2*q_err[2]/q_err[3];
}

void ATTITUDE::calculate_euler_error_rate() {
    /*
        edot = -[wx]dE + dw
    */

    double dw[3], wdesx[3][3];

    dw[0] = w_body_b[0] - w_ref_b[0];
    dw[1] = w_body_b[1] - w_ref_b[1];
    dw[2] = w_body_b[2] - w_ref_b[2];

    wdesx[0][0] = 0.0;         wdesx[0][1] = -w_ref_b[2]; wdesx[0][2] = w_ref_b[1]; 
    wdesx[1][0] = w_ref_b[2];  wdesx[1][1] = 0.0;         wdesx[1][2] = -w_ref_b[0]; 
    wdesx[2][0] = -w_ref_b[1]; wdesx[2][1] = w_ref_b[0];  wdesx[2][2] = 0.0; 

    eul_er_rate_est[0] = -(wdesx[0][0]*eul_er_est[0] + wdesx[0][1]*eul_er_est[1] + wdesx[0][2]*eul_er_est[2]) + dw[0];
    eul_er_rate_est[1] = -(wdesx[1][0]*eul_er_est[0] + wdesx[1][1]*eul_er_est[1] + wdesx[1][2]*eul_er_est[2]) + dw[1];
    eul_er_rate_est[2] = -(wdesx[2][0]*eul_er_est[0] + wdesx[2][1]*eul_er_est[1] + wdesx[2][2]*eul_er_est[2]) + dw[2];


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

void ATTITUDE::set_pos_rel(double pos2[]) {
    /*
        This function finds the relative position of the 
        chaser to the target. It sets the relative vector
        to be rel_r = pos_target - pos_chaser
    */
    pos_rel[0] = pos2[0] - position[0];
    pos_rel[1] = pos2[1] - position[1];
    pos_rel[2] = pos2[2] - position[2];
}

void ATTITUDE::set_torques(double torque_set1, double torque_set2, double torque_set3) {
    torques[0] = torque_set1;
    torques[1] = torque_set2;
    torques[2] = torque_set3;
}

void ATTITUDE::set_w_body_b(double w_b[3]) {
    w_body_b[0] = w_b[0];
    w_body_b[1] = w_b[1];
    w_body_b[2] = w_b[2];
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

/********************************************************************
CONTROLS
********************************************************************/

void ATTITUDE::set_gains(double K_p, double K_i, double K_d) {
    Kp = K_p;
    Ki = K_i;
    Kd = K_d;
}

void ATTITUDE::set_qdes_center_pointing() {
    /* 
        This function sets the desired quaternion based on current position
        to be pointing at the center of the main orbiting body. 
    */
    DCM2quat(LVLH_i, qdes);
}

void ATTITUDE::set_wdes_center_pointing() {
    /* 

        *** CALCULATES CORRECTLY, VALIDATE ALGORITHM

        This function sets the desired angular velocity based on current position
        and velocity in order to maintain pointing at the center of the orbiting body
    */
    double rsq, rxv[3];
    rsq = position[0]*position[0] + position[1]*position[1] + position[2]*position[2];
    cross(position, velocity, rxv);

    w_ref_i[0] = rxv[0]/rsq;
    w_ref_i[1] = rxv[1]/rsq;
    w_ref_i[2] = rxv[2]/rsq;

    quat2DCM(qest, body_i);

    w_ref_b[0] = body_i[0][0]*w_ref_i[0] + body_i[0][1]*w_ref_i[1] + body_i[0][2]*w_ref_i[2];
    w_ref_b[1] = body_i[1][0]*w_ref_i[0] + body_i[1][1]*w_ref_i[1] + body_i[1][2]*w_ref_i[2];
    w_ref_b[2] = body_i[2][0]*w_ref_i[0] + body_i[2][1]*w_ref_i[1] + body_i[2][2]*w_ref_i[2];

}

void ATTITUDE::set_qdes_target_pointing() {
    /* 
        This function sets the desired quaternion based on current position
        to be pointing at the center of the target. 
    */
    DCM2quat(chaser_i, qdes);
}

void ATTITUDE::set_wdes_target_pointing() {
    /* 
        This function sets the desired angular velocity ...
        for now, set it to match target
    */
    set_wdes_center_pointing();

}

void ATTITUDE::PD() {
    /*
        This function issues desired control torques given current 
        and desired attitude
    */

    calculate_euler_error();
    calculate_euler_error_rate();

    control_out[0] = -Kp*eul_er_est[0] - Kd*eul_er_rate_est[0];
    control_out[1] = -Kp*eul_er_est[1] - Kd*eul_er_rate_est[1];
    control_out[2] = -Kp*eul_er_est[2] - Kd*eul_er_rate_est[2];

    torques[0] = Jmat[0][0]*control_out[0] + Jmat[0][1]*control_out[1] + Jmat[0][2]*control_out[2];
    torques[1] = Jmat[1][0]*control_out[0] + Jmat[1][1]*control_out[1] + Jmat[1][2]*control_out[2];
    torques[2] = Jmat[2][0]*control_out[0] + Jmat[2][1]*control_out[1] + Jmat[2][2]*control_out[2];
}

/********************************************************************
ESTIMATION
********************************************************************/
void ATTITUDE::TRIAD() {
    /*
        This function uses two vectors to calculate a reference frame
    */
}

void ATTITUDE::camera() {
    /*
        This function simulates the chaser sensing the relative location
        of the target using a camera.
    */

}

/********************************************************************
TOP LEVEL SCRIPTS
********************************************************************/

void ATTITUDE::target_initialize() {
    /* 
        This script assumes the following variables have been set:
         * Position
         * Velocity
         * J
        
        And will initialize the following variables:
         * qest
         * LVLH_i
         * w_body
         * wdot_body
         * External Torques
    */
    double offset_eul[3];
    offset_eul[0] = 1.0;
    offset_eul[1] = 1.0;
    offset_eul[2] = 1.0;

    set_torques(0.0, 0.0, 0.0);
    set_vec(0.0, 0.0, 0.0, wdot_b_b);
    set_vec(0.0, 0.0, 0.0, w_body_b);

    calculate_LVLH_i();
    set_qest(LVLH_i, 0.0, offset_eul);

}

void ATTITUDE::chaser_initialize() {
    /* 
        This script assumes the following variables have been set:
         * Position
         * Velocity
         * J
        
        And will initialize the following variables:
         * qest
         * chaser_i
         * w_body
         * wdot_body
         * External Torques
    */

    double offset_eul[3];
    offset_eul[0] = 1.0;
    offset_eul[1] = 1.0;
    offset_eul[2] = 1.0;

    set_torques(0.0, 0.0, 0.0);
    set_vec(0.0, 0.0, 0.0, wdot_b_b);
    set_vec(0.0, 0.0, 0.0, w_body_b);

    calculate_chaser_frame();
    set_qest(chaser_i, 0.0, offset_eul);

    set_qdes_target_pointing();
    set_wdes_target_pointing();

}

void ATTITUDE::target_dynamics_update(double pos[], double vel[]) {
    /*
        This script is to update the dynamics of the target
        assuming it is set to reference the LVLH frame and moves
        independent of the chaser. 
    */
    set_rv(pos, vel);
    calculate_LVLH_i();
    calculate_wdot_body_bwrti();
    calculate_qdot();
    calculate_body_LVLH();
    set_qdes_center_pointing();
    set_wdes_center_pointing();

    PD();
}

void ATTITUDE::chaser_dynamics_update(double pos[], double vel[], double target_pos[]) {
    /*
        This script is to update the dynamics of the chaser
        assuming it is set to reference the defined chaser frame and moves
        relative to the target. 
    */

    set_rv(pos, vel);
    set_pos_rel(target_pos);
    calculate_chaser_frame();
    calculate_wdot_body_bwrti();
    calculate_qdot();
    calculate_body_chaser();
    set_qdes_target_pointing();
    set_wdes_target_pointing();

    PD();
}