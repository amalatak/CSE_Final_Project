#include "../include/UTILITIES.hh"
#include <stddef.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>

using std::cout;
using std::endl;

double UTILITIES::dot(double u[], double v[]) {
    // calculate u.v
    return u[0]*v[0] + u[1]*v[1] + u[2]*v[2];
}

void UTILITIES::addition(double u[3], double v[3], double u_plus_v[3]) {
    /* Adds two vectors */
    u_plus_v[0] = u[0] + v[0];
    u_plus_v[1] = u[1] + v[1];
    u_plus_v[2] = u[2] + v[2];
}
void UTILITIES::subtract(double u[3], double v[3], double u_minus_v[3]) {
    /* Subtracts two vectors */
    u_minus_v[0] = u[0] - v[0];
    u_minus_v[1] = u[1] - v[1];
    u_minus_v[2] = u[2] - v[2];
}

void UTILITIES::identity(double eye[3][3]) {
    eye[0][0] = 1; eye[0][1] = 0; eye[0][2] = 0; 
    eye[1][0] = 0; eye[1][1] = 1; eye[1][2] = 0; 
    eye[2][0] = 0; eye[2][1] = 0; eye[2][2] = 1; 
}

void UTILITIES::cross(double u[], double v[], double cross_p[]) {
    // calculate u x v
    cross_p[0] = u[1]*v[2] - v[1]*u[2];
    cross_p[1] = v[0]*u[2] - u[0]*v[2];
    cross_p[2] = u[0]*v[1] - v[0]*u[1];
}

void UTILITIES::norm(double vec[3], double vec_norm[3]) {
    double vec_mag = sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);
    vec_norm[0] = vec[0]/vec_mag;
    vec_norm[1] = vec[1]/vec_mag;
    vec_norm[2] = vec[2]/vec_mag;
}

void UTILITIES::print_mat(double frame[3][3]) {
    cout << "[" << frame[0][0] << ", " << frame[0][1] << ", " << frame[0][2] << ", " << endl;
    cout << " " << frame[1][0] << ", " << frame[1][1] << ", " << frame[1][2] << ", " << endl;
    cout << " " << frame[2][0] << ", " << frame[2][1] << ", " << frame[2][2] << "] " << endl;
}

void UTILITIES::print_quat(double quat[4]) {
    cout << "quat: [" << 
    quat[0] << ", " <<
    quat[1] << ", " <<
    quat[2] << ", " <<
    quat[3] << "] " <<endl;
}

void UTILITIES::print_vec(double vec[3]) {
    cout << "some vec: [" << 
    vec[0] << ", " <<
    vec[1] << ", " <<
    vec[2] << "] " <<endl; 
}

void UTILITIES::print_scalar(double scalar) {
    cout << "Some scalar: " << scalar << endl;
}

void UTILITIES::set_vec(double vec1, double vec2, double vec3, double vec[3]) {
    vec[0] = vec1;
    vec[1] = vec2;
    vec[2] = vec3;
}

void UTILITIES::set_mat(double mat[3][3], double set_mat[3][3]) {
    set_mat[0][0] = mat[0][0]; set_mat[0][1] = mat[0][1]; set_mat[0][2] = mat[0][2];
    set_mat[1][0] = mat[1][0]; set_mat[1][1] = mat[1][1]; set_mat[1][2] = mat[1][2];
    set_mat[2][0] = mat[2][0]; set_mat[2][1] = mat[2][1]; set_mat[2][2] = mat[2][2];
}

void UTILITIES::transpose(double mat[3][3], double mat_transpose[3][3]) {
    /* 
        Takes transpose of a 3x3 matrix 
    */

    mat_transpose[0][0] = mat[0][0]; mat_transpose[0][1] = mat[1][0]; mat_transpose[0][2] = mat[2][0];
    mat_transpose[1][0] = mat[0][1]; mat_transpose[1][1] = mat[1][1]; mat_transpose[1][2] = mat[2][1];
    mat_transpose[2][0] = mat[0][2]; mat_transpose[2][1] = mat[1][2]; mat_transpose[2][2] = mat[2][2];
}
void UTILITIES::matmul(double mat1[3][3], double mat2[3][3], double mat1xmat2[3][3]) {
    /* 
        Multiplies two 3x3 matrices
        mat1xmat2 = mat1*mat2
    */
    mat1xmat2[0][0] = mat1[0][0]*mat2[0][0] + mat1[0][1]*mat2[1][0] + mat1[0][2]*mat2[2][0];
    mat1xmat2[0][1] = mat1[0][0]*mat2[0][1] + mat1[0][1]*mat2[1][1] + mat1[0][2]*mat2[2][1];
    mat1xmat2[0][2] = mat1[0][0]*mat2[0][2] + mat1[0][1]*mat2[1][2] + mat1[0][2]*mat2[2][2];

    mat1xmat2[1][0] = mat1[1][0]*mat2[0][0] + mat1[1][1]*mat2[1][0] + mat1[1][2]*mat2[2][0];
    mat1xmat2[1][1] = mat1[1][0]*mat2[0][1] + mat1[1][1]*mat2[1][1] + mat1[1][2]*mat2[2][1];
    mat1xmat2[1][2] = mat1[1][0]*mat2[0][2] + mat1[1][1]*mat2[1][2] + mat1[1][2]*mat2[2][2];

    mat1xmat2[2][0] = mat1[2][0]*mat2[0][0] + mat1[2][1]*mat2[1][0] + mat1[2][2]*mat2[2][0];
    mat1xmat2[2][1] = mat1[2][0]*mat2[0][1] + mat1[2][1]*mat2[1][1] + mat1[2][2]*mat2[2][1];
    mat1xmat2[2][2] = mat1[2][0]*mat2[0][2] + mat1[2][1]*mat2[1][2] + mat1[2][2]*mat2[2][2];
}

void UTILITIES::matvecmul(double mat[3][3], double vec[3], double matxvec[3]) {
    /* 
        Multiplies a 3x3 matrix by a 3 element vector
        matxvec = mat*vec
    */
    matxvec[0] = mat[0][0]*vec[0] + mat[0][1]*vec[1] + mat[0][2]*vec[2];
    matxvec[1] = mat[1][0]*vec[0] + mat[1][1]*vec[1] + mat[1][2]*vec[2];
    matxvec[2] = mat[2][0]*vec[0] + mat[2][1]*vec[1] + mat[2][2]*vec[2];

}
