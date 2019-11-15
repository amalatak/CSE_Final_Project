/*************************************************************************
PURPOSE: ( Orbiter Attitude Model )
**************************************************************************/
#ifndef UTILITIES_H
#define UTILITIES_H

#include <iostream>
#include <vector>
#include <math.h>


#ifdef __cplusplus
extern "C" {
#endif


class UTILITIES {

public:

    void print_mat(double mat[3][3]);
    void print_quat(double quat[4]);
    void print_vec(double vec[3]);
    void set_vec(double vec1, double vec2, double vec3, double vec[3]);
    
    /* Math Functions */
    double dot(double u[], double v[]);
    void cross(double u[], double v[], double cross_p[]);
    void norm(double vec[3], double vec_norm[3]);
    void transpose(double mat[3][3], double mat_transpose[3][3]);
    void matmul(double mat1[3][3], double mat2[3][3], double mat1xmat2[3][3]);
    void matvecmul(double mat[3][3], double vec[3], double matxvec[3]);
};

#ifdef __cplusplus
}
#endif

#endif

