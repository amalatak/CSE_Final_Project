#include "../include/mass_geometry.hh"
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

void mass_geometry::set_Jmat(double J_00, double J_01, double J_02, 
                        double J_10, double J_11, double J_12, 
                        double J_20, double J_21, double J_22)
    {
    Jmat[0][0] = J_00; Jmat[0][1] = J_01; Jmat[0][2] = J_02;
    Jmat[1][0] = J_10; Jmat[1][1] = J_11; Jmat[1][2] = J_12;
    Jmat[2][0] = J_20; Jmat[2][1] = J_21; Jmat[2][2] = J_22;

}

void mass_geometry::set_Jmat_inv() {
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