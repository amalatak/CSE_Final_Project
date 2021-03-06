/*************************************************************************
PURPOSE: ( Orbiter DCM Library Model )
**************************************************************************/
#ifndef DCM_HH
#define DCM_HH

#include "UTILITIES.hh"
#include <iostream>
#include <vector>
#include <math.h>


#ifdef __cplusplus
extern "C" {
#endif


class DCM {
private:

    UTILITIES utility;

public:
    void set_DCM_from_DCM(double DCM_input[3][3], double DCM_set[3][3]);

    void set_DCM_from_elements(double DCM_0_0, double DCM_0_1, double DCM_0_2, 
                               double DCM_1_0, double DCM_1_1, double DCM_1_2, 
                               double DCM_2_0, double DCM_2_1, double DCM_2_2, 
                               double DCM_set[3][3]);

    void calculate_body_chaser(double chaser_frame[3][3], double Ti2b[3][3], double body_chaser[3][3]);
    void calculate_body_LVLH(double LVLH_i[3][3], double body_i[3][3], double body_lvlh[3][3]);

    void T1(double angle, double T1_mat[3][3]);
    void T2(double angle, double T2_mat[3][3]);
    void T3(double angle, double T3_mat[3][3]);

};

#ifdef __cplusplus
}
#endif

#endif

