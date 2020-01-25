/*************************************************************************
PURPOSE: ( Orbiter DCM Library Model )
**************************************************************************/
#ifndef DCM_HH
#define DCM_HH

#include "UTILITIES.hh"
#include "estimation.hh"
#include <iostream>
#include <vector>
#include <math.h>


#ifdef __cplusplus
extern "C" {
#endif


class DCM {
private:

    estimation estimator;
    UTILITIES utility;

public:

    void calculate_chaser_frame(double pos_rel[3], double velocity[3], double chaser_i[3][3]);
    void calculate_body_chaser(double chaser_frame[3][3], double Ti2b[3][3], double body_chaser[3][3]);
    void calculate_LVLH_i(double position[3], double velocity[3], double LVLH_i[3][3]);
    void calculate_body_LVLH(double LVLH_i[3][3], double body_i[3][3], double body_lvlh[3][3]);

    void T1(double angle, double T1_mat[3][3]);
    void T2(double angle, double T2_mat[3][3]);
    void T3(double angle, double T3_mat[3][3]);

};

#ifdef __cplusplus
}
#endif

#endif

