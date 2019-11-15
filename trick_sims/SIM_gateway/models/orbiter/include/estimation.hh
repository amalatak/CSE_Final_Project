/*************************************************************************
PURPOSE: ( Orbiter Attitude Model )
**************************************************************************/
#ifndef ESTIMATION_H
#define ESTIMATION_H

#include <iostream>
#include <vector>
#include <math.h>
#include "../include/UTILITIES.hh"


#ifdef __cplusplus
extern "C" {
#endif


class estimation {
private:
    UTILITIES utility;


public:

    /* ESTIMATION */
    void TRIAD(double vec1[3], double vec2[3], double frame_est[3][3]);
    void camera();
};

#ifdef __cplusplus
}
#endif

#endif

