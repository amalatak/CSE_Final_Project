/******************************* TRICK HEADER ****************************
PURPOSE: (Set the initial data values)
*************************************************************************/

/* Model Include files */
#include <math.h>
#include <stdio.h>
#include <iostream>
#include "../include/orbiter.hh"

/* utility include file */
#include "../include/ATTITUDE.hh"

using namespace std;

/* default data job */
int orbiter_default_data( ORBITER* C ) {

    C->mu = 4.9048695e12;

    C->pos0[0] = 1900.0e3 ; // orbiter radius
    C->pos0[1] = 1000.0 ;
    C->pos0[2] = 1000.0 ;

    C->time = 0.0 ;

    C->SumOfMoments[0] = 0.0;
    C->SumOfMoments[1] = 0.0;
    C->SumOfMoments[2] = 0.1;

    C->attitude.set_Jmat(100.0, 0.0, 0.0,
                         0.0, 100.0, 0.0,
                         0.0, 0.0, 100.0);
    C->attitude.set_Jmat_inv();

    C->attitude.set_body_i(1.0, 0.0, 0.0,
                           0.0, 1.0, 0.0,
                           0.0, 0.0, 1.0);

    return 0 ;
}

/* initialization job */
int orbiter_init( ORBITER* C ) {
   
    C->pos[0] = C->pos0[0] ; 
    C->pos[1] = C->pos0[1] ;
    C->pos[2] = C->pos0[2] ;

    C->vel0[0] = 1;
    C->vel0[1] = sqrt(C->mu/(C->pos[0]));
    C->vel0[2] = 1;

    C->vel[0] = C->vel0[0] ; 
    C->vel[1] = C->vel0[1] ; 
    C->vel[2] = C->vel0[2] ; 

    C->acc[0] = -C->mu*C->pos[0]/sqrt(C->pos[0]*C->pos[0] + C->pos[1]*C->pos[1] + C->pos[2]*C->pos[2]);
    C->acc[1] = -C->mu*C->pos[1]/sqrt(C->pos[0]*C->pos[0] + C->pos[1]*C->pos[1] + C->pos[2]*C->pos[2]);
    C->acc[2] = -C->mu*C->pos[2]/sqrt(C->pos[0]*C->pos[0] + C->pos[1]*C->pos[1] + C->pos[2]*C->pos[2]);

    C->attitude.set_rv(C->pos, C->vel);
    C->attitude.calculate_LVLH_i();
    C->attitude.set_torques(C->SumOfMoments);

    return 0 ; 
}

