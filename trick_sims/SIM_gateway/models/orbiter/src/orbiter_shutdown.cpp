/************************************************************************
PURPOSE: (Print the final orbiter state.)
*************************************************************************/
#include <stdio.h>
#include "../include/orbiter.hh"
#include "trick/exec_proto.h"

int orbiter_shutdown( ORBITER* C) {
    double t = exec_get_sim_time();
    printf( "========================================\n");
    printf( "        Orbiter State at Shutdown       \n");
    printf( "t = %g\n", t);
    printf( "pos = [%.9f, %.9f, %.9f]\n", C->pos[0], C->pos[1], C->pos[2]);
    printf( "vel = [%.9f, %.9f, %.9f]\n", C->vel[0], C->vel[1], C->vel[2]);
    printf( "========================================\n");
    return 0 ;
}
