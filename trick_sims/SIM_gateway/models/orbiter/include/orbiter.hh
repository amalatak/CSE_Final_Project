/*************************************************************************
PURPOSE: (Represent the state and initial conditions of a cannonball)
**************************************************************************/
#ifndef ORBITER_H
#define ORBITER_H

#include "ATTITUDE.hh"

typedef struct {
    double r_mag;
    double off_angle;

    /* TARGET */
    double target_vel0[3] ;    /* *i m Init velocity of chaser */
    double target_pos0[3] ;    /* *i m Init position of chaser */

    double target_acc[3] ;     /* m/s2 xyz-acceleration  */
    double target_vel[3] ;     /* m/s xyz-velocity */
    double target_pos[3] ;     /* m xyz-position */

    /* CHASER */
    double chaser_vel0[3] ;    /* *i m Init velocity of chaser */
    double chaser_pos0[3] ;    /* *i m Init position of chaser */

    double chaser_acc[3] ;     /* m/s2 xyz-acceleration  */
    double chaser_vel[3] ;     /* m/s xyz-velocity */
    double chaser_pos[3] ;     /* m xyz-position */

    double time;        /* s Model time */
    double mu;          /* m3/s2 moon standard gravitational parameter */

    ATTITUDE chaser;
    ATTITUDE target;


} ORBIT_SYSTEM ;

#ifdef __cplusplus
extern "C" {
#endif
    int orbit_system_default_data(ORBIT_SYSTEM*) ;
    int orbit_system_init(ORBIT_SYSTEM*) ;
    int orbit_system_shutdown(ORBIT_SYSTEM*) ;
#ifdef __cplusplus
}
#endif

#endif
