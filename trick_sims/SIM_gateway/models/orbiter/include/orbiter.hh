/*************************************************************************
PURPOSE: (Represent the state and initial conditions of a cannonball)
**************************************************************************/
#ifndef ORBITER_H
#define ORBITER_H

typedef struct {

    double vel0[3] ;    /* *i m Init velocity of orbiter */
    double pos0[3] ;    /* *i m Init position of orbiter */

    double acc[3] ;     /* m/s2 xyz-acceleration  */
    double vel[3] ;     /* m/s xyz-velocity */
    double pos[3] ;     /* m xyz-position */

    double time;        /* s Model time */
    double mu;          /* m3/s2 moon standard gravitational parameter */

} ORBITER ;

#ifdef __cplusplus
extern "C" {
#endif
    int orbiter_default_data(ORBITER*) ;
    int orbiter_init(ORBITER*) ;
    int orbiter_shutdown(ORBITER*) ;
#ifdef __cplusplus
}
#endif

#endif
