#ifndef ORBIT_UTILS_H
#define ORBIT_UTILS_H

typedef struct {

    double vel0[3] ;    /* *i m Init velocity of orbiter */
    double pos0[3] ;    /* *i m Init position of orbiter */
    double oe0[6] ;      /* orbital elements of orbiter */
    double rv0[6] ;      /* initial pv vector */

    double acc[3] ;     /* m/s2 xyz-acceleration  */
    double vel[3] ;     /* m/s xyz-velocity */
    double pos[3] ;     /* m xyz-position */

    double time;        /* s Model time */
    double mu;          /* m3/s2 moon standard gravitational parameter */

} ORBIT ;

#ifdef __cplusplus
extern "C" {
#endif

double * rv2oe(double rv[], double mu) ;

int orbiter_default_data(ORBIT*) ;
int orbit_init(ORBIT*) ;
int orbit_shutdown(ORBIT*) ;

#ifdef __cplusplus
}
#endif

#endif

