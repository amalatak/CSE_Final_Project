#include <Python.h>
#if PY_VERSION_HEX >= 0x03000000
extern "C" {

PyObject * PyInit__m4fbc2b835aa824420d037a47086569f0(void) ; /* /home/amalatak/Documents/Programming/CSE_Final_Project/trick_sims/SIM_gateway/S_source.hh */
PyObject * PyInit__md0bdaa7eb6a286abc5bedec9685b5b98(void) ; /* /home/amalatak/Documents/Programming/CSE_Final_Project/trick_sims/SIM_gateway/models/orbiter/include/orbiter.hh */
PyObject * PyInit__m4e2ba54134d357f6443bf075f8a67623(void) ; /* /home/amalatak/Documents/Programming/CSE_Final_Project/trick_sims/SIM_gateway/models/orbiter/include/orbiter_numeric.hh */
PyObject * PyInit__sim_services(void) ;
PyObject * PyInit__top(void) ;
PyObject * PyInit__swig_double(void) ;
PyObject * PyInit__swig_int(void) ;
PyObject * PyInit__swig_ref(void) ;

void init_swig_modules(void) {

    PyImport_AppendInittab("_md0bdaa7eb6a286abc5bedec9685b5b98", PyInit__md0bdaa7eb6a286abc5bedec9685b5b98) ;
    PyImport_AppendInittab("_m4e2ba54134d357f6443bf075f8a67623", PyInit__m4e2ba54134d357f6443bf075f8a67623) ;
    PyImport_AppendInittab("_m4fbc2b835aa824420d037a47086569f0", PyInit__m4fbc2b835aa824420d037a47086569f0) ;
    PyImport_AppendInittab("_sim_services", PyInit__sim_services) ;
    PyImport_AppendInittab("_top", PyInit__top) ;
    PyImport_AppendInittab("_swig_double", PyInit__swig_double) ;
    PyImport_AppendInittab("_swig_int", PyInit__swig_int) ;
    PyImport_AppendInittab("_swig_ref", PyInit__swig_ref) ;
    return ;
}

}
#else
extern "C" {

void init_m4fbc2b835aa824420d037a47086569f0(void) ; /* /home/amalatak/Documents/Programming/CSE_Final_Project/trick_sims/SIM_gateway/S_source.hh */
void init_md0bdaa7eb6a286abc5bedec9685b5b98(void) ; /* /home/amalatak/Documents/Programming/CSE_Final_Project/trick_sims/SIM_gateway/models/orbiter/include/orbiter.hh */
void init_m4e2ba54134d357f6443bf075f8a67623(void) ; /* /home/amalatak/Documents/Programming/CSE_Final_Project/trick_sims/SIM_gateway/models/orbiter/include/orbiter_numeric.hh */
void init_sim_services(void) ;
void init_top(void) ;
void init_swig_double(void) ;
void init_swig_int(void) ;
void init_swig_ref(void) ;

void init_swig_modules(void) {

    init_md0bdaa7eb6a286abc5bedec9685b5b98() ;
    init_m4e2ba54134d357f6443bf075f8a67623() ;
    init_m4fbc2b835aa824420d037a47086569f0() ;
    init_sim_services() ;
    init_top() ;
    init_swig_double() ;
    init_swig_int() ;
    init_swig_ref() ;
    return ;
}

}
#endif
