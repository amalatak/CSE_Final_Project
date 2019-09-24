#include <Python.h>
#if PY_VERSION_HEX >= 0x03000000
extern "C" {

PyObject * PyInit__md38a7ef8dc1d324ace74a19a83805998(void) ; /* /home/amalatak/Documents/Programming/CSE_Final_Project/trick_sims/SIM_cannon_numeric/S_source.hh */
PyObject * PyInit__ma2404a1fbe417ddcf7e1f2dcd4bd85a3(void) ; /* /home/amalatak/Documents/Programming/CSE_Final_Project/trick_sims/SIM_cannon_numeric/models/cannon/include/cannon.h */
PyObject * PyInit__md90de8bdc9357de96137da391049d8b6(void) ; /* /home/amalatak/Documents/Programming/CSE_Final_Project/trick_sims/SIM_cannon_numeric/models/cannon/include/cannon_numeric.h */
PyObject * PyInit__sim_services(void) ;
PyObject * PyInit__top(void) ;
PyObject * PyInit__swig_double(void) ;
PyObject * PyInit__swig_int(void) ;
PyObject * PyInit__swig_ref(void) ;

void init_swig_modules(void) {

    PyImport_AppendInittab("_ma2404a1fbe417ddcf7e1f2dcd4bd85a3", PyInit__ma2404a1fbe417ddcf7e1f2dcd4bd85a3) ;
    PyImport_AppendInittab("_md90de8bdc9357de96137da391049d8b6", PyInit__md90de8bdc9357de96137da391049d8b6) ;
    PyImport_AppendInittab("_md38a7ef8dc1d324ace74a19a83805998", PyInit__md38a7ef8dc1d324ace74a19a83805998) ;
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

void init_md38a7ef8dc1d324ace74a19a83805998(void) ; /* /home/amalatak/Documents/Programming/CSE_Final_Project/trick_sims/SIM_cannon_numeric/S_source.hh */
void init_ma2404a1fbe417ddcf7e1f2dcd4bd85a3(void) ; /* /home/amalatak/Documents/Programming/CSE_Final_Project/trick_sims/SIM_cannon_numeric/models/cannon/include/cannon.h */
void init_md90de8bdc9357de96137da391049d8b6(void) ; /* /home/amalatak/Documents/Programming/CSE_Final_Project/trick_sims/SIM_cannon_numeric/models/cannon/include/cannon_numeric.h */
void init_sim_services(void) ;
void init_top(void) ;
void init_swig_double(void) ;
void init_swig_int(void) ;
void init_swig_ref(void) ;

void init_swig_modules(void) {

    init_ma2404a1fbe417ddcf7e1f2dcd4bd85a3() ;
    init_md90de8bdc9357de96137da391049d8b6() ;
    init_md38a7ef8dc1d324ace74a19a83805998() ;
    init_sim_services() ;
    init_top() ;
    init_swig_double() ;
    init_swig_int() ;
    init_swig_ref() ;
    return ;
}

}
#endif
