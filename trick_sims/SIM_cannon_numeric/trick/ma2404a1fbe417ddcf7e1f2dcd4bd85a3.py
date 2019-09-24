# This file was automatically generated by SWIG (http://www.swig.org).
# Version 3.0.8
#
# Do not make changes to this file unless you know what you are doing--modify
# the SWIG interface file instead.





from sys import version_info
if version_info >= (2, 6, 0):
    def swig_import_helper():
        from os.path import dirname
        import imp
        fp = None
        try:
            fp, pathname, description = imp.find_module('_ma2404a1fbe417ddcf7e1f2dcd4bd85a3', [dirname(__file__)])
        except ImportError:
            import _ma2404a1fbe417ddcf7e1f2dcd4bd85a3
            return _ma2404a1fbe417ddcf7e1f2dcd4bd85a3
        if fp is not None:
            try:
                _mod = imp.load_module('_ma2404a1fbe417ddcf7e1f2dcd4bd85a3', fp, pathname, description)
            finally:
                fp.close()
            return _mod
    _ma2404a1fbe417ddcf7e1f2dcd4bd85a3 = swig_import_helper()
    del swig_import_helper
else:
    import _ma2404a1fbe417ddcf7e1f2dcd4bd85a3
del version_info
try:
    _swig_property = property
except NameError:
    pass  # Python < 2.2 doesn't have 'property'.


def _swig_setattr_nondynamic(self, class_type, name, value, static=1):
    if (name == "thisown"):
        return self.this.own(value)
    if (name == "this"):
        if type(value).__name__ == 'SwigPyObject':
            self.__dict__[name] = value
            return
    method = class_type.__swig_setmethods__.get(name, None)
    if method:
        return method(self, value)
    if (not static):
        if _newclass:
            object.__setattr__(self, name, value)
        else:
            self.__dict__[name] = value
    else:
        raise AttributeError("You cannot add attributes to %s" % self)


def _swig_setattr(self, class_type, name, value):
    return _swig_setattr_nondynamic(self, class_type, name, value, 0)


def _swig_getattr_nondynamic(self, class_type, name, static=1):
    if (name == "thisown"):
        return self.this.own()
    method = class_type.__swig_getmethods__.get(name, None)
    if method:
        return method(self)
    if (not static):
        return object.__getattr__(self, name)
    else:
        raise AttributeError(name)

def _swig_getattr(self, class_type, name):
    return _swig_getattr_nondynamic(self, class_type, name, 0)


def _swig_repr(self):
    try:
        strthis = "proxy of " + self.this.__repr__()
    except Exception:
        strthis = ""
    return "<%s.%s; %s >" % (self.__class__.__module__, self.__class__.__name__, strthis,)

try:
    _object = object
    _newclass = 1
except AttributeError:
    class _object:
        pass
    _newclass = 0


class SwigPyIterator(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, SwigPyIterator, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, SwigPyIterator, name)

    def __init__(self, *args, **kwargs):
        raise AttributeError("No constructor defined - class is abstract")
    __repr__ = _swig_repr
    __swig_destroy__ = _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.delete_SwigPyIterator
    __del__ = lambda self: None

    def value(self):
        return _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.SwigPyIterator_value(self)

    def incr(self, n=1):
        return _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.SwigPyIterator_incr(self, n)

    def decr(self, n=1):
        return _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.SwigPyIterator_decr(self, n)

    def distance(self, x):
        return _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.SwigPyIterator_distance(self, x)

    def equal(self, x):
        return _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.SwigPyIterator_equal(self, x)

    def copy(self):
        return _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.SwigPyIterator_copy(self)

    def next(self):
        return _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.SwigPyIterator_next(self)

    def __next__(self):
        return _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.SwigPyIterator___next__(self)

    def previous(self):
        return _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.SwigPyIterator_previous(self)

    def advance(self, n):
        return _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.SwigPyIterator_advance(self, n)

    def __eq__(self, x):
        return _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.SwigPyIterator___eq__(self, x)

    def __ne__(self, x):
        return _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.SwigPyIterator___ne__(self, x)

    def __iadd__(self, n):
        return _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.SwigPyIterator___iadd__(self, n)

    def __isub__(self, n):
        return _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.SwigPyIterator___isub__(self, n)

    def __add__(self, n):
        return _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.SwigPyIterator___add__(self, n)

    def __sub__(self, *args):
        return _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.SwigPyIterator___sub__(self, *args)
    def __iter__(self):
        return self
SwigPyIterator_swigregister = _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.SwigPyIterator_swigregister
SwigPyIterator_swigregister(SwigPyIterator)


def _swig_getattr(self,class_type,name):
    if (name == "thisown"): return self.this.own()
    method = class_type.__swig_getmethods__.get(name,None)
    if method: return method(self)
    all_keys = [attr for attr in dir(class_type) if not attr.startswith('__')and attr != '_s' ]
    data_keys = sorted(class_type.__swig_setmethods__.keys())
    method_keys = [ x for x in all_keys if x not in data_keys ]
    raise AttributeError("Type %s does not contain member %s.\n%s data = %s\n%s methods = %s" %
     (self.__class__.__name__,name,self.__class__.__name__,data_keys,self.__class__.__name__,method_keys))

def _swig_setattr_nondynamic(self,class_type,name,value,static=1):
    if (name == "thisown"): return self.this.own(value)
    if (name == "this"):
# this line is changed to handle older swigs that used PySwigObject instead of the current SwigPyObject
        if type(value).__name__ == 'SwigPyObject' or type(value).__name__ == 'PySwigObject' :
            self.__dict__[name] = value
            return
    method = class_type.__swig_setmethods__.get(name,None)
    if method: return method(self,value)
    if (not static) or hasattr(self,name):
        self.__dict__[name] = value
    else:
        all_keys = [attr for attr in dir(class_type) if not attr.startswith('__') and attr != '_s' ]
        data_keys = sorted(class_type.__swig_setmethods__.keys())
        method_keys = [ x for x in all_keys if x not in data_keys ]
        raise AttributeError("Type %s does not contain member %s.\n%s data = %s\n%s methods = %s" %
         (self.__class__.__name__,name,self.__class__.__name__,data_keys,self.__class__.__name__,method_keys))

def _swig_setattr(self,class_type,name,value):
    return _swig_setattr_nondynamic(self,class_type,name,value,1)

class CANNON(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, CANNON, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, CANNON, name)
    __repr__ = _swig_repr

    def __getitem__(self, *args):
        return _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON___getitem__(self, *args)

    def __len__(self, *args):
        return _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON___len__(self, *args)
    __swig_setmethods__["vel0"] = _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON_vel0_set
    __swig_getmethods__["vel0"] = _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON_vel0_get
    if _newclass:
        vel0 = _swig_property(_ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON_vel0_get, _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON_vel0_set)
    __swig_setmethods__["pos0"] = _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON_pos0_set
    __swig_getmethods__["pos0"] = _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON_pos0_get
    if _newclass:
        pos0 = _swig_property(_ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON_pos0_get, _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON_pos0_set)
    __swig_setmethods__["init_speed"] = _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON_init_speed_set
    __swig_getmethods__["init_speed"] = _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON_init_speed_get
    if _newclass:
        init_speed = _swig_property(_ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON_init_speed_get, _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON_init_speed_set)
    __swig_setmethods__["init_angle"] = _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON_init_angle_set
    __swig_getmethods__["init_angle"] = _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON_init_angle_get
    if _newclass:
        init_angle = _swig_property(_ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON_init_angle_get, _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON_init_angle_set)
    __swig_setmethods__["acc"] = _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON_acc_set
    __swig_getmethods__["acc"] = _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON_acc_get
    if _newclass:
        acc = _swig_property(_ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON_acc_get, _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON_acc_set)
    __swig_setmethods__["vel"] = _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON_vel_set
    __swig_getmethods__["vel"] = _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON_vel_get
    if _newclass:
        vel = _swig_property(_ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON_vel_get, _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON_vel_set)
    __swig_setmethods__["pos"] = _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON_pos_set
    __swig_getmethods__["pos"] = _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON_pos_get
    if _newclass:
        pos = _swig_property(_ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON_pos_get, _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON_pos_set)
    __swig_setmethods__["time"] = _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON_time_set
    __swig_getmethods__["time"] = _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON_time_get
    if _newclass:
        time = _swig_property(_ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON_time_get, _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON_time_set)
    __swig_setmethods__["impact"] = _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON_impact_set
    __swig_getmethods__["impact"] = _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON_impact_get
    if _newclass:
        impact = _swig_property(_ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON_impact_get, _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON_impact_set)
    __swig_setmethods__["impactTime"] = _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON_impactTime_set
    __swig_getmethods__["impactTime"] = _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON_impactTime_get
    if _newclass:
        impactTime = _swig_property(_ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON_impactTime_get, _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON_impactTime_set)

    def __init__(self, **kwargs):
        import _sim_services
        this = _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.new_CANNON()
        try: self.this.append(this)
        except: self.this = this
        if 'TMMName' in kwargs:
            self.this.own(0)
            isThisInMM = _sim_services.get_alloc_info_at(this)
            if isThisInMM:
                _sim_services.set_alloc_name_at(this, kwargs['TMMName'])
            else:
                _sim_services.TMM_declare_ext_var(this, _sim_services.TRICK_STRUCTURED, "CANNON", 0, kwargs['TMMName'], 0, None)
            alloc_info = _sim_services.get_alloc_info_at(this)
            alloc_info.stcl = _sim_services.TRICK_LOCAL
            alloc_info.alloc_type = _sim_services.TRICK_ALLOC_NEW


    __swig_destroy__ = _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.delete_CANNON
    __del__ = lambda self: None
CANNON_swigregister = _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.CANNON_swigregister
CANNON_swigregister(CANNON)


def cannon_default_data(*args):
    return _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.cannon_default_data(*args)
cannon_default_data = _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.cannon_default_data

def cannon_init(*args):
    return _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.cannon_init(*args)
cannon_init = _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.cannon_init

def cannon_shutdown(*args):
    return _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.cannon_shutdown(*args)
cannon_shutdown = _ma2404a1fbe417ddcf7e1f2dcd4bd85a3.cannon_shutdown
# This file is compatible with both classic and new-style classes.


