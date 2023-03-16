/* Created by Language version: 7.7.0 */
/* VECTORIZED */
#define NRN_VECTORIZED 1
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "scoplib_ansi.h"
#undef PI
#define nil 0
#include "md1redef.h"
#include "section.h"
#include "nrniv_mf.h"
#include "md2redef.h"
 
#if METHOD3
extern int _method3;
#endif

#if !NRNGPU
#undef exp
#define exp hoc_Exp
extern double hoc_Exp(double);
#endif
 
#define nrn_init _nrn_init__cska
#define _nrn_initial _nrn_initial__cska
#define nrn_cur _nrn_cur__cska
#define _nrn_current _nrn_current__cska
#define nrn_jacob _nrn_jacob__cska
#define nrn_state _nrn_state__cska
#define _net_receive _net_receive__cska 
#define states states__cska 
 
#define _threadargscomma_ _p, _ppvar, _thread, _nt,
#define _threadargsprotocomma_ double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt,
#define _threadargs_ _p, _ppvar, _thread, _nt
#define _threadargsproto_ double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *getarg();
 /* Thread safe. No static _p or _ppvar. */
 
#define t _nt->_t
#define dt _nt->_dt
#define gbar _p[0]
#define i _p[1]
#define g _p[2]
#define A _p[3]
#define B _p[4]
#define DA _p[5]
#define DB _p[6]
#define ik _p[7]
#define v _p[8]
#define _g _p[9]
#define _ion_ik	*_ppvar[0]._pval
#define _ion_dikdv	*_ppvar[1]._pval
 
#if MAC
#if !defined(v)
#define v _mlhv
#endif
#if !defined(h)
#define h _mlhh
#endif
#endif
 
#if defined(__cplusplus)
extern "C" {
#endif
 static int hoc_nrnpointerindex =  -1;
 static Datum* _extcall_thread;
 static Prop* _extcall_prop;
 /* external NEURON variables */
 extern double celsius;
 /* declaration of user functions */
 static void _hoc_Ainf(void);
 static void _hoc_Binf(void);
 static void _hoc_tauB(void);
 static void _hoc_tauA(void);
 static int _mechtype;
extern void _nrn_cacheloop_reg(int, int);
extern void hoc_register_prop_size(int, int, int);
extern void hoc_register_limits(int, HocParmLimits*);
extern void hoc_register_units(int, HocParmUnits*);
extern void nrn_promote(Prop*, int, int);
extern Memb_func* memb_func;
 
#define NMODL_TEXT 1
#if NMODL_TEXT
static const char* nmodl_file_text;
static const char* nmodl_filename;
extern void hoc_reg_nmodl_text(int, const char*);
extern void hoc_reg_nmodl_filename(int, const char*);
#endif

 extern void _nrn_setdata_reg(int, void(*)(Prop*));
 static void _setdata(Prop* _prop) {
 _extcall_prop = _prop;
 }
 static void _hoc_setdata() {
 Prop *_prop, *hoc_getdata_range(int);
 _prop = hoc_getdata_range(_mechtype);
   _setdata(_prop);
 hoc_retpushx(1.);
}
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 "setdata_cska", _hoc_setdata,
 "Ainf_cska", _hoc_Ainf,
 "Binf_cska", _hoc_Binf,
 "tauB_cska", _hoc_tauB,
 "tauA_cska", _hoc_tauA,
 0, 0
};
#define Ainf Ainf_cska
#define Binf Binf_cska
#define tauB tauB_cska
#define tauA tauA_cska
 extern double Ainf( _threadargsprotocomma_ double );
 extern double Binf( _threadargsprotocomma_ double );
 extern double tauB( _threadargsprotocomma_ double );
 extern double tauA( _threadargsprotocomma_ double );
 /* declare global and static user variables */
#define e e_cska
 double e = -80;
#define shift shift_cska
 double shift = 5;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "e_cska", "mV",
 "shift_cska", "mV",
 "gbar_cska", "S/cm2",
 "i_cska", "mA/cm2",
 "g_cska", "S/cm2",
 0,0
};
 static double A0 = 0;
 static double B0 = 0;
 static double delta_t = 0.01;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "e_cska", &e_cska,
 "shift_cska", &shift_cska,
 0,0
};
 static DoubVec hoc_vdoub[] = {
 0,0,0
};
 static double _sav_indep;
 static void nrn_alloc(Prop*);
static void  nrn_init(_NrnThread*, _Memb_list*, int);
static void nrn_state(_NrnThread*, _Memb_list*, int);
 static void nrn_cur(_NrnThread*, _Memb_list*, int);
static void  nrn_jacob(_NrnThread*, _Memb_list*, int);
 
static int _ode_count(int);
static void _ode_map(int, double**, double**, double*, Datum*, double*, int);
static void _ode_spec(_NrnThread*, _Memb_list*, int);
static void _ode_matsol(_NrnThread*, _Memb_list*, int);
 
#define _cvode_ieq _ppvar[2]._i
 static void _ode_matsol_instance1(_threadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"cska",
 "gbar_cska",
 0,
 "i_cska",
 "g_cska",
 0,
 "A_cska",
 "B_cska",
 0,
 0};
 static Symbol* _k_sym;
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
 	_p = nrn_prop_data_alloc(_mechtype, 10, _prop);
 	/*initialize range parameters*/
 	gbar = 0.0477;
 	_prop->param = _p;
 	_prop->param_size = 10;
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 3, _prop);
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_k_sym);
 	_ppvar[0]._pval = &prop_ion->param[3]; /* ik */
 	_ppvar[1]._pval = &prop_ion->param[4]; /* _ion_dikdv */
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 0,0
};
 static void _update_ion_pointer(Datum*);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, _NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _cska_reg() {
	int _vectorized = 1;
  _initlists();
 	ion_reg("k", -10000.);
 	_k_sym = hoc_lookup("k_ion");
 	register_mech(_mechanism, nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init, hoc_nrnpointerindex, 1);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
     _nrn_thread_reg(_mechtype, 2, _update_ion_pointer);
 #if NMODL_TEXT
  hoc_reg_nmodl_text(_mechtype, nmodl_file_text);
  hoc_reg_nmodl_filename(_mechtype, nmodl_filename);
#endif
  hoc_register_prop_size(_mechtype, 10, 3);
  hoc_register_dparam_semantics(_mechtype, 0, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 1, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 2, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 cska C:/Users/bpg/Desktop/Projects/CN Schools/BNNI/2021/Material/Code/Running/FIcurve/cska.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static char *modelname = "cska.mod - Crustatean A-type potassium channel";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static int _slist1[2], _dlist1[2];
 static int states(_threadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {int _reset = 0; {
   DA = ( Ainf ( _threadargscomma_ v ) - A ) / tauA ( _threadargscomma_ v ) ;
   DB = ( Binf ( _threadargscomma_ v ) - B ) / tauB ( _threadargscomma_ v ) ;
   }
 return _reset;
}
 static int _ode_matsol1 (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
 DA = DA  / (1. - dt*( ( ( ( - 1.0 ) ) ) / tauA ( _threadargscomma_ v ) )) ;
 DB = DB  / (1. - dt*( ( ( ( - 1.0 ) ) ) / tauB ( _threadargscomma_ v ) )) ;
  return 0;
}
 /*END CVODE*/
 static int states (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) { {
    A = A + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / tauA ( _threadargscomma_ v ))))*(- ( ( ( Ainf ( _threadargscomma_ v ) ) ) / tauA ( _threadargscomma_ v ) ) / ( ( ( ( - 1.0 ) ) ) / tauA ( _threadargscomma_ v ) ) - A) ;
    B = B + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / tauB ( _threadargscomma_ v ))))*(- ( ( ( Binf ( _threadargscomma_ v ) ) ) / tauB ( _threadargscomma_ v ) ) / ( ( ( ( - 1.0 ) ) ) / tauB ( _threadargscomma_ v ) ) - B) ;
   }
  return 0;
}
 
double Ainf ( _threadargsprotocomma_ double _lVm ) {
   double _lAinf;
 _lAinf = pow( ( 0.0761 * exp ( ( _lVm + shift + 94.22 ) / 31.84 ) / ( 1.0 + exp ( ( _lVm + shift + 1.17 ) / 28.93 ) ) ) , ( 1.0 / 3.0 ) ) ;
   
return _lAinf;
 }
 
static void _hoc_Ainf(void) {
  double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r =  Ainf ( _p, _ppvar, _thread, _nt, *getarg(1) );
 hoc_retpushx(_r);
}
 
double tauA ( _threadargsprotocomma_ double _lVm ) {
   double _ltauA;
 _ltauA = 0.3632 + 1.158 / ( 1.0 + exp ( ( _lVm + shift + 55.96 ) / 20.12 ) ) ;
   
return _ltauA;
 }
 
static void _hoc_tauA(void) {
  double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r =  tauA ( _p, _ppvar, _thread, _nt, *getarg(1) );
 hoc_retpushx(_r);
}
 
double Binf ( _threadargsprotocomma_ double _lVm ) {
   double _lBinf;
 _lBinf = 1.0 / pow( ( 1.0 + exp ( ( _lVm + shift + 53.3 ) / 14.54 ) ) , 4.0 ) ;
   
return _lBinf;
 }
 
static void _hoc_Binf(void) {
  double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r =  Binf ( _p, _ppvar, _thread, _nt, *getarg(1) );
 hoc_retpushx(_r);
}
 
double tauB ( _threadargsprotocomma_ double _lVm ) {
   double _ltauB;
 _ltauB = 1.24 + 2.678 / ( 1.0 + exp ( ( _lVm - shift + 50.0 ) / 16.027 ) ) ;
   
return _ltauB;
 }
 
static void _hoc_tauB(void) {
  double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r =  tauB ( _p, _ppvar, _thread, _nt, *getarg(1) );
 hoc_retpushx(_r);
}
 
static int _ode_count(int _type){ return 2;}
 
static void _ode_spec(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
     _ode_spec1 (_p, _ppvar, _thread, _nt);
  }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
	double* _p; Datum* _ppvar;
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 2; ++_i) {
		_pv[_i] = _pp + _slist1[_i];  _pvdot[_i] = _pp + _dlist1[_i];
		_cvode_abstol(_atollist, _atol, _i);
	}
 }
 
static void _ode_matsol_instance1(_threadargsproto_) {
 _ode_matsol1 (_p, _ppvar, _thread, _nt);
 }
 
static void _ode_matsol(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
 _ode_matsol_instance1(_threadargs_);
 }}
 extern void nrn_update_ion_pointer(Symbol*, Datum*, int, int);
 static void _update_ion_pointer(Datum* _ppvar) {
   nrn_update_ion_pointer(_k_sym, _ppvar, 0, 3);
   nrn_update_ion_pointer(_k_sym, _ppvar, 1, 4);
 }

static void initmodel(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
  int _i; double _save;{
  A = A0;
  B = B0;
 {
   A = Ainf ( _threadargscomma_ v ) ;
   B = Binf ( _threadargscomma_ v ) ;
   }
 
}
}

static void nrn_init(_NrnThread* _nt, _Memb_list* _ml, int _type){
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v = _v;
 initmodel(_p, _ppvar, _thread, _nt);
 }
}

static double _nrn_current(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt, double _v){double _current=0.;v=_v;{ {
   g = gbar * pow( A , 3.0 ) * B ;
   i = g * ( v - e ) ;
   ik = i ;
   }
 _current += ik;

} return _current;
}

static void nrn_cur(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 _g = _nrn_current(_p, _ppvar, _thread, _nt, _v + .001);
 	{ double _dik;
  _dik = ik;
 _rhs = _nrn_current(_p, _ppvar, _thread, _nt, _v);
  _ion_dikdv += (_dik - ik)/.001 ;
 	}
 _g = (_g - _rhs)/.001;
  _ion_ik += ik ;
#if CACHEVEC
  if (use_cachevec) {
	VEC_RHS(_ni[_iml]) -= _rhs;
  }else
#endif
  {
	NODERHS(_nd) -= _rhs;
  }
 
}
 
}

static void nrn_jacob(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml];
#if CACHEVEC
  if (use_cachevec) {
	VEC_D(_ni[_iml]) += _g;
  }else
#endif
  {
     _nd = _ml->_nodelist[_iml];
	NODED(_nd) += _g;
  }
 
}
 
}

static void nrn_state(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v = 0.0; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
 _nd = _ml->_nodelist[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v=_v;
{
 {   states(_p, _ppvar, _thread, _nt);
  } }}

}

static void terminal(){}

static void _initlists(){
 double _x; double* _p = &_x;
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = &(A) - _p;  _dlist1[0] = &(DA) - _p;
 _slist1[1] = &(B) - _p;  _dlist1[1] = &(DB) - _p;
_first = 0;
}

#if defined(__cplusplus)
} /* extern "C" */
#endif

#if NMODL_TEXT
static const char* nmodl_filename = "cska.mod";
static const char* nmodl_file_text = 
  "TITLE cska.mod - Crustatean A-type potassium channel\n"
  "\n"
  "COMMENT \n"
  "\n"
  "\"Connor and Stevens\" model for A-type potassium from Connor,\n"
  "Walter and McKown (Biophys. J. 18:81-102, 1977).\n"
  "\n"
  "ENDCOMMENT\n"
  "\n"
  "NEURON {\n"
  "    SUFFIX cska\n"
  "    USEION k WRITE ik\n"
  "    RANGE gbar, g, i\n"
  "    GLOBAL e\n"
  "}\n"
  "\n"
  "PARAMETER {\n"
  "    gbar = 0.0477 (S/cm2)\n"
  "    e = -80 (mV)                        : CS minus 5mV to match HH\n"
  "    shift = 5 (mV)\n"
  "}\n"
  "\n"
  "STATE {\n"
  "    A B\n"
  "}\n"
  "\n"
  "ASSIGNED {\n"
  "    v (mV)\n"
  "    i (mA/cm2)\n"
  "    ik (mA/cm2)\n"
  "    g (S/cm2)\n"
  "    celsius (degC)                      : These values valid at 18degC\n"
  "}\n"
  "\n"
  "BREAKPOINT {\n"
  "    SOLVE states METHOD cnexp\n"
  "    g = gbar * A^3 * B\n"
  "    i = g * (v - e)\n"
  "    ik =  i\n"
  "}\n"
  "\n"
  "INITIAL {\n"
  "    A = Ainf(v)\n"
  "    B = Binf(v)\n"
  "}\n"
  "\n"
  "DERIVATIVE states {\n"
  "    A' =  (Ainf(v) - A)/tauA(v)\n"
  "    B' =  (Binf(v) - B)/tauB(v)\n"
  "}\n"
  "\n"
  "FUNCTION Ainf(Vm (mV)) {\n"
  "    Ainf = (0.0761*exp((Vm+shift+94.22(mV))/31.84(mV))/(1+exp((Vm+shift+1.17(mV))/28.93(mV))))^(1/3)\n"
  "}\n"
  "\n"
  "FUNCTION tauA(Vm (mV)) (ms) {\n"
  "    tauA = 0.3632(ms)  + 1.158(ms)/(1 + exp((Vm+shift + 55.96(mV))/20.12(mV)))\n"
  "}\n"
  "\n"
  "FUNCTION Binf(Vm (mV)) {\n"
  "    Binf = 1/(1 + exp((Vm+shift + 53.3(mV))/14.54(mV)))^4\n"
  "}\n"
  "\n"
  "FUNCTION tauB(Vm (mV)) (ms) {\n"
  "    tauB = 1.24(ms) + 2.678(ms)/(1 + exp((Vm-shift+50(mV))/16.027(mV)))\n"
  "}\n"
  ;
#endif
