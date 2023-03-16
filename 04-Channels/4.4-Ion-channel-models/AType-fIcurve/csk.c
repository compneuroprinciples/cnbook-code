/* Created by Language version: 7.7.0 */
/* NOT VECTORIZED */
#define NRN_VECTORIZED 0
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
 
#define nrn_init _nrn_init__csk
#define _nrn_initial _nrn_initial__csk
#define nrn_cur _nrn_cur__csk
#define _nrn_current _nrn_current__csk
#define nrn_jacob _nrn_jacob__csk
#define nrn_state _nrn_state__csk
#define _net_receive _net_receive__csk 
#define states states__csk 
 
#define _threadargscomma_ /**/
#define _threadargsprotocomma_ /**/
#define _threadargs_ /**/
#define _threadargsproto_ /**/
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *getarg();
 static double *_p; static Datum *_ppvar;
 
#define t nrn_threads->_t
#define dt nrn_threads->_dt
#define gbar _p[0]
#define i _p[1]
#define g _p[2]
#define n _p[3]
#define Dn _p[4]
#define ek _p[5]
#define ik _p[6]
#define _g _p[7]
#define _ion_ek	*_ppvar[0]._pval
#define _ion_ik	*_ppvar[1]._pval
#define _ion_dikdv	*_ppvar[2]._pval
 
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
 /* external NEURON variables */
 extern double celsius;
 /* declaration of user functions */
 static void _hoc_alphan(void);
 static void _hoc_betan(void);
 static void _hoc_vtrap(void);
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
 _p = _prop->param; _ppvar = _prop->dparam;
 }
 static void _hoc_setdata() {
 Prop *_prop, *hoc_getdata_range(int);
 _prop = hoc_getdata_range(_mechtype);
   _setdata(_prop);
 hoc_retpushx(1.);
}
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 "setdata_csk", _hoc_setdata,
 "alphan_csk", _hoc_alphan,
 "betan_csk", _hoc_betan,
 "vtrap_csk", _hoc_vtrap,
 0, 0
};
#define alphan alphan_csk
#define betan betan_csk
#define vtrap vtrap_csk
 extern double alphan( double );
 extern double betan( double );
 extern double vtrap( double , double );
 /* declare global and static user variables */
#define Qten Qten_csk
 double Qten = 3.13;
#define Q Q_csk
 double Q = 0;
#define celsius_exp celsius_exp_csk
 double celsius_exp = 6.3;
#define nshift nshift_csk
 double nshift = 0.7;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "celsius_exp_csk", "degC",
 "nshift_csk", "mV",
 "gbar_csk", "S/cm2",
 "i_csk", "mA/cm2",
 "g_csk", "S/cm2",
 0,0
};
 static double delta_t = 0.01;
 static double n0 = 0;
 static double v = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "Qten_csk", &Qten_csk,
 "celsius_exp_csk", &celsius_exp_csk,
 "nshift_csk", &nshift_csk,
 "Q_csk", &Q_csk,
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
 
#define _cvode_ieq _ppvar[3]._i
 static void _ode_matsol_instance1(_threadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"csk",
 "gbar_csk",
 0,
 "i_csk",
 "g_csk",
 0,
 "n_csk",
 0,
 0};
 static Symbol* _k_sym;
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
 	_p = nrn_prop_data_alloc(_mechtype, 8, _prop);
 	/*initialize range parameters*/
 	gbar = 0.02;
 	_prop->param = _p;
 	_prop->param_size = 8;
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 4, _prop);
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_k_sym);
 nrn_promote(prop_ion, 0, 1);
 	_ppvar[0]._pval = &prop_ion->param[0]; /* ek */
 	_ppvar[1]._pval = &prop_ion->param[3]; /* ik */
 	_ppvar[2]._pval = &prop_ion->param[4]; /* _ion_dikdv */
 
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

 void _csk_reg() {
	int _vectorized = 0;
  _initlists();
 	ion_reg("k", -10000.);
 	_k_sym = hoc_lookup("k_ion");
 	register_mech(_mechanism, nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init, hoc_nrnpointerindex, 0);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
     _nrn_thread_reg(_mechtype, 2, _update_ion_pointer);
 #if NMODL_TEXT
  hoc_reg_nmodl_text(_mechtype, nmodl_file_text);
  hoc_reg_nmodl_filename(_mechtype, nmodl_filename);
#endif
  hoc_register_prop_size(_mechtype, 8, 4);
  hoc_register_dparam_semantics(_mechtype, 0, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 1, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 2, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 3, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 csk C:/Users/bpg/Desktop/Projects/CN Schools/BNNI/2021/Material/Code/Running/FIcurve/csk.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static char *modelname = "cska.mod - Crustatean delayed rectifier potassium channel";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static int _slist1[1], _dlist1[1];
 static int states(_threadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 () {_reset=0;
 {
   Dn = Q / 2.0 * ( ( 1.0 - n ) * alphan ( _threadargscomma_ v ) - n * betan ( _threadargscomma_ v ) ) ;
   }
 return _reset;
}
 static int _ode_matsol1 () {
 Dn = Dn  / (1. - dt*( ( Q / 2.0 )*( ( ( ( ( - 1.0 ) ) )*( alphan ( _threadargscomma_ v ) ) - ( 1.0 )*( betan ( _threadargscomma_ v ) ) ) ) )) ;
  return 0;
}
 /*END CVODE*/
 static int states () {_reset=0;
 {
    n = n + (1. - exp(dt*(( Q / 2.0 )*( ( ( ( ( - 1.0 ) ) )*( alphan ( _threadargscomma_ v ) ) - ( 1.0 )*( betan ( _threadargscomma_ v ) ) ) ))))*(- ( ( ( Q ) / 2.0 )*( ( ( ( 1.0 ) )*( alphan ( _threadargscomma_ v ) ) ) ) ) / ( ( ( Q ) / 2.0 )*( ( ( ( ( - 1.0 ) ) )*( alphan ( _threadargscomma_ v ) ) - ( 1.0 )*( betan ( _threadargscomma_ v ) ) ) ) ) - n) ;
   }
  return 0;
}
 
double alphan (  double _lVm ) {
   double _lalphan;
 _lalphan = 0.01 * vtrap ( _threadargscomma_ - ( _lVm + 50.0 + nshift ) , 10.0 ) ;
   
return _lalphan;
 }
 
static void _hoc_alphan(void) {
  double _r;
   _r =  alphan (  *getarg(1) );
 hoc_retpushx(_r);
}
 
double betan (  double _lVm ) {
   double _lbetan;
 _lbetan = 0.125 * exp ( - ( _lVm + 60.0 + nshift ) / 80.0 ) ;
   
return _lbetan;
 }
 
static void _hoc_betan(void) {
  double _r;
   _r =  betan (  *getarg(1) );
 hoc_retpushx(_r);
}
 
double vtrap (  double _lx , double _ly ) {
   double _lvtrap;
 if ( fabs ( _lx / _ly ) < 1e-6 ) {
     _lvtrap = _ly * ( 1.0 - _lx / _ly / 2.0 ) ;
     }
   else {
     _lvtrap = _lx / ( exp ( _lx / _ly ) - 1.0 ) ;
     }
   
return _lvtrap;
 }
 
static void _hoc_vtrap(void) {
  double _r;
   _r =  vtrap (  *getarg(1) , *getarg(2) );
 hoc_retpushx(_r);
}
 
static int _ode_count(int _type){ return 1;}
 
static void _ode_spec(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
  ek = _ion_ek;
     _ode_spec1 ();
  }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 1; ++_i) {
		_pv[_i] = _pp + _slist1[_i];  _pvdot[_i] = _pp + _dlist1[_i];
		_cvode_abstol(_atollist, _atol, _i);
	}
 }
 
static void _ode_matsol_instance1(_threadargsproto_) {
 _ode_matsol1 ();
 }
 
static void _ode_matsol(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
  ek = _ion_ek;
 _ode_matsol_instance1(_threadargs_);
 }}
 extern void nrn_update_ion_pointer(Symbol*, Datum*, int, int);
 static void _update_ion_pointer(Datum* _ppvar) {
   nrn_update_ion_pointer(_k_sym, _ppvar, 0, 0);
   nrn_update_ion_pointer(_k_sym, _ppvar, 1, 3);
   nrn_update_ion_pointer(_k_sym, _ppvar, 2, 4);
 }

static void initmodel() {
  int _i; double _save;_ninits++;
 _save = t;
 t = 0.0;
{
  n = n0;
 {
   n = alphan ( _threadargscomma_ v ) / ( alphan ( _threadargscomma_ v ) + betan ( _threadargscomma_ v ) ) ;
   Q = pow( Qten , ( ( celsius - celsius_exp ) / 10.0 ) ) ;
   }
  _sav_indep = t; t = _save;

}
}

static void nrn_init(_NrnThread* _nt, _Memb_list* _ml, int _type){
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
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
  ek = _ion_ek;
 initmodel();
 }}

static double _nrn_current(double _v){double _current=0.;v=_v;{ {
   g = gbar * pow( n , 4.0 ) ;
   i = g * ( v - ek ) ;
   ik = i ;
   }
 _current += ik;

} return _current;
}

static void nrn_cur(_NrnThread* _nt, _Memb_list* _ml, int _type){
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
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
  ek = _ion_ek;
 _g = _nrn_current(_v + .001);
 	{ double _dik;
  _dik = ik;
 _rhs = _nrn_current(_v);
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
 
}}

static void nrn_jacob(_NrnThread* _nt, _Memb_list* _ml, int _type){
Node *_nd; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
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
 
}}

static void nrn_state(_NrnThread* _nt, _Memb_list* _ml, int _type){
Node *_nd; double _v = 0.0; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
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
  ek = _ion_ek;
 { error =  states();
 if(error){fprintf(stderr,"at line 40 in file csk.mod:\n    SOLVE states METHOD cnexp\n"); nrn_complain(_p); abort_run(error);}
 } }}

}

static void terminal(){}

static void _initlists() {
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = &(n) - _p;  _dlist1[0] = &(Dn) - _p;
_first = 0;
}

#if NMODL_TEXT
static const char* nmodl_filename = "csk.mod";
static const char* nmodl_file_text = 
  "TITLE cska.mod - Crustatean delayed rectifier potassium channel\n"
  "\n"
  "COMMENT \n"
  "\n"
  "\"Connor and Stevens\" model for sodium channel from Connor,\n"
  "Walter and McKown (Biophys. J. 18:81-102, 1977).\n"
  "\n"
  "ENDCOMMENT\n"
  "\n"
  "NEURON {\n"
  "    SUFFIX csk\n"
  "    USEION k READ ek WRITE ik\n"
  "    RANGE gbar, g, i, n\n"
  "    GLOBAL Q\n"
  "}\n"
  "\n"
  "PARAMETER {\n"
  "    gbar = 0.020 (S/cm2)\n"
  "    Qten = 3.13                         : Adjusted to get the CS-factor of \n"
  "    celsius_exp = 6.3 (degC)            : Temperature at which\n"
  "    : experiments performed\n"
  "    nshift = 0.7 (mV)                   :  CS shift and shift to match HH  =-4.3 + 5 \n"
  "}\n"
  "\n"
  "STATE {\n"
  "    n\n"
  "}\n"
  "\n"
  "ASSIGNED {\n"
  "    v (mV)\n"
  "    ek (mV)\n"
  "    i (mA/cm2)\n"
  "    ik (mA/cm2)\n"
  "    g (S/cm2)\n"
  "    celsius (degC)\n"
  "    Q\n"
  "}\n"
  "\n"
  "BREAKPOINT {\n"
  "    SOLVE states METHOD cnexp\n"
  "    g = gbar * n^4\n"
  "    i = g * (v - ek)\n"
  "    ik = i \n"
  "}\n"
  "\n"
  "INITIAL {\n"
  "    n = alphan(v)/(alphan(v) + betan(v))\n"
  "    Q = Qten^((celsius - celsius_exp)/10)\n"
  "}\n"
  "\n"
  "DERIVATIVE states {\n"
  "    n' = Q / 2 * ((1 - n)*alphan(v) - n*betan(v))\n"
  "}\n"
  "\n"
  "FUNCTION alphan(Vm (mV)) (/ms) {\n"
  "    alphan = 0.01 * vtrap(-(Vm + 50 + nshift), 10)\n"
  "}\n"
  "\n"
  "FUNCTION betan(Vm (mV)) (/ms) {\n"
  "    betan = 0.125 * exp(-(Vm + 60 + nshift)/80)\n"
  "}\n"
  "\n"
  "FUNCTION vtrap(x (mV) ,y (mV)) (mV) {  :Traps for 0 in denominator of rate eqns.\n"
  "        if (fabs(x/y) < 1e-6) {\n"
  "                vtrap = y*(1 - x/y/2)\n"
  "        }else{\n"
  "                vtrap = x/(exp(x/y) - 1)\n"
  "        }\n"
  "}\n"
  ;
#endif
