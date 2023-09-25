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
 
#define nrn_init _nrn_init__ca2p
#define _nrn_initial _nrn_initial__ca2p
#define nrn_cur _nrn_cur__ca2p
#define _nrn_current _nrn_current__ca2p
#define nrn_jacob _nrn_jacob__ca2p
#define nrn_state _nrn_state__ca2p
#define _net_receive _net_receive__ca2p 
#define state state__ca2p 
 
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
#define cai0 _p[0]
#define rshell _p[1]
#define ca2i _p[2]
#define ca1 _p[3]
#define ca2 _p[4]
#define ica _p[5]
#define cai _p[6]
#define a12 _p[7]
#define v1 _p[8]
#define v2 _p[9]
#define df12 _p[10]
#define df21 _p[11]
#define chflux _p[12]
#define Dca1 _p[13]
#define Dca2 _p[14]
#define v _p[15]
#define _g _p[16]
#define _ion_cai	*_ppvar[0]._pval
#define _ion_ica	*_ppvar[1]._pval
#define _style_ca	*((int*)_ppvar[2]._pvoid)
#define diam	*_ppvar[3]._pval
 
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
 /* declaration of user functions */
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
 "setdata_ca2p", _hoc_setdata,
 0, 0
};
 /* declare global and static user variables */
 static int _thread1data_inuse = 0;
static double _thread1data[1];
#define _gth 4
#define DFree DFree_ca2p
 double DFree = 0.23;
#define bBu bBu_ca2p
 double bBu = 20;
#define r12_ca2p _thread1data[0]
#define r12 _thread[_gth]._pval[0]
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "DFree_ca2p", "um2/ms",
 "r12_ca2p", "um",
 "bBu_ca2p", "1",
 "cai0_ca2p", "mM",
 "rshell_ca2p", "um",
 "ca1_ca2p", "mM",
 "ca2_ca2p", "mM",
 "ca2i_ca2p", "mM",
 0,0
};
 static double ca20 = 0;
 static double ca10 = 0;
 static double delta_t = 0.1;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "DFree_ca2p", &DFree_ca2p,
 "r12_ca2p", &r12_ca2p,
 "bBu_ca2p", &bBu_ca2p,
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
 
#define _cvode_ieq _ppvar[4]._i
 static void _ode_synonym(int, double**, Datum**);
 static void _ode_matsol_instance1(_threadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"ca2p",
 "cai0_ca2p",
 "rshell_ca2p",
 0,
 "ca2i_ca2p",
 0,
 "ca1_ca2p",
 "ca2_ca2p",
 0,
 0};
 static Symbol* _morphology_sym;
 static Symbol* _ca_sym;
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
 	_p = nrn_prop_data_alloc(_mechtype, 17, _prop);
 	/*initialize range parameters*/
 	cai0 = 5e-05;
 	rshell = 1;
 	_prop->param = _p;
 	_prop->param_size = 17;
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 5, _prop);
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_morphology_sym);
 	_ppvar[3]._pval = &prop_ion->param[0]; /* diam */
 prop_ion = need_memb(_ca_sym);
 nrn_check_conc_write(_prop, prop_ion, 1);
 nrn_promote(prop_ion, 3, 0);
 	_ppvar[0]._pval = &prop_ion->param[1]; /* cai */
 	_ppvar[1]._pval = &prop_ion->param[3]; /* ica */
 	_ppvar[2]._pvoid = (void*)(&(prop_ion->dparam[0]._i)); /* iontype for ca */
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 0,0
};
 static void _thread_mem_init(Datum*);
 static void _thread_cleanup(Datum*);
 static void _update_ion_pointer(Datum*);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, _NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _ca2pool_reg() {
	int _vectorized = 1;
  _initlists();
 	ion_reg("ca", -10000.);
 	_morphology_sym = hoc_lookup("morphology");
 	_ca_sym = hoc_lookup("ca_ion");
 	register_mech(_mechanism, nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init, hoc_nrnpointerindex, 6);
  _extcall_thread = (Datum*)ecalloc(5, sizeof(Datum));
  _thread_mem_init(_extcall_thread);
  _thread1data_inuse = 0;
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
     _nrn_thread_reg(_mechtype, 1, _thread_mem_init);
     _nrn_thread_reg(_mechtype, 0, _thread_cleanup);
     _nrn_thread_reg(_mechtype, 2, _update_ion_pointer);
 #if NMODL_TEXT
  hoc_reg_nmodl_text(_mechtype, nmodl_file_text);
  hoc_reg_nmodl_filename(_mechtype, nmodl_filename);
#endif
  hoc_register_prop_size(_mechtype, 17, 5);
  hoc_register_dparam_semantics(_mechtype, 0, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 1, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 2, "#ca_ion");
  hoc_register_dparam_semantics(_mechtype, 4, "cvodeieq");
  hoc_register_dparam_semantics(_mechtype, 3, "diam");
 	nrn_writes_conc(_mechtype, 0);
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 	hoc_register_synonym(_mechtype, _ode_synonym);
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 ca2p C:/Users/bpg/Desktop/Projects/CN book/Edition2/Exercises/Code-Edn1/Chapt6/Fig6.5/ca2pool.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
 
#define FARADAY _nrnunit_FARADAY[_nrnunit_use_legacy_]
static double _nrnunit_FARADAY[2] = {0x9.a60645c954db8p+0, 9.64853}; /* 9.64853321233100125 */
 
#define PI _nrnunit_PI[_nrnunit_use_legacy_]
static double _nrnunit_PI[2] = {0xc.90fdaa22168cp-2, 3.14159}; /* 3.14159265358979312 */
static int _reset;
static char *modelname = "Ca concentration in submembrane and core shells.";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
 
#define _deriv1_advance _thread[0]._i
#define _dith1 1
#define _recurse _thread[2]._i
#define _newtonspace1 _thread[3]._pvoid
extern void* nrn_cons_newtonspace(int);
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static int _slist2[2];
  static int _slist1[2], _dlist1[2];
 static int state(_threadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {int _reset = 0; {
   chflux = - ica * diam / ( 2.0 * FARADAY * v1 ) ;
   Dca1 = ( chflux + df21 * ( ca2 - ca1 ) ) / ( bBu + 1.0 ) ;
   Dca2 = df12 * ( ca1 - ca2 ) / ( bBu + 1.0 ) ;
   cai = ca1 ;
   ca2i = ca2 ;
   }
 return _reset;
}
 static int _ode_matsol1 (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
 chflux = - ica * diam / ( 2.0 * FARADAY * v1 ) ;
 Dca1 = Dca1  / (1. - dt*( ( ( ( df21 )*( ( ( - 1.0 ) ) ) ) ) / ( bBu + 1.0 ) )) ;
 Dca2 = Dca2  / (1. - dt*( ( ( df12 )*( ( ( - 1.0 ) ) ) ) / ( bBu + 1.0 ) )) ;
 cai = ca1 ;
 ca2i = ca2 ;
  return 0;
}
 /*END CVODE*/
 
static int state (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {int _reset=0; int error = 0;
 { double* _savstate1 = _thread[_dith1]._pval;
 double* _dlist2 = _thread[_dith1]._pval + 2;
 int _counte = -1;
 if (!_recurse) {
 _recurse = 1;
 {int _id; for(_id=0; _id < 2; _id++) { _savstate1[_id] = _p[_slist1[_id]];}}
 error = nrn_newton_thread(_newtonspace1, 2,_slist2, _p, state, _dlist2, _ppvar, _thread, _nt);
 _recurse = 0; if(error) {abort_run(error);}}
 {
   chflux = - ica * diam / ( 2.0 * FARADAY * v1 ) ;
   Dca1 = ( chflux + df21 * ( ca2 - ca1 ) ) / ( bBu + 1.0 ) ;
   Dca2 = df12 * ( ca1 - ca2 ) / ( bBu + 1.0 ) ;
   cai = ca1 ;
   ca2i = ca2 ;
   {int _id; for(_id=0; _id < 2; _id++) {
if (_deriv1_advance) {
 _dlist2[++_counte] = _p[_dlist1[_id]] - (_p[_slist1[_id]] - _savstate1[_id])/dt;
 }else{
_dlist2[++_counte] = _p[_slist1[_id]] - _savstate1[_id];}}}
 } }
 return _reset;}
 
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
  cai = _ion_cai;
  ica = _ion_ica;
  cai = _ion_cai;
     _ode_spec1 (_p, _ppvar, _thread, _nt);
  _ion_cai = cai;
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
 static void _ode_synonym(int _cnt, double** _pp, Datum** _ppd) { 
	double* _p; Datum* _ppvar;
 	int _i; 
	for (_i=0; _i < _cnt; ++_i) {_p = _pp[_i]; _ppvar = _ppd[_i];
 _ion_cai =  ca1 ;
 }}
 
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
  cai = _ion_cai;
  ica = _ion_ica;
  cai = _ion_cai;
 _ode_matsol_instance1(_threadargs_);
 }}
 
static void _thread_mem_init(Datum* _thread) {
   _thread[_dith1]._pval = (double*)ecalloc(4, sizeof(double));
   _newtonspace1 = nrn_cons_newtonspace(2);
  if (_thread1data_inuse) {_thread[_gth]._pval = (double*)ecalloc(1, sizeof(double));
 }else{
 _thread[_gth]._pval = _thread1data; _thread1data_inuse = 1;
 }
 }
 
static void _thread_cleanup(Datum* _thread) {
   free((void*)(_thread[_dith1]._pval));
   nrn_destroy_newtonspace(_newtonspace1);
  if (_thread[_gth]._pval == _thread1data) {
   _thread1data_inuse = 0;
  }else{
   free((void*)_thread[_gth]._pval);
  }
 }
 extern void nrn_update_ion_pointer(Symbol*, Datum*, int, int);
 static void _update_ion_pointer(Datum* _ppvar) {
   nrn_update_ion_pointer(_ca_sym, _ppvar, 0, 1);
   nrn_update_ion_pointer(_ca_sym, _ppvar, 1, 3);
 }

static void initmodel(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
  int _i; double _save;{
  ca2 = ca20;
  ca1 = ca10;
 {
   if ( rshell > diam / 4.0 ) {
     rshell = diam / 4.0 ;
     }
   r12 = diam / 2.0 ;
   a12 = 2.0 * ( r12 - rshell ) ;
   v1 = rshell * ( 2.0 * r12 - rshell ) ;
   v2 = ( r12 - rshell ) * ( r12 - rshell ) ;
   df12 = DFree * a12 / ( r12 * v2 ) ;
   df21 = DFree * a12 / ( r12 * v1 ) ;
   cai = cai0 ;
   ca2i = cai ;
   ca1 = cai ;
   ca2 = cai ;
   chflux = 0.0 ;
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
  cai = _ion_cai;
  ica = _ion_ica;
  cai = _ion_cai;
 initmodel(_p, _ppvar, _thread, _nt);
  _ion_cai = cai;
  nrn_wrote_conc(_ca_sym, (&(_ion_cai)) - 1, _style_ca);
}
}

static double _nrn_current(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt, double _v){double _current=0.;v=_v;{
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
double _dtsav = dt;
if (secondorder) { dt *= 0.5; }
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
  cai = _ion_cai;
  ica = _ion_ica;
  cai = _ion_cai;
 {  _deriv1_advance = 1;
 derivimplicit_thread(2, _slist1, _dlist1, _p, state, _ppvar, _thread, _nt);
_deriv1_advance = 0;
     if (secondorder) {
    int _i;
    for (_i = 0; _i < 2; ++_i) {
      _p[_slist1[_i]] += dt*_p[_dlist1[_i]];
    }}
 } {
   }
  _ion_cai = cai;
}}
 dt = _dtsav;
}

static void terminal(){}

static void _initlists(){
 double _x; double* _p = &_x;
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = &(ca1) - _p;  _dlist1[0] = &(Dca1) - _p;
 _slist1[1] = &(ca2) - _p;  _dlist1[1] = &(Dca2) - _p;
 _slist2[0] = &(ca2) - _p;
 _slist2[1] = &(ca1) - _p;
_first = 0;
}

#if defined(__cplusplus)
} /* extern "C" */
#endif

#if NMODL_TEXT
static const char* nmodl_filename = "ca2pool.mod";
static const char* nmodl_file_text = 
  "TITLE Ca concentration in submembrane and core shells.\n"
  "\n"
  "COMMENT\n"
  "Internal calcium accumulation ([Ca]i) in 2 pool model.\n"
  "Pools consist of a thin submembrane shell, and the remaining core\n"
  "of the cell compartment. Submembrane concentration is calculated at\n"
  "the submembrane surface.\n"
  "Instantaneous buffering included.\n"
  "\n"
  "Implemented by BPG 19-9-07\n"
  "ENDCOMMENT\n"
  "\n"
  "NEURON {\n"
  "	SUFFIX ca2p\n"
  "	USEION ca READ cai, ica WRITE cai\n"
  "	RANGE ca2i, cai0, rshell\n"
  "	GLOBAL DFree,bBu\n"
  "}\n"
  "\n"
  "\n"
  "INDEPENDENT {t FROM 0 TO 1 WITH 10 (ms)}\n"
  "\n"
  "UNITS {\n"
  "        (mol)   = (1)\n"
  "        (mmol)   = (millimol)\n"
  "	(molar) = (1/liter)\n"
  "	(mM)	= (millimolar)\n"
  "	(um)	= (micron)\n"
  "	(mA)	= (milliamp)\n"
  "	FARADAY = (faraday)	 (10000 coulomb)\n"
  "	PI	= (pi) (1)\n"
  "}\n"
  "\n"
  "PARAMETER {\n"
  "	ica		(mA/cm2)\n"
  "	cai0=5e-5	(mM)	: cai for setting Jleak\n"
  "	rshell=1	(um)	: thickness of juxtamembrane shell\n"
  "	DFree=0.23	(um2/ms)	: diffusion coeff\n"
  "	r12=10		(um)	: distance between shells 1 and 2\n"
  "	bBu=20		(1)\n"
  "}\n"
  "\n"
  "ASSIGNED {\n"
  "	diam		(um)\n"
  "	cai		(mM)	: free calcium in shell 1 (submembrane)\n"
  "	ca2i		(mM)	: free calcium in shell 2 (core)\n"
  "	a12		(um)	: cross-sectional area (per unit length)\n"
  "	v1		(um2)	: submembrane shell volume (per unit length)\n"
  "	v2		(um2)	: core shell volume (per unit length)\n"
  "	df12		(/ms)	: diffusional flux between shells 1 & 2\n"
  "	df21		(/ms)	: etc\n"
  "	chflux		(mM/ms)\n"
  "}\n"
  "\n"
  "STATE {\n"
  "	ca1		(mM) : free ca conc. in shell 1\n"
  "	ca2		(mM) : free ca conc. in shell 2\n"
  "}\n"
  "\n"
  "INITIAL {\n"
  ": check and alter dimensions as per segment diameter\n"
  "	if (rshell > diam/4) {rshell = diam/4}	: restrict to half radius\n"
  "	r12 = diam/2 	: distance to centre (from membrane)\n"
  "	a12 = 2*(r12-rshell)	: cross-sectional area between shells\n"
  "	v1 = rshell*(2*r12-rshell)	: volume of submembrane shell\n"
  "	v2 = (r12-rshell)*(r12-rshell)	: volume of core shell\n"
  "	df12 = DFree*a12/(r12*v2)\n"
  "	df21 = DFree*a12/(r12*v1)\n"
  "	cai = cai0\n"
  "	ca2i = cai\n"
  "	ca1 = cai\n"
  "	ca2 = cai\n"
  "	chflux = 0\n"
  "}\n"
  "\n"
  "BREAKPOINT {\n"
  "	SOLVE state METHOD derivimplicit\n"
  "}\n"
  "\n"
  "DERIVATIVE state { \n"
  "\n"
  "        chflux = -ica*diam/(2*FARADAY*v1)	: channel influx\n"
  "\n"
  "        ca1' = (chflux + df21*(ca2-ca1))/(bBu+1)\n"
  "	ca2' = df12*(ca1-ca2)/(bBu+1)\n"
  "\n"
  "	cai = ca1	: Ca(i) for Ca channels and Ica is conc. in shell.1\n"
  "	ca2i = ca2\n"
  "}\n"
  "\n"
  "	\n"
  "COMMENT\n"
  "At this time, conductances (and channel states and currents are\n"
  "calculated at the midpoint of a dt interval.  Membrane potential and\n"
  "concentrations are calculated at the edges of a dt interval.  With\n"
  "secondorder=2 everything turns out to be second order correct.\n"
  "ENDCOMMENT\n"
  "\n"
  ;
#endif
