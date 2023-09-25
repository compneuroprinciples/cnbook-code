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
 
#define nrn_init _nrn_init__ca3c
#define _nrn_initial _nrn_initial__ca3c
#define nrn_cur _nrn_cur__ca3c
#define _nrn_current _nrn_current__ca3c
#define nrn_jacob _nrn_jacob__ca3c
#define nrn_state _nrn_state__ca3c
#define _net_receive _net_receive__ca3c 
#define state state__ca3c 
 
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
#define ca2i _p[0]
#define df12 _p[1]
#define df21 _p[2]
#define df23 _p[3]
#define df32 _p[4]
#define chflux _p[5]
#define pflux _p[6]
#define Jleak _p[7]
#define ca1 _p[8]
#define ca2 _p[9]
#define ca3 _p[10]
#define ica _p[11]
#define cai _p[12]
#define a23 _p[13]
#define v2 _p[14]
#define v3 _p[15]
#define Dca1 _p[16]
#define Dca2 _p[17]
#define Dca3 _p[18]
#define _g _p[19]
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
 "setdata_ca3c", _hoc_setdata,
 0, 0
};
 /* declare global and static user variables */
#define DFree DFree_ca3c
 double DFree = 0.6;
#define Vmax Vmax_ca3c
 double Vmax = 9e-08;
#define alpha12 alpha12_ca3c
 double alpha12 = 1;
#define alpha1 alpha1_ca3c
 double alpha1 = 0.0001;
#define bBu bBu_ca3c
 double bBu = 20;
#define kd kd_ca3c
 double kd = 0.001;
#define refcai refcai_ca3c
 double refcai = 5e-05;
#define r23 r23_ca3c
 double r23 = 1;
#define r12 r12_ca3c
 double r12 = 1;
#define rshell rshell_ca3c
 double rshell = 1;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "refcai_ca3c", "mM",
 "rshell_ca3c", "um",
 "alpha1_ca3c", "1",
 "alpha12_ca3c", "/um",
 "DFree_ca3c", "um2/ms",
 "r12_ca3c", "um",
 "r23_ca3c", "um",
 "Vmax_ca3c", "mmol/ms-cm2",
 "kd_ca3c", "mM",
 "bBu_ca3c", "1",
 "ca1_ca3c", "mM",
 "ca2_ca3c", "mM",
 "ca3_ca3c", "mM",
 "ca2i_ca3c", "mM",
 "df12_ca3c", "/ms",
 "df21_ca3c", "/ms",
 "df23_ca3c", "/ms",
 "df32_ca3c", "/ms",
 "chflux_ca3c", "mM/ms",
 "pflux_ca3c", "mmol/ms-cm2",
 "Jleak_ca3c", "mmol/ms-cm2",
 0,0
};
 static double ca30 = 0;
 static double ca20 = 0;
 static double ca10 = 0;
 static double delta_t = 0.1;
 static double v = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "refcai_ca3c", &refcai_ca3c,
 "rshell_ca3c", &rshell_ca3c,
 "alpha1_ca3c", &alpha1_ca3c,
 "alpha12_ca3c", &alpha12_ca3c,
 "DFree_ca3c", &DFree_ca3c,
 "r12_ca3c", &r12_ca3c,
 "r23_ca3c", &r23_ca3c,
 "Vmax_ca3c", &Vmax_ca3c,
 "kd_ca3c", &kd_ca3c,
 "bBu_ca3c", &bBu_ca3c,
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
"ca3c",
 0,
 "ca2i_ca3c",
 "df12_ca3c",
 "df21_ca3c",
 "df23_ca3c",
 "df32_ca3c",
 "chflux_ca3c",
 "pflux_ca3c",
 "Jleak_ca3c",
 0,
 "ca1_ca3c",
 "ca2_ca3c",
 "ca3_ca3c",
 0,
 0};
 static Symbol* _morphology_sym;
 static Symbol* _ca_sym;
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
 	_p = nrn_prop_data_alloc(_mechtype, 20, _prop);
 	/*initialize range parameters*/
 	_prop->param = _p;
 	_prop->param_size = 20;
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
 static void _update_ion_pointer(Datum*);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, _NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _ca3pool_reg() {
	int _vectorized = 0;
  _initlists();
 	ion_reg("ca", -10000.);
 	_morphology_sym = hoc_lookup("morphology");
 	_ca_sym = hoc_lookup("ca_ion");
 	register_mech(_mechanism, nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init, hoc_nrnpointerindex, 0);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
     _nrn_thread_reg(_mechtype, 2, _update_ion_pointer);
 #if NMODL_TEXT
  hoc_reg_nmodl_text(_mechtype, nmodl_file_text);
  hoc_reg_nmodl_filename(_mechtype, nmodl_filename);
#endif
  hoc_register_prop_size(_mechtype, 20, 5);
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
 	ivoc_help("help ?1 ca3c C:/Users/bpg/Desktop/Projects/CN book/Edition2/Exercises/Code-Edn1/Chapt6/Fig6.7/ca3pool.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
 
#define FARADAY _nrnunit_FARADAY[_nrnunit_use_legacy_]
static double _nrnunit_FARADAY[2] = {0x9.a60645c954db8p+0, 9.64853}; /* 9.64853321233100125 */
 
#define PI _nrnunit_PI[_nrnunit_use_legacy_]
static double _nrnunit_PI[2] = {0xc.90fdaa22168cp-2, 3.14159}; /* 3.14159265358979312 */
static int _reset;
static char *modelname = "Three compartment model of calcium concentration";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
 static int _deriv1_advance = 0;
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static int _slist2[3]; static double _dlist2[3];
 static double _savstate1[3], *_temp1 = _savstate1;
 static int _slist1[3], _dlist1[3];
 static int state(_threadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 () {_reset=0;
 {
   chflux = - ica / ( 2.0 * FARADAY * alpha1 * rshell ) ;
   pflux = - ( Vmax * ca2 / ( ca2 + kd ) ) + Jleak ;
   Dca1 = ( chflux + ( df21 * ( ca2 - ca1 ) ) ) / ( bBu + 1.0 ) ;
   Dca2 = ( ( ( 1e7 ) * pflux / rshell ) + df12 * ( ca1 - ca2 ) + df32 * ( ca3 - ca2 ) ) / ( bBu + 1.0 ) ;
   Dca3 = df23 * ( ca2 - ca3 ) / ( bBu + 1.0 ) ;
   cai = ca1 ;
   ca2i = ca2 ;
   }
 return _reset;
}
 static int _ode_matsol1 () {
 chflux = - ica / ( 2.0 * FARADAY * alpha1 * rshell ) ;
 pflux = - ( Vmax * ca2 / ( ca2 + kd ) ) + Jleak ;
 Dca1 = Dca1  / (1. - dt*( ( ( ( ( df21 )*( ( ( - 1.0 ) ) ) ) ) ) / ( bBu + 1.0 ) )) ;
 Dca2 = Dca2  / (1. - dt*( ( ( ( df12 )*( ( ( - 1.0 ) ) ) + ( df32 )*( ( ( - 1.0 ) ) ) ) ) / ( bBu + 1.0 ) )) ;
 Dca3 = Dca3  / (1. - dt*( ( ( df23 )*( ( ( - 1.0 ) ) ) ) / ( bBu + 1.0 ) )) ;
 cai = ca1 ;
 ca2i = ca2 ;
  return 0;
}
 /*END CVODE*/
 
static int state () {_reset=0;
 { static int _recurse = 0;
 int _counte = -1;
 if (!_recurse) {
 _recurse = 1;
 {int _id; for(_id=0; _id < 3; _id++) { _savstate1[_id] = _p[_slist1[_id]];}}
 error = newton(3,_slist2, _p, state, _dlist2);
 _recurse = 0; if(error) {abort_run(error);}}
 {
   chflux = - ica / ( 2.0 * FARADAY * alpha1 * rshell ) ;
   pflux = - ( Vmax * ca2 / ( ca2 + kd ) ) + Jleak ;
   Dca1 = ( chflux + ( df21 * ( ca2 - ca1 ) ) ) / ( bBu + 1.0 ) ;
   Dca2 = ( ( ( 1e7 ) * pflux / rshell ) + df12 * ( ca1 - ca2 ) + df32 * ( ca3 - ca2 ) ) / ( bBu + 1.0 ) ;
   Dca3 = df23 * ( ca2 - ca3 ) / ( bBu + 1.0 ) ;
   cai = ca1 ;
   ca2i = ca2 ;
   {int _id; for(_id=0; _id < 3; _id++) {
if (_deriv1_advance) {
 _dlist2[++_counte] = _p[_dlist1[_id]] - (_p[_slist1[_id]] - _savstate1[_id])/dt;
 }else{
_dlist2[++_counte] = _p[_slist1[_id]] - _savstate1[_id];}}}
 } }
 return _reset;}
 
static int _ode_count(int _type){ return 3;}
 
static void _ode_spec(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   Datum* _thread;
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
     _ode_spec1 ();
  _ion_cai = cai;
 }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 3; ++_i) {
		_pv[_i] = _pp + _slist1[_i];  _pvdot[_i] = _pp + _dlist1[_i];
		_cvode_abstol(_atollist, _atol, _i);
	}
 }
 static void _ode_synonym(int _cnt, double** _pp, Datum** _ppd) { 
 	int _i; 
	for (_i=0; _i < _cnt; ++_i) {_p = _pp[_i]; _ppvar = _ppd[_i];
 _ion_cai =  ca1 ;
 }}
 
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
  cai = _ion_cai;
  ica = _ion_ica;
  cai = _ion_cai;
 _ode_matsol_instance1(_threadargs_);
 }}
 extern void nrn_update_ion_pointer(Symbol*, Datum*, int, int);
 static void _update_ion_pointer(Datum* _ppvar) {
   nrn_update_ion_pointer(_ca_sym, _ppvar, 0, 1);
   nrn_update_ion_pointer(_ca_sym, _ppvar, 1, 3);
 }

static void initmodel() {
  int _i; double _save;_ninits++;
 _save = t;
 t = 0.0;
{
  ca3 = ca30;
  ca2 = ca20;
  ca1 = ca10;
 {
   if ( rshell > diam / 4.0 ) {
     rshell = diam / 4.0 ;
     }
   r12 = PI * diam / 2.0 ;
   r23 = diam / 2.0 ;
   a23 = 2.0 * ( r23 - rshell ) ;
   v2 = rshell * ( 2.0 * r23 - rshell ) ;
   v3 = ( r23 - rshell ) * ( r23 - rshell ) ;
   df12 = DFree * alpha12 / ( ( 1.0 - alpha1 ) * r12 ) ;
   df21 = DFree * alpha12 / ( alpha1 * r12 ) ;
   df23 = DFree * a23 * ( 1.0 - alpha1 ) / ( r23 * v3 ) ;
   df32 = DFree * a23 / ( r23 * v2 ) ;
   
/*VERBATIM*/
	cai = _ion_cai;
 ca2i = cai ;
   ca1 = cai ;
   ca2 = cai ;
   ca3 = cai ;
   chflux = 0.0 ;
   pflux = 0.0 ;
   Jleak = Vmax * refcai / ( refcai + kd ) ;
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
  cai = _ion_cai;
  ica = _ion_ica;
  cai = _ion_cai;
 initmodel();
  _ion_cai = cai;
  nrn_wrote_conc(_ca_sym, (&(_ion_cai)) - 1, _style_ca);
}}

static double _nrn_current(double _v){double _current=0.;v=_v;{
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
double _dtsav = dt;
if (secondorder) { dt *= 0.5; }
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
  cai = _ion_cai;
  ica = _ion_ica;
  cai = _ion_cai;
 { error = _deriv1_advance = 1;
 derivimplicit(_ninits, 3, _slist1, _dlist1, _p, &t, dt, state, &_temp1);
_deriv1_advance = 0;
 if(error){fprintf(stderr,"at line 98 in file ca3pool.mod:\n	SOLVE state METHOD derivimplicit\n"); nrn_complain(_p); abort_run(error);}
    if (secondorder) {
    int _i;
    for (_i = 0; _i < 3; ++_i) {
      _p[_slist1[_i]] += dt*_p[_dlist1[_i]];
    }}
 } {
   }
  _ion_cai = cai;
}}
 dt = _dtsav;
}

static void terminal(){}

static void _initlists() {
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = &(ca1) - _p;  _dlist1[0] = &(Dca1) - _p;
 _slist1[1] = &(ca2) - _p;  _dlist1[1] = &(Dca2) - _p;
 _slist1[2] = &(ca3) - _p;  _dlist1[2] = &(Dca3) - _p;
 _slist2[0] = &(ca3) - _p;
 _slist2[1] = &(ca2) - _p;
 _slist2[2] = &(ca1) - _p;
_first = 0;
}

#if NMODL_TEXT
static const char* nmodl_filename = "ca3pool.mod";
static const char* nmodl_file_text = 
  "TITLE Three compartment model of calcium concentration\n"
  "\n"
  "COMMENT\n"
  "Internal calcium accumulation ([Ca]i) with 3 compartment model.\n"
  "Consists of a submembrane shell and a core shell, with the submembrane\n"
  "shell being subdivided into two compartments, corresponding to membrane\n"
  "surrounding calcium channels, and the remaining membrane, respectively.\n"
  "\n"
  "Adapted from Lyle J. Borg-Graham, Interpretations of data and mechanisms\n"
  "for hippocampal pyramidal cell models.  In \"Cerebral Cortex, Vol 13:\n"
  "Cortical Models\", Plenum Press 1998\n"
  "\n"
  "Implemented by BPG 14-6-06\n"
  "ENDCOMMENT\n"
  "\n"
  "NEURON {\n"
  "	SUFFIX ca3c\n"
  "	USEION ca READ cai, ica WRITE cai\n"
  "	RANGE ca2i,chflux,pflux,Jleak,df12,df21,df23,df32\n"
  "	GLOBAL DFree,rshell,alpha1,alpha12,Vmax,kd,bBu,r12,r23\n"
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
  "	refcai=5e-5	(mM)	: cai for setting Jleak\n"
  "	rshell=1	(um)	: thickness of juxtamembrane shell\n"
  "	alpha1=1e-4	(1)	: colocation fraction\n"
  "	alpha12=1	(/um)	: interdigitation coefficient\n"
  "	DFree=0.6	(um2/ms)	: diffusion coeff\n"
  "	r12=1		(um)	: distance between comps. 1 and 2\n"
  "	r23=1		(um)	: distance between comps. 2 and 3\n"
  "	Vmax=9e-8	(mmol/ms-cm2)	: De Schutter\n"
  "	kd=0.001	(mM)		: De Schutter\n"
  "	bBu=20		(1)\n"
  "}\n"
  "\n"
  "ASSIGNED {\n"
  "	diam		(um)\n"
  "	cai		(mM)	: free calcium in compartment 1\n"
  "	ca2i		(mM)	: free calcium in compartment 2\n"
  "	a23		(um)	: cross-sectional area (per unit length)\n"
  "	v2		(um2)	: submembrane shell volume (per unit length)\n"
  "	v3		(um2)	: core shell volume (per unit length)\n"
  "	df12		(/ms)	: diffusional flux between comps.1 & 2\n"
  "	df21		(/ms)	: etc\n"
  "	df23		(/ms)\n"
  "	df32		(/ms)\n"
  "	chflux		(mM/ms)\n"
  "	pflux		(mmol/ms-cm2)\n"
  "	Jleak		(mmol/ms-cm2)\n"
  "}\n"
  "\n"
  "STATE {\n"
  "	ca1		(mM) : free ca conc. in compartment 1\n"
  "	ca2		(mM) : free ca conc. in compartment 2\n"
  "	ca3		(mM) : free ca conc. in compartment 3\n"
  "}\n"
  "\n"
  "INITIAL {\n"
  ": check and alter dimensions as per segment diameter\n"
  "	if (rshell > diam/4) {rshell = diam/4}	: restrict to half radius\n"
  "	r12 = PI*diam/2 	: max half circumference\n"
  "	r23 = diam/2 		: membrane to centre (radius)\n"
  "	a23 = 2*(r23-rshell)	: cross-sectional area between shells\n"
  "	v2 = rshell*(2*r23-rshell)	: volume of submembrane shell\n"
  "	v3 = (r23-rshell)*(r23-rshell)	: volume of core shell\n"
  "	df12 = DFree*alpha12/((1-alpha1)*r12)\n"
  "	df21 = DFree*alpha12/(alpha1*r12)\n"
  "	df23 = DFree*a23*(1-alpha1)/(r23*v3)\n"
  "	df32 = DFree*a23/(r23*v2)\n"
  "VERBATIM\n"
  "	cai = _ion_cai;\n"
  "ENDVERBATIM\n"
  "	ca2i = cai\n"
  "	ca1 = cai\n"
  "	ca2 = cai\n"
  "	ca3 = cai\n"
  "	chflux = 0\n"
  "	pflux = 0\n"
  "	Jleak = Vmax*refcai/(refcai+kd)\n"
  "}\n"
  "\n"
  "BREAKPOINT {\n"
  "	SOLVE state METHOD derivimplicit\n"
  "}\n"
  "\n"
  "DERIVATIVE state { \n"
  "\n"
  "        chflux = -ica/(2*FARADAY*alpha1*rshell)	: channel influx\n"
  "\n"
  "        pflux = -(Vmax*ca2/(ca2+kd))+Jleak  : pump eflux\n"
  "\n"
  "        ca1' = (chflux + (df21*(ca2-ca1)))/(bBu+1)\n"
  "        ca2' = (((1e7)*pflux/rshell) + df12*(ca1-ca2) + df32*(ca3-ca2))/(bBu+1)\n"
  "	ca3' = df23*(ca2-ca3)/(bBu+1)\n"
  "\n"
  "	cai = ca1	: Ca(i) for Ca channels and Ict is conc. in comp.1\n"
  "	ca2i = ca2	: Ca(i) for other channels is conc. in comp.2\n"
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
