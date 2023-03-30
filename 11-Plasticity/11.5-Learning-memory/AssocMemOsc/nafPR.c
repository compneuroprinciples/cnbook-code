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
 
#define nrn_init _nrn_init__nafPR
#define _nrn_initial _nrn_initial__nafPR
#define nrn_cur _nrn_cur__nafPR
#define _nrn_current _nrn_current__nafPR
#define nrn_jacob _nrn_jacob__nafPR
#define nrn_state _nrn_state__nafPR
#define _net_receive _net_receive__nafPR 
#define mh mh__nafPR 
#define states states__nafPR 
 
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
#define gmax _p[0]
#define i _p[1]
#define g _p[2]
#define h _p[3]
#define ina _p[4]
#define Dh _p[5]
#define _g _p[6]
#define _ion_ina	*_ppvar[0]._pval
#define _ion_dinadv	*_ppvar[1]._pval
 
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
 static void _hoc_alpha(void);
 static void _hoc_beta(void);
 static void _hoc_mh(void);
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
 "setdata_nafPR", _hoc_setdata,
 "alpha_nafPR", _hoc_alpha,
 "beta_nafPR", _hoc_beta,
 "mh_nafPR", _hoc_mh,
 0, 0
};
#define alpha alpha_nafPR
#define beta beta_nafPR
 extern double alpha( double , double );
 extern double beta( double , double );
 /* declare global and static user variables */
#define Inf Inf_nafPR
 double Inf[2];
#define Tau Tau_nafPR
 double Tau[2];
#define exptemp exptemp_nafPR
 double exptemp = 37;
#define erev erev_nafPR
 double erev = 60;
#define hexp hexp_nafPR
 double hexp = 1;
#define hq10 hq10_nafPR
 double hq10 = 3;
#define hbetaV0 hbetaV0_nafPR
 double hbetaV0 = 40;
#define hbetaB hbetaB_nafPR
 double hbetaB = -5;
#define hbetaA hbetaA_nafPR
 double hbetaA = 4;
#define hbflag hbflag_nafPR
 double hbflag = 2;
#define halphaV0 halphaV0_nafPR
 double halphaV0 = 17;
#define halphaB halphaB_nafPR
 double halphaB = -18;
#define halphaA halphaA_nafPR
 double halphaA = 0.128;
#define haflag haflag_nafPR
 double haflag = 1;
#define mexp mexp_nafPR
 double mexp = -2;
#define mq10 mq10_nafPR
 double mq10 = 3;
#define mbetaV0 mbetaV0_nafPR
 double mbetaV0 = 40.1;
#define mbetaB mbetaB_nafPR
 double mbetaB = 5;
#define mbetaA mbetaA_nafPR
 double mbetaA = 0.28;
#define mbflag mbflag_nafPR
 double mbflag = 3;
#define malphaV0 malphaV0_nafPR
 double malphaV0 = 13.1;
#define malphaB malphaB_nafPR
 double malphaB = -4;
#define malphaA malphaA_nafPR
 double malphaA = -0.32;
#define maflag maflag_nafPR
 double maflag = 3;
#define qq10 qq10_nafPR
 double qq10[2];
#define vrest vrest_nafPR
 double vrest = -60;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "erev_nafPR", "mV",
 "gmax_nafPR", "mho/cm2",
 "i_nafPR", "mA/cm^2",
 "g_nafPR", "mho/cm^2",
 0,0
};
 static double delta_t = 1;
 static double h0 = 0;
 static double v = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "erev_nafPR", &erev_nafPR,
 "vrest_nafPR", &vrest_nafPR,
 "exptemp_nafPR", &exptemp_nafPR,
 "maflag_nafPR", &maflag_nafPR,
 "malphaA_nafPR", &malphaA_nafPR,
 "malphaB_nafPR", &malphaB_nafPR,
 "malphaV0_nafPR", &malphaV0_nafPR,
 "mbflag_nafPR", &mbflag_nafPR,
 "mbetaA_nafPR", &mbetaA_nafPR,
 "mbetaB_nafPR", &mbetaB_nafPR,
 "mbetaV0_nafPR", &mbetaV0_nafPR,
 "mq10_nafPR", &mq10_nafPR,
 "mexp_nafPR", &mexp_nafPR,
 "haflag_nafPR", &haflag_nafPR,
 "halphaA_nafPR", &halphaA_nafPR,
 "halphaB_nafPR", &halphaB_nafPR,
 "halphaV0_nafPR", &halphaV0_nafPR,
 "hbflag_nafPR", &hbflag_nafPR,
 "hbetaA_nafPR", &hbetaA_nafPR,
 "hbetaB_nafPR", &hbetaB_nafPR,
 "hbetaV0_nafPR", &hbetaV0_nafPR,
 "hq10_nafPR", &hq10_nafPR,
 "hexp_nafPR", &hexp_nafPR,
 0,0
};
 static DoubVec hoc_vdoub[] = {
 "Inf_nafPR", Inf_nafPR, 2,
 "Tau_nafPR", Tau_nafPR, 2,
 "qq10_nafPR", qq10_nafPR, 2,
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
"nafPR",
 "gmax_nafPR",
 0,
 "i_nafPR",
 "g_nafPR",
 0,
 "h_nafPR",
 0,
 0};
 static Symbol* _na_sym;
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
 	_p = nrn_prop_data_alloc(_mechtype, 7, _prop);
 	/*initialize range parameters*/
 	gmax = 0.03;
 	_prop->param = _p;
 	_prop->param_size = 7;
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 3, _prop);
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_na_sym);
 	_ppvar[0]._pval = &prop_ion->param[3]; /* ina */
 	_ppvar[1]._pval = &prop_ion->param[4]; /* _ion_dinadv */
 
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

 void _nafPR_reg() {
	int _vectorized = 0;
  _initlists();
 	ion_reg("na", -10000.);
 	_na_sym = hoc_lookup("na_ion");
 	register_mech(_mechanism, nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init, hoc_nrnpointerindex, 0);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
     _nrn_thread_reg(_mechtype, 2, _update_ion_pointer);
 #if NMODL_TEXT
  hoc_reg_nmodl_text(_mechtype, nmodl_file_text);
  hoc_reg_nmodl_filename(_mechtype, nmodl_filename);
#endif
  hoc_register_prop_size(_mechtype, 7, 3);
  hoc_register_dparam_semantics(_mechtype, 0, "na_ion");
  hoc_register_dparam_semantics(_mechtype, 1, "na_ion");
  hoc_register_dparam_semantics(_mechtype, 2, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 nafPR C:/Users/bpg/Desktop/Projects/CN Schools/BNNI/2021/Material/Code/Running/AssocMem/nafPR.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
 static double FARADAY = 96489.0;
 static double R = 8.31441;
static int _reset;
static char *modelname = "";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int mh(double);
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static int _slist1[1], _dlist1[1];
 static int states(_threadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 () {_reset=0;
 {
   mh ( _threadargscomma_ v ) ;
   Dh = ( - h + Inf [ 1 ] ) / Tau [ 1 ] ;
   }
 return _reset;
}
 static int _ode_matsol1 () {
 mh ( _threadargscomma_ v ) ;
 Dh = Dh  / (1. - dt*( ( ( - 1.0 ) ) / Tau[1] )) ;
  return 0;
}
 /*END CVODE*/
 static int states () {_reset=0;
 {
   mh ( _threadargscomma_ v ) ;
    h = h + (1. - exp(dt*(( ( - 1.0 ) ) / Tau[1])))*(- ( ( ( Inf[1] ) ) / Tau[1] ) / ( ( ( - 1.0 ) ) / Tau[1] ) - h) ;
   }
  return 0;
}
 
static int  mh (  double _lv ) {
   double _la , _lb , _lj ;
 qq10 [ 0 ] = pow( mq10 , ( ( celsius - exptemp ) / 10. ) ) ;
   qq10 [ 1 ] = pow( hq10 , ( ( celsius - exptemp ) / 10. ) ) ;
   {int  _lj ;for ( _lj = 0 ; _lj <= 1 ; _lj ++ ) {
     _la = alpha ( _threadargscomma_ _lv , ((double) _lj ) ) ;
     _lb = beta ( _threadargscomma_ _lv , ((double) _lj ) ) ;
     Inf [ _lj ] = _la / ( _la + _lb ) ;
     Tau [ _lj ] = 1. / ( _la + _lb ) / qq10 [ _lj ] ;
     if ( hexp  == 0.0 ) {
       Tau [ 1 ] = 1. ;
       Inf [ 1 ] = 1. ;
       }
     } }
    return 0; }
 
static void _hoc_mh(void) {
  double _r;
   _r = 1.;
 mh (  *getarg(1) );
 hoc_retpushx(_r);
}
 
double alpha (  double _lv , double _lj ) {
   double _lalpha;
 double _lflag , _lA , _lB , _lV0 ;
 if ( _lj  == 1.0  && hexp  == 0.0 ) {
     _lalpha = 0.0 ;
     }
   else {
     if ( _lj  == 1.0 ) {
       _lA = halphaA ;
       _lB = halphaB ;
       _lV0 = halphaV0 + vrest ;
       _lflag = haflag ;
       }
     else {
       _lA = malphaA ;
       _lB = malphaB ;
       _lV0 = malphaV0 + vrest ;
       _lflag = maflag ;
       }
     if ( _lflag  == 1.0 ) {
       _lalpha = _lA * exp ( ( _lv - _lV0 ) / _lB ) ;
       }
     else if ( _lflag  == 2.0 ) {
       _lalpha = _lA / ( exp ( ( _lv - _lV0 ) / _lB ) + 1.0 ) ;
       }
     else if ( _lflag  == 3.0 ) {
       if ( _lv  == _lV0 ) {
         _lalpha = _lA * _lB ;
         }
       else {
         _lalpha = _lA * ( _lv - _lV0 ) / ( exp ( ( _lv - _lV0 ) / _lB ) - 1.0 ) ;
         }
       }
     }
   
return _lalpha;
 }
 
static void _hoc_alpha(void) {
  double _r;
   _r =  alpha (  *getarg(1) , *getarg(2) );
 hoc_retpushx(_r);
}
 
double beta (  double _lv , double _lj ) {
   double _lbeta;
 double _lflag , _lA , _lB , _lV0 ;
 if ( _lj  == 1.0  && hexp  == 0.0 ) {
     _lbeta = 1.0 ;
     }
   else {
     if ( _lj  == 1.0 ) {
       _lA = hbetaA ;
       _lB = hbetaB ;
       _lV0 = hbetaV0 + vrest ;
       _lflag = hbflag ;
       }
     else {
       _lA = mbetaA ;
       _lB = mbetaB ;
       _lV0 = mbetaV0 + vrest ;
       _lflag = mbflag ;
       }
     if ( _lflag  == 1.0 ) {
       _lbeta = _lA * exp ( ( _lv - _lV0 ) / _lB ) ;
       }
     else if ( _lflag  == 2.0 ) {
       _lbeta = _lA / ( exp ( ( _lv - _lV0 ) / _lB ) + 1.0 ) ;
       }
     else if ( _lflag  == 3.0 ) {
       if ( _lv  == _lV0 ) {
         _lbeta = _lA * _lB ;
         }
       else {
         _lbeta = _lA * ( _lv - _lV0 ) / ( exp ( ( _lv - _lV0 ) / _lB ) - 1.0 ) ;
         }
       }
     }
   
return _lbeta;
 }
 
static void _hoc_beta(void) {
  double _r;
   _r =  beta (  *getarg(1) , *getarg(2) );
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
 _ode_matsol_instance1(_threadargs_);
 }}
 extern void nrn_update_ion_pointer(Symbol*, Datum*, int, int);
 static void _update_ion_pointer(Datum* _ppvar) {
   nrn_update_ion_pointer(_na_sym, _ppvar, 0, 3);
   nrn_update_ion_pointer(_na_sym, _ppvar, 1, 4);
 }

static void initmodel() {
  int _i; double _save;_ninits++;
 _save = t;
 t = 0.0;
{
  h = h0;
 {
   mh ( _threadargscomma_ v ) ;
   h = Inf [ 1 ] ;
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
 initmodel();
 }}

static double _nrn_current(double _v){double _current=0.;v=_v;{ {
   mh ( _threadargscomma_ v ) ;
   g = gmax * Inf [ 0 ] * Inf [ 0 ] * h ;
   i = g * ( v - erev ) ;
   ina = i ;
   }
 _current += ina;

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
 _g = _nrn_current(_v + .001);
 	{ double _dina;
  _dina = ina;
 _rhs = _nrn_current(_v);
  _ion_dinadv += (_dina - ina)/.001 ;
 	}
 _g = (_g - _rhs)/.001;
  _ion_ina += ina ;
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
 { error =  states();
 if(error){fprintf(stderr,"at line 77 in file nafPR.mod:\n  SOLVE states METHOD cnexp\n"); nrn_complain(_p); abort_run(error);}
 } }}

}

static void terminal(){}

static void _initlists() {
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = &(h) - _p;  _dlist1[0] = &(Dh) - _p;
_first = 0;
}

#if NMODL_TEXT
static const char* nmodl_filename = "nafPR.mod";
static const char* nmodl_file_text = 
  ": $Id: nafPR.mod,v 1.1 2009/01/19 16:39:51 sterratt Exp $\n"
  "\n"
  "INDEPENDENT {t FROM 0 TO 1 WITH 1 (ms)}\n"
  "\n"
  "NEURON { SUFFIX nafPR }\n"
  "NEURON {  USEION na WRITE ina }\n"
  "ASSIGNED { ina }\n"
  "\n"
  "PARAMETER {\n"
  "	erev 		= 60.  (mV)\n"
  "	gmax 		= 0.030    (mho/cm2)\n"
  "\n"
  "        vrest           = -60.0\n"
  "	exptemp		= 37\n"
  "	maflag 		= 3\n"
  "	malphaA 	= -0.32\n"
  "	malphaB		= -4.0\n"
  "	malphaV0	= 13.1\n"
  "	mbflag 		= 3\n"
  "	mbetaA 		= 0.28\n"
  "	mbetaB		= 5.0\n"
  "	mbetaV0		= 40.1\n"
  "	mq10		= 3\n"
  "	mexp 		= -2\n"
  "\n"
  "	haflag 		= 1\n"
  "	halphaA 	= 0.128\n"
  "	halphaB		= -18\n"
  "	halphaV0	= 17.\n"
  "	hbflag 		= 2\n"
  "	hbetaA 		= 4.\n"
  "	hbetaB		= -5.\n"
  "	hbetaV0		= 40.\n"
  "	hq10		= 3\n"
  "	hexp 		= 1\n"
  "\n"
  "	celsius			   (degC)\n"
  "	dt 				   (ms)\n"
  "	v 			       (mV)\n"
  "\n"
  "} : end PARAMETER\n"
  "\n"
  "NEURON {\n"
  "	RANGE gmax, g, i\n"
  "	GLOBAL erev, Inf, Tau, vrest, qq10\n"
  "} : end NEURON\n"
  "\n"
  "CONSTANT {\n"
  "	  FARADAY = 96489.0	: Faraday's constant\n"
  "	  R= 8.31441		: Gas constant\n"
  "\n"
  "} : end CONSTANT\n"
  "\n"
  "UNITS {\n"
  "	(mA) = (milliamp)\n"
  "	(mV) = (millivolt)\n"
  "	(umho) = (micromho)\n"
  "} : end UNITS\n"
  "\n"
  "ASSIGNED {\n"
  "	i (mA/cm^2)		\n"
  "	g (mho/cm^2)\n"
  "	Inf[2]		: 0 = m and 1 = h\n"
  "	Tau[2]		: 0 = m and 1 = h\n"
  "        qq10[2]\n"
  "} : end ASSIGNED \n"
  "\n"
  "STATE { h }\n"
  "\n"
  "INITIAL { \n"
  " 	mh(v)\n"
  "	h = Inf[1]\n"
  "}\n"
  "\n"
  "BREAKPOINT {\n"
  "\n"
  "  SOLVE states METHOD cnexp\n"
  "  mh(v)\n"
  "  g = gmax * Inf[0]*Inf[0] * h\n"
  "\n"
  "  i = g*(v-erev) \n"
  "  ina=i\n"
  "} : end BREAKPOINT\n"
  "\n"
  ": ASSIGNMENT PROCEDURES\n"
  ": Must be given by a user routines in parameters.multi\n"
  ": E.G.:\n"
  ":   PROCEDURE iassign () { i = g*(v-erev) ina=i }\n"
  ":   PROCEDURE iassign () { i = g*ghkca(v) ica=i }\n"
  "\n"
  ":-------------------------------------------------------------------\n"
  "\n"
  "DERIVATIVE states {\n"
  "	mh(v)\n"
  "	h' = (-h + Inf[1]) / Tau[1]\n"
  " }\n"
  "\n"
  ":-------------------------------------------------------------------\n"
  ": NOTE : 0 = m and 1 = h\n"
  "PROCEDURE mh (v) {\n"
  "	LOCAL a, b, j\n"
  "\n"
  "	qq10[0] = mq10^((celsius-exptemp)/10.)	\n"
  "	qq10[1] = hq10^((celsius-exptemp)/10.)	\n"
  "\n"
  "	: Calculater Inf and Tau values for h and m\n"
  "	FROM j = 0 TO 1 {\n"
  "		a = alpha (v, j)\n"
  "		b = beta (v, j)\n"
  "\n"
  "		Inf[j] = a / (a + b)\n"
  "		Tau[j] = 1. / (a + b) / qq10[j]\n"
  "		if (hexp==0) { Tau[1] = 1. Inf[1] = 1.}\n"
  "	}\n"
  "} : end PROCEDURE mh (v)\n"
  "\n"
  ":-------------------------------------------------------------------\n"
  "FUNCTION alpha(v,j) {\n"
  "  LOCAL flag, A, B, V0\n"
  "  if (j==1 && hexp==0) {\n"
  "	  alpha = 0\n"
  "  } else {\n"
  "\n"
  "     if (j == 1) {\n"
  "	  A = halphaA B = halphaB V0 = halphaV0+vrest flag = haflag\n"
  "     } else {\n"
  "	  A = malphaA B = malphaB V0 = malphaV0+vrest flag = maflag\n"
  "     }\n"
  "\n"
  "     if (flag == 1) { :  EXPONENTIAL\n"
  "	 alpha = A*exp((v-V0)/B)	\n"
  "     } else if (flag == 2) { :  SIGMOID\n"
  "	 alpha = A/(exp((v-V0)/B)+1)\n"
  "     } else if (flag == 3) { :  LINOID\n"
  "	 if(v == V0) {\n"
  "           alpha = A*B\n"
  "         } else {\n"
  "           alpha = A*(v-V0)/(exp((v-V0)/B)-1) }\n"
  "     }\n"
  "}\n"
  "} : end FUNCTION alpha (v,j)\n"
  "\n"
  ":-------------------------------------------------------------------\n"
  "FUNCTION beta (v,j) {\n"
  "  LOCAL flag, A, B, V0\n"
  "  if (j==1 && hexp==0) {\n"
  "	  beta = 1\n"
  "  } else {\n"
  "\n"
  "     if (j == 1) {\n"
  "	  A = hbetaA B = hbetaB V0 = hbetaV0+vrest flag = hbflag\n"
  "     } else {\n"
  "	  A = mbetaA B = mbetaB V0 = mbetaV0+vrest flag = mbflag\n"
  "     }\n"
  "\n"
  "    if (flag == 1) { :  EXPONENTIAL\n"
  "	 beta = A*exp((v-V0)/B)\n"
  "     } else if (flag == 2) { :  SIGMOID\n"
  "	 beta = A/(exp((v-V0)/B)+1)\n"
  "     } else if (flag == 3) { :  LINOID\n"
  "	 if(v == V0) {\n"
  "            beta = A*B \n"
  "         } else {\n"
  "            beta = A*(v-V0)/(exp((v-V0)/B)-1) }\n"
  "     }\n"
  "}\n"
  "} : end FUNCTION beta (v,j)\n"
  ;
#endif
