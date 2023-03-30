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
 
#define nrn_init _nrn_init__cal
#define _nrn_initial _nrn_initial__cal
#define nrn_cur _nrn_cur__cal
#define _nrn_current _nrn_current__cal
#define nrn_jacob _nrn_jacob__cal
#define nrn_state _nrn_state__cal
#define _net_receive _net_receive__cal 
#define iassign iassign__cal 
#define mh mh__cal 
#define states states__cal 
 
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
#define m _p[3]
#define h _p[4]
#define ica _p[5]
#define Dm _p[6]
#define Dh _p[7]
#define _g _p[8]
#define _ion_ica	*_ppvar[0]._pval
#define _ion_dicadv	*_ppvar[1]._pval
 
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
 static void _hoc_FRT(void);
 static void _hoc_alpha(void);
 static void _hoc_beta(void);
 static void _hoc_ghkca(void);
 static void _hoc_iassign(void);
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
 "setdata_cal", _hoc_setdata,
 "FRT_cal", _hoc_FRT,
 "alpha_cal", _hoc_alpha,
 "beta_cal", _hoc_beta,
 "ghkca_cal", _hoc_ghkca,
 "iassign_cal", _hoc_iassign,
 "mh_cal", _hoc_mh,
 0, 0
};
#define FRT FRT_cal
#define alpha alpha_cal
#define beta beta_cal
#define ghkca ghkca_cal
 extern double FRT( double );
 extern double alpha( double , double );
 extern double beta( double , double );
 extern double ghkca( double );
 /* declare global and static user variables */
#define Inf Inf_cal
 double Inf[2];
#define Tau Tau_cal
 double Tau[2];
#define cai cai_cal
 double cai = 0;
#define cao cao_cal
 double cao = 0;
#define exptemp exptemp_cal
 double exptemp = 37;
#define erev erev_cal
 double erev = 125;
#define hexp hexp_cal
 double hexp = 0;
#define hq10 hq10_cal
 double hq10 = 3;
#define hbetaV0 hbetaV0_cal
 double hbetaV0 = 0;
#define hbetaB hbetaB_cal
 double hbetaB = 0;
#define hbetaA hbetaA_cal
 double hbetaA = 0;
#define hbflag hbflag_cal
 double hbflag = 0;
#define halphaV0 halphaV0_cal
 double halphaV0 = 0;
#define halphaB halphaB_cal
 double halphaB = 0;
#define halphaA halphaA_cal
 double halphaA = 0;
#define haflag haflag_cal
 double haflag = 0;
#define mexp mexp_cal
 double mexp = 2;
#define mq10 mq10_cal
 double mq10 = 1;
#define mbetaV0 mbetaV0_cal
 double mbetaV0 = -8.9;
#define mbetaB mbetaB_cal
 double mbetaB = 5;
#define mbetaA mbetaA_cal
 double mbetaA = 0.02;
#define mbflag mbflag_cal
 double mbflag = 3;
#define malphaV0 malphaV0_cal
 double malphaV0 = 5;
#define malphaB malphaB_cal
 double malphaB = -13.889;
#define malphaA malphaA_cal
 double malphaA = 1.6;
#define maflag maflag_cal
 double maflag = 2;
#define vrest vrest_cal
 double vrest = 0;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "erev_cal", "mV",
 "cao_cal", "mM",
 "cai_cal", "mM",
 "gmax_cal", "mho/cm^2",
 "i_cal", "mA/cm^2",
 "g_cal", "mho/cm^2",
 0,0
};
 static double delta_t = 1;
 static double h0 = 0;
 static double m0 = 0;
 static double v = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "erev_cal", &erev_cal,
 "vrest_cal", &vrest_cal,
 "maflag_cal", &maflag_cal,
 "malphaA_cal", &malphaA_cal,
 "malphaB_cal", &malphaB_cal,
 "malphaV0_cal", &malphaV0_cal,
 "mbflag_cal", &mbflag_cal,
 "mbetaA_cal", &mbetaA_cal,
 "mbetaB_cal", &mbetaB_cal,
 "mbetaV0_cal", &mbetaV0_cal,
 "exptemp_cal", &exptemp_cal,
 "mq10_cal", &mq10_cal,
 "mexp_cal", &mexp_cal,
 "haflag_cal", &haflag_cal,
 "halphaA_cal", &halphaA_cal,
 "halphaB_cal", &halphaB_cal,
 "halphaV0_cal", &halphaV0_cal,
 "hbflag_cal", &hbflag_cal,
 "hbetaA_cal", &hbetaA_cal,
 "hbetaB_cal", &hbetaB_cal,
 "hbetaV0_cal", &hbetaV0_cal,
 "hq10_cal", &hq10_cal,
 "hexp_cal", &hexp_cal,
 "cao_cal", &cao_cal,
 "cai_cal", &cai_cal,
 0,0
};
 static DoubVec hoc_vdoub[] = {
 "Inf_cal", Inf_cal, 2,
 "Tau_cal", Tau_cal, 2,
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
"cal",
 "gmax_cal",
 0,
 "i_cal",
 "g_cal",
 0,
 "m_cal",
 "h_cal",
 0,
 0};
 static Symbol* _ca_sym;
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
 	_p = nrn_prop_data_alloc(_mechtype, 9, _prop);
 	/*initialize range parameters*/
 	gmax = 0;
 	_prop->param = _p;
 	_prop->param_size = 9;
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 3, _prop);
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_ca_sym);
 	_ppvar[0]._pval = &prop_ion->param[3]; /* ica */
 	_ppvar[1]._pval = &prop_ion->param[4]; /* _ion_dicadv */
 
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

 void _cal_reg() {
	int _vectorized = 0;
  _initlists();
 	ion_reg("ca", -10000.);
 	_ca_sym = hoc_lookup("ca_ion");
 	register_mech(_mechanism, nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init, hoc_nrnpointerindex, 0);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
     _nrn_thread_reg(_mechtype, 2, _update_ion_pointer);
 #if NMODL_TEXT
  hoc_reg_nmodl_text(_mechtype, nmodl_file_text);
  hoc_reg_nmodl_filename(_mechtype, nmodl_filename);
#endif
  hoc_register_prop_size(_mechtype, 9, 3);
  hoc_register_dparam_semantics(_mechtype, 0, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 1, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 2, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 cal C:/Users/bpg/Desktop/Projects/CN Schools/BNNI/2021/Material/Code/Running/AssocMem/cal.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
 static double FARADAY = 96489.0;
 static double R = 8.31441;
static int _reset;
static char *modelname = "Kevins Cvode modified Generalized Hodgkin-Huxley eqn Channel Model ";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int iassign();
static int mh(double);
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static int _slist1[2], _dlist1[2];
 static int states(_threadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 () {_reset=0;
 {
   mh ( _threadargscomma_ v ) ;
   Dm = ( - m + Inf [ 0 ] ) / Tau [ 0 ] ;
   Dh = ( - h + Inf [ 1 ] ) / Tau [ 1 ] ;
   }
 return _reset;
}
 static int _ode_matsol1 () {
 mh ( _threadargscomma_ v ) ;
 Dm = Dm  / (1. - dt*( ( ( - 1.0 ) ) / Tau[0] )) ;
 Dh = Dh  / (1. - dt*( ( ( - 1.0 ) ) / Tau[1] )) ;
  return 0;
}
 /*END CVODE*/
 static int states () {_reset=0;
 {
   mh ( _threadargscomma_ v ) ;
    m = m + (1. - exp(dt*(( ( - 1.0 ) ) / Tau[0])))*(- ( ( ( Inf[0] ) ) / Tau[0] ) / ( ( ( - 1.0 ) ) / Tau[0] ) - m) ;
    h = h + (1. - exp(dt*(( ( - 1.0 ) ) / Tau[1])))*(- ( ( ( Inf[1] ) ) / Tau[1] ) / ( ( ( - 1.0 ) ) / Tau[1] ) - h) ;
   }
  return 0;
}
 
static int  mh (  double _lv ) {
   double _la , _lb , _lj , _lqq10 [ 2 ] ;
 _lqq10 [ 0 ] = pow( mq10 , ( ( celsius - exptemp ) / 10. ) ) ;
   _lqq10 [ 1 ] = pow( hq10 , ( ( celsius - exptemp ) / 10. ) ) ;
   {int  _lj ;for ( _lj = 0 ; _lj <= 1 ; _lj ++ ) {
     _la = alpha ( _threadargscomma_ _lv , ((double) _lj ) ) ;
     _lb = beta ( _threadargscomma_ _lv , ((double) _lj ) ) ;
     if ( ((double) _lj )  == 1.0  && hexp  == 0.0 ) {
       Tau [ _lj ] = 1. ;
       Inf [ _lj ] = 1. ;
       }
     else {
       Inf [ _lj ] = _la / ( _la + _lb ) ;
       Tau [ _lj ] = 1. / ( _la + _lb ) / _lqq10 [ _lj ] ;
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
 
double FRT (  double _ltemperature ) {
   double _lFRT;
 _lFRT = FARADAY * 0.001 / R / ( _ltemperature + 273.15 ) ;
   
return _lFRT;
 }
 
static void _hoc_FRT(void) {
  double _r;
   _r =  FRT (  *getarg(1) );
 hoc_retpushx(_r);
}
 
double ghkca (  double _lv ) {
   double _lghkca;
 double _lnu , _lefun ;
 _lnu = _lv * 2.0 * FRT ( _threadargscomma_ celsius ) ;
   if ( fabs ( _lnu ) < 1.e-6 ) {
     _lefun = 1. - _lnu / 2. ;
     }
   else {
     _lefun = _lnu / ( exp ( _lnu ) - 1. ) ;
     }
   _lghkca = - FARADAY * 2.e-3 * _lefun * ( cao - cai * exp ( _lnu ) ) ;
   
return _lghkca;
 }
 
static void _hoc_ghkca(void) {
  double _r;
   _r =  ghkca (  *getarg(1) );
 hoc_retpushx(_r);
}
 
static int  iassign (  ) {
   i = g * ( v - erev ) ;
   ica = i ;
    return 0; }
 
static void _hoc_iassign(void) {
  double _r;
   _r = 1.;
 iassign (  );
 hoc_retpushx(_r);
}
 
static int _ode_count(int _type){ return 2;}
 
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
	for (_i=0; _i < 2; ++_i) {
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
   nrn_update_ion_pointer(_ca_sym, _ppvar, 0, 3);
   nrn_update_ion_pointer(_ca_sym, _ppvar, 1, 4);
 }

static void initmodel() {
  int _i; double _save;_ninits++;
 _save = t;
 t = 0.0;
{
  h = h0;
  m = m0;
 {
   mh ( _threadargscomma_ v ) ;
   m = Inf [ 0 ] ;
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
   double _lhexp_val , _lindex , _lmexp_val , _lmexp2 ;
 _lhexp_val = 1.0 ;
   _lmexp_val = 1.0 ;
   if ( hexp > 0.0 ) {
     {int  _lindex ;for ( _lindex = 1 ; _lindex <= ((int) hexp ) ; _lindex ++ ) {
       _lhexp_val = h * _lhexp_val ;
       } }
     }
   if ( mexp > 0.0 ) {
     {int  _lindex ;for ( _lindex = 1 ; _lindex <= ((int) mexp ) ; _lindex ++ ) {
       _lmexp_val = m * _lmexp_val ;
       } }
     }
   else if ( mexp < 0.0 ) {
     _lmexp2 = - mexp ;
     {int  _lindex ;for ( _lindex = 1 ; _lindex <= ((int) _lmexp2 ) ; _lindex ++ ) {
       _lmexp_val = Inf [ 0 ] * _lmexp_val ;
       } }
     }
   g = gmax * _lmexp_val * _lhexp_val ;
   iassign ( _threadargs_ ) ;
   }
 _current += ica;

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
 	{ double _dica;
  _dica = ica;
 _rhs = _nrn_current(_v);
  _ion_dicadv += (_dica - ica)/.001 ;
 	}
 _g = (_g - _rhs)/.001;
  _ion_ica += ica ;
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
 if(error){fprintf(stderr,"at line 93 in file geneval_cvode.inc:\n\n"); nrn_complain(_p); abort_run(error);}
 } }}

}

static void terminal(){}

static void _initlists() {
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = &(m) - _p;  _dlist1[0] = &(Dm) - _p;
 _slist1[1] = &(h) - _p;  _dlist1[1] = &(Dh) - _p;
_first = 0;
}

#if NMODL_TEXT
static const char* nmodl_filename = "cal.mod";
static const char* nmodl_file_text = 
  "NEURON { SUFFIX cal }\n"
  "NEURON { USEION ca WRITE ica }\n"
  "ASSIGNED { ica }\n"
  "\n"
  "PARAMETER {\n"
  "	erev 		= 125    (mV)\n"
  "	gmax 		= 0    (mho/cm^2)\n"
  "        vrest           = 0\n"
  "\n"
  "	maflag 		= 2\n"
  "	malphaA 	= 1.6\n"
  "	malphaB		= -13.889\n"
  "	malphaV0	= 5.\n"
  "	mbflag 		= 3\n"
  "	mbetaA 		= 0.02\n"
  "	mbetaB		= 5.\n"
  "	mbetaV0		= -8.9\n"
  "	exptemp		= 37\n"
  "	mq10		= 1\n"
  "	mexp 		= 2\n"
  "\n"
  "	haflag 		= 0\n"
  "	halphaA 	= 0\n"
  "	halphaB		= 0\n"
  "	halphaV0	= 0\n"
  "	hbflag 		= 0\n"
  "	hbetaA 		= 0\n"
  "	hbetaB		= 0\n"
  "	hbetaV0		= 0\n"
  "	hq10		= 3\n"
  "	hexp 		= 0\n"
  "}\n"
  "\n"
  ":::INCLUDE \"geneval_cvode.inc\"\n"
  ": $Id: geneval_cvode.inc,v 1.6 2004/02/04 21:04:15 billl Exp $  \n"
  "TITLE Kevins Cvode modified Generalized Hodgkin-Huxley eqn Channel Model \n"
  "\n"
  "COMMENT\n"
  "\n"
  "Each channel has activation and inactivation particles as in the original\n"
  "Hodgkin Huxley formulation.  The activation particle mm and inactivation\n"
  "particle hh go from on to off states according to kinetic variables alpha\n"
  "and beta which are voltage dependent.\n"
  "Allows exponential, sigmoid and linoid forms (flags 0,1,2)\n"
  "See functions alpha() and beta() for details of parameterization\n"
  "\n"
  "ENDCOMMENT\n"
  "\n"
  "INDEPENDENT {t FROM 0 TO 1 WITH 1 (ms)}\n"
  "\n"
  "NEURON {\n"
  "	RANGE gmax, g, i\n"
  "	GLOBAL erev, Inf, Tau, vrest\n"
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
  "COMMENT\n"
  "** Parameter values should come from files specific to particular channels\n"
  "\n"
  "PARAMETER {\n"
  "	erev 		= 0    (mV)\n"
  "	gmax 		= 0    (mho/cm^2)\n"
  "\n"
  "	maflag 		= 0\n"
  "	malphaA 	= 0\n"
  "	malphaB		= 0\n"
  "	malphaV0	= 0\n"
  "	mbflag 		= 0\n"
  "	mbetaA 		= 0\n"
  "	mbetaB		= 0\n"
  "	mbetaV0		= 0\n"
  "	exptemp		= 0\n"
  "	mq10		= 3\n"
  "	mexp 		= 0\n"
  "\n"
  "	haflag 		= 0\n"
  "	halphaA 	= 0\n"
  "	halphaB		= 0\n"
  "	halphaV0	= 0\n"
  "	hbflag 		= 0\n"
  "	hbetaA 		= 0\n"
  "	hbetaB		= 0\n"
  "	hbetaV0		= 0\n"
  "	hq10		= 3\n"
  "	hexp 		= 0\n"
  "} : end PARAMETER\n"
  "ENDCOMMENT\n"
  "\n"
  "PARAMETER {\n"
  "  cao                (mM)\n"
  "  cai                (mM)\n"
  "  celsius			   (degC)\n"
  "  dt 				   (ms)\n"
  "  v 			       (mV)\n"
  "}\n"
  "\n"
  "ASSIGNED {\n"
  "	i (mA/cm^2)		\n"
  "	g (mho/cm^2)\n"
  "	Inf[2]		: 0 = m and 1 = h\n"
  "	Tau[2]		: 0 = m and 1 = h\n"
  "} : end ASSIGNED \n"
  "\n"
  "STATE { m h }\n"
  "\n"
  "INITIAL { \n"
  " 	mh(v)\n"
  "	m = Inf[0] h = Inf[1]\n"
  "}\n"
  "\n"
  "BREAKPOINT {\n"
  "\n"
  "  LOCAL hexp_val, index, mexp_val, mexp2\n"
  "\n"
  "  SOLVE states METHOD cnexp\n"
  "\n"
  "  hexp_val = 1\n"
  "  mexp_val = 1\n"
  "\n"
  "  : Determining h's exponent value\n"
  "  if (hexp > 0) {\n"
  "    FROM index=1 TO hexp {\n"
  "      hexp_val = h * hexp_val\n"
  "    }\n"
  "  }\n"
  "\n"
  "  : Determining m's exponent value\n"
  "  if (mexp > 0) {\n"
  "    FROM index = 1 TO mexp {\n"
  "      mexp_val = m * mexp_val\n"
  "    }\n"
  "  } else if (mexp<0) {\n"
  "    mexp2=-mexp\n"
  "    FROM index = 1 TO mexp2 {\n"
  "      mexp_val = Inf[0] * mexp_val\n"
  "    }\n"
  "  }\n"
  "\n"
  "  :			       mexp			    hexp\n"
  "  : Note that mexp_val is now = m      and hexp_val is now = h \n"
  "  g = gmax * mexp_val * hexp_val\n"
  "\n"
  "  iassign()\n"
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
  "  mh(v)\n"
  "  m' = (-m + Inf[0]) / Tau[0] \n"
  "  h' = (-h + Inf[1]) / Tau[1]\n"
  "}\n"
  "\n"
  ":-------------------------------------------------------------------\n"
  ": NOTE : 0 = m and 1 = h\n"
  "PROCEDURE mh (v) {\n"
  "  LOCAL a, b, j, qq10[2]\n"
  "\n"
  "  qq10[0] = mq10^((celsius-exptemp)/10.)	\n"
  "  qq10[1] = hq10^((celsius-exptemp)/10.)	\n"
  "\n"
  "  : Calculater Inf and Tau values for h and m\n"
  "  FROM j = 0 TO 1 {\n"
  "    a = alpha (v, j)\n"
  "    b = beta (v, j)\n"
  "\n"
  "    if (j==1 && hexp==0) { Tau[j] = 1. Inf[j] = 1.\n"
  "    } else {\n"
  "      Inf[j] = a / (a + b)\n"
  "      Tau[j] = 1. / (a + b) / qq10[j]\n"
  "    }\n"
  "  }\n"
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
  "\n"
  ":-------------------------------------------------------------------\n"
  "FUNCTION FRT(temperature) {\n"
  "	FRT = FARADAY * 0.001 / R / (temperature + 273.15)\n"
  "} : end FUNCTION FRT (temperature)\n"
  "\n"
  ":-------------------------------------------------------------------\n"
  " FUNCTION ghkca (v) { : Goldman-Hodgkin-Katz eqn\n"
  "       LOCAL nu, efun\n"
  "\n"
  "       nu = v*2*FRT(celsius)\n"
  "       if(fabs(nu) < 1.e-6) {\n"
  "               efun = 1.- nu/2.\n"
  "       } else {\n"
  "               efun = nu/(exp(nu)-1.) }\n"
  "\n"
  "       ghkca = -FARADAY*2.e-3*efun*(cao - cai*exp(nu))\n"
  " } : end FUNCTION ghkca()\n"
  ":::end INCLUDE geneval_cvode.inc\n"
  "\n"
  "PROCEDURE iassign () { i=g*(v-erev) ica=i }\n"
  "\n"
  ":* defaults\n"
  ;
#endif
