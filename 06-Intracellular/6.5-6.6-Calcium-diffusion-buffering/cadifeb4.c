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
 
#define nrn_init _nrn_init__cadifeb4
#define _nrn_initial _nrn_initial__cadifeb4
#define nrn_cur _nrn_cur__cadifeb4
#define _nrn_current _nrn_current__cadifeb4
#define nrn_jacob _nrn_jacob__cadifeb4
#define nrn_state _nrn_state__cadifeb4
#define _net_receive _net_receive__cadifeb4 
#define coord coord__cadifeb4 
#define state state__cadifeb4 
 
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
#define cai0 _p[0]
#define ca (_p + 1)
#define ica _p[5]
#define cai _p[6]
#define Kd _p[7]
#define B0 _p[8]
#define Dca (_p + 9)
#define _g _p[13]
#define _ion_ica	*_ppvar[0]._pval
#define _ion_cai	*_ppvar[1]._pval
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
 extern double celsius;
 /* declaration of user functions */
 static void _hoc_coord(void);
 static void _hoc_ss(void);
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
 "setdata_cadifeb4", _hoc_setdata,
 "coord_cadifeb4", _hoc_coord,
 "ss_cadifeb4", _hoc_ss,
 0, 0
};
#define ss ss_cadifeb4
 extern double ss( );
 /* declare global and static user variables */
#define DFree DFree_cadifeb4
 double DFree = 0.23;
#define TotalBuffer TotalBuffer_cadifeb4
 double TotalBuffer = 0.003;
#define cao cao_cadifeb4
 double cao = 0;
#define k2buf k2buf_cadifeb4
 double k2buf = 0.1;
#define k1buf k1buf_cadifeb4
 double k1buf = 100;
#define sthick sthick_cadifeb4
 double sthick = 0.1;
#define vol vol_cadifeb4
 double vol[4];
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "cao_cadifeb4", "mM",
 "sthick_cadifeb4", "um",
 "DFree_cadifeb4", "um2/ms",
 "k1buf_cadifeb4", "/mM-ms",
 "k2buf_cadifeb4", "/ms",
 "TotalBuffer_cadifeb4", "mM",
 "vol_cadifeb4", "1",
 "cai0_cadifeb4", "mM",
 "ca_cadifeb4", "mM",
 0,0
};
 static double ca0 = 0;
 static double delta_t = 0.01;
 static double v = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "cao_cadifeb4", &cao_cadifeb4,
 "sthick_cadifeb4", &sthick_cadifeb4,
 "DFree_cadifeb4", &DFree_cadifeb4,
 "k1buf_cadifeb4", &k1buf_cadifeb4,
 "k2buf_cadifeb4", &k2buf_cadifeb4,
 "TotalBuffer_cadifeb4", &TotalBuffer_cadifeb4,
 0,0
};
 static DoubVec hoc_vdoub[] = {
 "vol_cadifeb4", vol_cadifeb4, 4,
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
"cadifeb4",
 "cai0_cadifeb4",
 0,
 0,
 "ca_cadifeb4[4]",
 0,
 0};
 static Symbol* _morphology_sym;
 static Symbol* _ca_sym;
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
 	_p = nrn_prop_data_alloc(_mechtype, 14, _prop);
 	/*initialize range parameters*/
 	cai0 = 5e-05;
 	_prop->param = _p;
 	_prop->param_size = 14;
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 5, _prop);
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_morphology_sym);
 	_ppvar[3]._pval = &prop_ion->param[0]; /* diam */
 prop_ion = need_memb(_ca_sym);
 nrn_check_conc_write(_prop, prop_ion, 1);
 nrn_promote(prop_ion, 3, 0);
 	_ppvar[0]._pval = &prop_ion->param[3]; /* ica */
 	_ppvar[1]._pval = &prop_ion->param[1]; /* cai */
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

 void _cadifeb4_reg() {
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
  hoc_register_prop_size(_mechtype, 14, 5);
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
 	ivoc_help("help ?1 cadifeb4 C:/Users/bpg/Desktop/Projects/CN book/Edition2/Chapter drafts/Chapt 6 - Intracellular/Scripts/My_Spine/cadifeb4.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
 
#define FARADAY _nrnunit_FARADAY[_nrnunit_use_legacy_]
static double _nrnunit_FARADAY[2] = {0xb.c72aa8304416p+13, 96485.3}; /* 96485.3321233100141 */
 
#define PI _nrnunit_PI[_nrnunit_use_legacy_]
static double _nrnunit_PI[2] = {0xc.90fdaa22168cp-2, 3.14159}; /* 3.14159265358979312 */
 
#define R _nrnunit_R[_nrnunit_use_legacy_]
static double _nrnunit_R[2] = {0x8.50809f44c85fp+0, 8.3145}; /* 8.3144626181532395 */
 static double _zfrat [ 4 ] ;
 static double _zdsq , _zdsqvol ;
static int _reset;
static char *modelname = "Calcium ion accumulation and diffusion with kinetic buffering";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int coord();
 extern double *_getelm();
 
#define _MATELM1(_row,_col)	*(_getelm(_row + 1, _col + 1))
 
#define _RHS1(_arg) _coef1[_arg + 1]
 static double *_coef1;
 
#define _linmat1  1
 static void* _sparseobj1;
 static void* _cvsparseobj1;
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static int _slist1[4], _dlist1[4]; static double *_temp1;
 static int state();
 
static int  coord (  ) {
   double _lr , _ldr2 , _ldrs ;
 _lr = 1.0 / 2.0 ;
   _ldrs = sthick / diam ;
   _ldr2 = ( _lr - _ldrs ) / ( 2.0 * 4.0 - 3.0 ) ;
   vol [ 0 ] = PI * ( _lr - _ldrs / 2.0 ) * 2.0 * _ldrs ;
   _zfrat [ 0 ] = 2.0 * _lr ;
   _lr = _lr - _ldrs ;
   _zfrat [ 1 ] = 2.0 * PI * _lr / ( _ldrs + _ldr2 ) ;
   _lr = _lr - _ldr2 ;
   vol [ 1 ] = PI * ( _lr + _ldr2 / 2.0 ) * 2.0 * _ldr2 ;
   {int  _li ;for ( _li = 1 ; _li <= 4 - 2 ; _li ++ ) {
     vol [ _li ] = vol [ _li ] + PI * ( _lr - _ldr2 / 2.0 ) * 2.0 * _ldr2 ;
     _lr = _lr - _ldr2 ;
     _zfrat [ _li + 1 ] = 2.0 * PI * _lr / ( 2.0 * _ldr2 ) ;
     _lr = _lr - _ldr2 ;
     vol [ _li + 1 ] = PI * ( _lr + _ldr2 / 2.0 ) * 2.0 * _ldr2 ;
     } }
    return 0; }
 
static void _hoc_coord(void) {
  double _r;
   _r = 1.;
 coord (  );
 hoc_retpushx(_r);
}
 
static int state ()
 {_reset=0;
 {
   double b_flux, f_flux, _term; int _i;
 {int _i; double _dt1 = 1.0/dt;
for(_i=0;_i<4;_i++){
  	_RHS1(_i) = -_dt1*(_p[_slist1[_i]] - _p[_dlist1[_i]]);
	_MATELM1(_i, _i) = _dt1;
      
} 
for (_i=0; _i < 4; _i++) {
  	_RHS1(_i + 0) *= ( diam * diam * vol [ ((int) _i ) ] * 1.0) ;
_MATELM1(_i + 0, _i + 0) *= ( diam * diam * vol [ ((int) _i ) ] * 1.0);  } }
 /* COMPARTMENT _li , diam * diam * vol [ ((int) _i ) ] * 1.0 {
     ca }
   */
 /* ~ ca [ 0 ] < < ( - ica * PI * diam * 1.0 * ( 1e4 ) * _zfrat [ 0 ] / ( 2.0 * FARADAY ) )*/
 f_flux = b_flux = 0.;
 _RHS1( 0 +  0) += (b_flux =   ( - ica * PI * diam * 1.0 * ( 1e4 ) * _zfrat [ 0 ] / ( 2.0 * FARADAY ) ) );
 /*FLUX*/
  {int  _li ;for ( _li = 0 ; _li <= 4 - 2 ; _li ++ ) {
     /* ~ ca [ _li ] <-> ca [ _li + 1 ] ( DFree * _zfrat [ _li + 1 ] * 1.0 , DFree * _zfrat [ _li + 1 ] * 1.0 )*/
 f_flux =  DFree * _zfrat [ _li + 1 ] * 1.0 * ca [ _li] ;
 b_flux =  DFree * _zfrat [ _li + 1 ] * 1.0 * ca [ _li + 1] ;
 _RHS1( 0 +  _li) -= (f_flux - b_flux);
 _RHS1( 0 +  _li + 1) += (f_flux - b_flux);
 
 _term =  DFree * _zfrat [ _li + 1 ] * 1.0 ;
 _MATELM1( 0 +  _li ,0 +  _li)  += _term;
 _MATELM1( 0 +  _li + 1 ,0 +  _li)  -= _term;
 _term =  DFree * _zfrat [ _li + 1 ] * 1.0 ;
 _MATELM1( 0 +  _li ,0 +  _li + 1)  -= _term;
 _MATELM1( 0 +  _li + 1 ,0 +  _li + 1)  += _term;
 /*REACTION*/
  } }
   _zdsq = diam * diam ;
   {int  _li ;for ( _li = 0 ; _li <= 4 - 1 ; _li ++ ) {
     _zdsqvol = _zdsq * vol [ _li ] ;
     /* ~ ca [ _li ] <-> cai0 ( k1buf * B0 * _zdsqvol * 1.0 , k1buf * B0 * _zdsqvol * 1.0 )*/
 f_flux =  k1buf * B0 * _zdsqvol * 1.0 * ca [ _li] ;
 b_flux =  k1buf * B0 * _zdsqvol * 1.0 * cai0 ;
 _RHS1( 0 +  _li) -= (f_flux - b_flux);
 
 _term =  k1buf * B0 * _zdsqvol * 1.0 ;
 _MATELM1( 0 +  _li ,0 +  _li)  += _term;
 /*REACTION*/
  } }
   cai = ca [ 0 ] ;
     } return _reset;
 }
 
double ss (  ) {
   double _lss;
 error = _ss_sparse(&_sparseobj1, 4, _slist1, _dlist1, _p, &t, dt, state,&_coef1, _linmat1);
 if(error){fprintf(stderr,"at line 132 in file cadifeb4.mod:\n	SOLVE state STEADYSTATE sparse\n"); nrn_complain(_p); abort_run(error);}
    if (secondorder) {
    int _i;
    for (_i = 0; _i < 4; ++_i) {
      _p[_slist1[_i]] += dt*_p[_dlist1[_i]];
    }}
 _lss = cai ;
   
return _lss;
 }
 
static void _hoc_ss(void) {
  double _r;
   _r =  ss (  );
 hoc_retpushx(_r);
}
 
/*CVODE ode begin*/
 static int _ode_spec1() {_reset=0;{
 double b_flux, f_flux, _term; int _i;
 {int _i; for(_i=0;_i<4;_i++) _p[_dlist1[_i]] = 0.0;}
 /* COMPARTMENT _li , diam * diam * vol [ ((int) _i ) ] * 1.0 {
   ca }
 */
 /* ~ ca [ 0 ] < < ( - ica * PI * diam * 1.0 * ( 1e4 ) * _zfrat [ 0 ] / ( 2.0 * FARADAY ) )*/
 f_flux = b_flux = 0.;
 Dca [ 0] += (b_flux =   ( - ica * PI * diam * 1.0 * ( 1e4 ) * _zfrat [ 0 ] / ( 2.0 * FARADAY ) ) );
 /*FLUX*/
  {int  _li ;for ( _li = 0 ; _li <= 4 - 2 ; _li ++ ) {
   /* ~ ca [ _li ] <-> ca [ _li + 1 ] ( DFree * _zfrat [ _li + 1 ] * 1.0 , DFree * _zfrat [ _li + 1 ] * 1.0 )*/
 f_flux =  DFree * _zfrat [ _li + 1 ] * 1.0 * ca [ _li] ;
 b_flux =  DFree * _zfrat [ _li + 1 ] * 1.0 * ca [ _li + 1] ;
 Dca [ _li] -= (f_flux - b_flux);
 Dca [ _li + 1] += (f_flux - b_flux);
 
 /*REACTION*/
  } }
 _zdsq = diam * diam ;
 {int  _li ;for ( _li = 0 ; _li <= 4 - 1 ; _li ++ ) {
   _zdsqvol = _zdsq * vol [ _li ] ;
   /* ~ ca [ _li ] <-> cai0 ( k1buf * B0 * _zdsqvol * 1.0 , k1buf * B0 * _zdsqvol * 1.0 )*/
 f_flux =  k1buf * B0 * _zdsqvol * 1.0 * ca [ _li] ;
 b_flux =  k1buf * B0 * _zdsqvol * 1.0 * cai0 ;
 Dca [ _li] -= (f_flux - b_flux);
 
 /*REACTION*/
  } }
 cai = ca [ 0 ] ;
 for (_i=0; _i < 4; _i++) { _p[_dlist1[_i + 0]] /= ( diam * diam * vol [ ((int) _i ) ] * 1.0);}
   } return _reset;
 }
 
/*CVODE matsol*/
 static int _ode_matsol1() {_reset=0;{
 double b_flux, f_flux, _term; int _i;
   b_flux = f_flux = 0.;
 {int _i; double _dt1 = 1.0/dt;
for(_i=0;_i<4;_i++){
  	_RHS1(_i) = _dt1*(_p[_dlist1[_i]]);
	_MATELM1(_i, _i) = _dt1;
      
} 
for (_i=0; _i < 4; _i++) {
  	_RHS1(_i + 0) *= ( diam * diam * vol [ ((int) _i ) ] * 1.0) ;
_MATELM1(_i + 0, _i + 0) *= ( diam * diam * vol [ ((int) _i ) ] * 1.0);  } }
 /* COMPARTMENT _li , diam * diam * vol [ ((int) _i ) ] * 1.0 {
 ca }
 */
 /* ~ ca [ 0 ] < < ( - ica * PI * diam * 1.0 * ( 1e4 ) * _zfrat [ 0 ] / ( 2.0 * FARADAY ) )*/
 /*FLUX*/
  {int  _li ;for ( _li = 0 ; _li <= 4 - 2 ; _li ++ ) {
 /* ~ ca [ _li ] <-> ca [ _li + 1 ] ( DFree * _zfrat [ _li + 1 ] * 1.0 , DFree * _zfrat [ _li + 1 ] * 1.0 )*/
 _term =  DFree * _zfrat [ _li + 1 ] * 1.0 ;
 _MATELM1( 0 +  _li ,0 +  _li)  += _term;
 _MATELM1( 0 +  _li + 1 ,0 +  _li)  -= _term;
 _term =  DFree * _zfrat [ _li + 1 ] * 1.0 ;
 _MATELM1( 0 +  _li ,0 +  _li + 1)  -= _term;
 _MATELM1( 0 +  _li + 1 ,0 +  _li + 1)  += _term;
 /*REACTION*/
  } }
 _zdsq = diam * diam ;
 {int  _li ;for ( _li = 0 ; _li <= 4 - 1 ; _li ++ ) {
 _zdsqvol = _zdsq * vol [ _li ] ;
 /* ~ ca [ _li ] <-> cai0 ( k1buf * B0 * _zdsqvol * 1.0 , k1buf * B0 * _zdsqvol * 1.0 )*/
 _term =  k1buf * B0 * _zdsqvol * 1.0 ;
 _MATELM1( 0 +  _li ,0 +  _li)  += _term;
 /*REACTION*/
  } }
 cai = ca [ 0 ] ;
   } return _reset;
 }
 
/*CVODE end*/
 
static int _ode_count(int _type){ return 4;}
 
static void _ode_spec(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
  ica = _ion_ica;
  cai = _ion_cai;
  cai = _ion_cai;
     _ode_spec1 ();
  _ion_cai = cai;
 }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 4; ++_i) {
		_pv[_i] = _pp + _slist1[_i];  _pvdot[_i] = _pp + _dlist1[_i];
		_cvode_abstol(_atollist, _atol, _i);
	}
 }
 static void _ode_synonym(int _cnt, double** _pp, Datum** _ppd) { 
 	int _i; 
	for (_i=0; _i < _cnt; ++_i) {_p = _pp[_i]; _ppvar = _ppd[_i];
 _ion_cai =  ca [ 0 ] ;
 }}
 
static void _ode_matsol_instance1(_threadargsproto_) {
 _cvode_sparse(&_cvsparseobj1, 4, _dlist1, _p, _ode_matsol1, &_coef1);
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
  ica = _ion_ica;
  cai = _ion_cai;
  cai = _ion_cai;
 _ode_matsol_instance1(_threadargs_);
 }}
 extern void nrn_update_ion_pointer(Symbol*, Datum*, int, int);
 static void _update_ion_pointer(Datum* _ppvar) {
   nrn_update_ion_pointer(_ca_sym, _ppvar, 0, 3);
   nrn_update_ion_pointer(_ca_sym, _ppvar, 1, 1);
 }

static void initmodel() {
  int _i; double _save;_ninits++;
 _save = t;
 t = 0.0;
{
 for (_i=0; _i<4; _i++) ca[_i] = ca0;
 {
   double _ltotal ;
 if ( sthick > diam / 4.0 ) {
     sthick = diam / 4.0 ;
     }
   coord ( _threadargs_ ) ;
   cai = cai0 ;
   Kd = k1buf / k2buf ;
   B0 = TotalBuffer / ( 1.0 + Kd * cai ) ;
   {int  _li ;for ( _li = 0 ; _li <= 4 - 1 ; _li ++ ) {
     ca [ _li ] = cai ;
     } }
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
  ica = _ion_ica;
  cai = _ion_cai;
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
  ica = _ion_ica;
  cai = _ion_cai;
  cai = _ion_cai;
 { error = sparse(&_sparseobj1, 4, _slist1, _dlist1, _p, &t, dt, state,&_coef1, _linmat1);
 if(error){fprintf(stderr,"at line 78 in file cadifeb4.mod:\n	SOLVE state METHOD sparse\n"); nrn_complain(_p); abort_run(error);}
    if (secondorder) {
    int _i;
    for (_i = 0; _i < 4; ++_i) {
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
 for(_i=0;_i<4;_i++){_slist1[0+_i] = (ca + _i) - _p;  _dlist1[0+_i] = (Dca + _i) - _p;}
_first = 0;
}

#if NMODL_TEXT
static const char* nmodl_filename = "cadifeb4.mod";
static const char* nmodl_file_text = 
  "TITLE Calcium ion accumulation and diffusion with kinetic buffering\n"
  ": The internal coordinate system is set up in PROCEDURE coord()\n"
  ": and must be executed before computing the concentrations.\n"
  ": The scale factors set up in this procedure do not have to be recomputed\n"
  ": when diam or DFree are changed.\n"
  ": The amount of calcium in an annulus is ca[i]*diam^2*vol[i] with\n"
  ": ca[0] being the second order correct concentration at the exact edge\n"
  ": and ca[NANN-1] being the concentration at the exact center\n"
  "\n"
  ": Version with fixed shell thickness.\n"
  ": sthick is a range variable set from HOC.\n"
  ": Excess buffer approximation.\n"
  ": Mods BPG 6-2-07\n"
  "\n"
  "NEURON {\n"
  "	SUFFIX cadifeb4\n"
  "	USEION ca READ ica, cai WRITE cai\n"
  "	GLOBAL vol, TotalBuffer\n"
  "	RANGE cai0\n"
  "}\n"
  "\n"
  "DEFINE NANN  4\n"
  "\n"
  "UNITS {\n"
  "	(mV)	= (millivolt)\n"
  "	(molar) = (1/liter)\n"
  "	(mM)	= (millimolar)\n"
  "	(um)	= (micron)\n"
  "	(mA)	= (milliamp)\n"
  "	(mol)	= (1)\n"
  "	FARADAY = (faraday)	 (coulomb)\n"
  "	PI	= (pi)		(1)\n"
  "	R 	= (k-mole)	(joule/degC)\n"
  "}\n"
  "\n"
  "PARAMETER {\n"
  "	celsius		(degC)\n"
  "	diam		(um)\n"
  "	cao		(mM)\n"
  "	\n"
  "	sthick = 0.1	(um)		: submembrane shell thickness\n"
  "	DFree = .23	(um2/ms)	: diffusion coeff\n"
  "\n"
  "	: to change rate of buffering without disturbing equilibrium\n"
  "	: multiply the following two by the same factor\n"
  "	k1buf	= 100	(/mM-ms)\n"
  "	k2buf	= 0.1	(/ms)\n"
  "	TotalBuffer = 0.003	(mM)\n"
  "	cai0 = 50e-6 (mM)	: Requires explicit use in INITIAL block\n"
  "}\n"
  "\n"
  "ASSIGNED {\n"
  "	ica		(mA/cm2)\n"
  "	cai		(mM)\n"
  "	vol[NANN]	(1)	: gets extra cm2 when multiplied by diam^2\n"
  "	Kd		(/mM)\n"
  "	B0		(mM)\n"
  "}\n"
  "\n"
  "STATE {\n"
  "	ca[NANN]	(mM) : ca[0] is equivalent to cai\n"
  "}\n"
  "\n"
  "INITIAL {LOCAL total\n"
  "	if (sthick > diam/4) {\n"
  "		sthick = diam/4	: limit outer shell thickness\n"
  "	}\n"
  "	coord()\n"
  "	cai = cai0\n"
  "	Kd = k1buf/k2buf\n"
  "	B0 = TotalBuffer/(1 + Kd*cai)\n"
  "	FROM i=0 TO NANN-1 {\n"
  "		ca[i] = cai\n"
  "	}\n"
  "}\n"
  "\n"
  "BREAKPOINT {\n"
  "	SOLVE state METHOD sparse\n"
  "}\n"
  "\n"
  "LOCAL frat[NANN] 	: gets extra cm when multiplied by diam\n"
  "\n"
  "PROCEDURE coord() {\n"
  "	LOCAL r, dr2, drs\n"
  "	: cylindrical coordinate system  with constant annuli thickness to\n"
  "	: center of cell. Note however that the first annulus is half thickness\n"
  "	: so that the concentration is second order correct spatially at\n"
  "	: the membrane or exact edge of the cell.\n"
  "	: note ca[0] is at edge of cell\n"
  "	:      ca[NANN-1] is at center of cell\n"
  "	r = 1/2					:starts at edge (half diam)\n"
  "	drs = sthick/diam			:relative shell thickness\n"
  "	dr2 = (r-drs)/(2*NANN-3)		:half thickness of annulus\n"
  "	vol[0] = PI*(r-drs/2)*2*drs\n"
  "	frat[0] = 2*r\n"
  "	r = r - drs\n"
  "	frat[1] = 2*PI*r/(drs+dr2)\n"
  "	r = r - dr2\n"
  "	vol[1] = PI*(r+dr2/2)*2*dr2	:outer half of annulus\n"
  "	FROM i=1 TO NANN-2 {\n"
  "		vol[i] = vol[i] + PI*(r-dr2/2)*2*dr2	:interior half\n"
  "		r = r - dr2\n"
  "		frat[i+1] = 2*PI*r/(2*dr2)	:exterior edge of annulus\n"
  "					: divided by distance between centers\n"
  "		r = r - dr2\n"
  "		vol[i+1] = PI*(r+dr2/2)*2*dr2	:outer half of annulus\n"
  "	}\n"
  "}\n"
  "\n"
  "LOCAL dsq, dsqvol	: can't define local variable in KINETIC block \n"
  "			: or use in COMPARTMENT\n"
  "\n"
  "KINETIC state {\n"
  "	COMPARTMENT i, diam*diam*vol[i]*1(um) {ca}\n"
  "\n"
  "	: all currents except pump\n"
  "	~ ca[0] << (-ica*PI*diam*1(um)*(1e4)*frat[0]/(2*FARADAY))\n"
  "	:diffusion\n"
  "	FROM i=0 TO NANN-2 {\n"
  "		~ ca[i] <-> ca[i+1] (DFree*frat[i+1]*1(um), DFree*frat[i+1]*1(um))\n"
  "	}\n"
  "	: excess buffer\n"
  "	dsq = diam*diam\n"
  "	FROM i=0 TO NANN-1 {\n"
  "		dsqvol = dsq*vol[i]\n"
  "		~ ca[i] <-> cai0 (k1buf*B0*dsqvol*1(um), k1buf*B0*dsqvol*1(um))\n"
  "	}\n"
  "	cai = ca[0]\n"
  "}\n"
  "	\n"
  "FUNCTION ss() (mM) {\n"
  "	SOLVE state STEADYSTATE sparse\n"
  "	ss = cai\n"
  "}\n"
  ;
#endif
