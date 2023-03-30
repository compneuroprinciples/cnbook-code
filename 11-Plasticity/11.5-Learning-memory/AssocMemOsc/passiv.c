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
 
#define nrn_init _nrn_init__Pass
#define _nrn_initial _nrn_initial__Pass
#define nrn_cur _nrn_cur__Pass
#define _nrn_current _nrn_current__Pass
#define nrn_jacob _nrn_jacob__Pass
#define nrn_state _nrn_state__Pass
#define _net_receive _net_receive__Pass 
 
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
#define g _p[0]
#define erev _p[1]
#define i _p[2]
#define v _p[3]
#define _g _p[4]
 
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
 "setdata_Pass", _hoc_setdata,
 0, 0
};
 /* declare global and static user variables */
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "g_Pass", "mho/cm2",
 "erev_Pass", "mV",
 "i_Pass", "mA/cm2",
 0,0
};
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
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
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"Pass",
 "g_Pass",
 "erev_Pass",
 0,
 "i_Pass",
 0,
 0,
 0};
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
 	_p = nrn_prop_data_alloc(_mechtype, 5, _prop);
 	/*initialize range parameters*/
 	g = 0.001;
 	erev = -70;
 	_prop->param = _p;
 	_prop->param_size = 5;
 
}
 static void _initlists();
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, _NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _passiv_reg() {
	int _vectorized = 1;
  _initlists();
 	register_mech(_mechanism, nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init, hoc_nrnpointerindex, 1);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
 #if NMODL_TEXT
  hoc_reg_nmodl_text(_mechtype, nmodl_file_text);
  hoc_reg_nmodl_filename(_mechtype, nmodl_filename);
#endif
  hoc_register_prop_size(_mechtype, 5, 0);
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 Pass C:/Users/bpg/Desktop/Projects/CN Schools/BNNI/2021/Material/Code/Running/AssocMem/passiv.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static char *modelname = "passive membrane channel";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}

static void initmodel(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
  int _i; double _save;{

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
   i = g * ( v - erev ) ;
   }
 _current += i;

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
 	{ _rhs = _nrn_current(_p, _ppvar, _thread, _nt, _v);
 	}
 _g = (_g - _rhs)/.001;
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

}

static void terminal(){}

static void _initlists(){
 double _x; double* _p = &_x;
 int _i; static int _first = 1;
  if (!_first) return;
_first = 0;
}

#if defined(__cplusplus)
} /* extern "C" */
#endif

#if NMODL_TEXT
static const char* nmodl_filename = "passiv.mod";
static const char* nmodl_file_text = 
  "TITLE passive membrane channel\n"
  "\n"
  "UNITS {\n"
  "	(mV) = (millivolt)\n"
  "	(mA) = (milliamp)\n"
  "}\n"
  "\n"
  "INDEPENDENT { v FROM -100 TO 50 WITH 50	(mV) }\n"
  "\n"
  "NEURON {\n"
  "	SUFFIX Pass\n"
  "	NONSPECIFIC_CURRENT i\n"
  "	RANGE g,erev\n"
  "}\n"
  "\n"
  "PARAMETER {\n"
  "	g = .001	(mho/cm2)\n"
  "	erev = -70	(mV)\n"
  "}\n"
  "\n"
  "ASSIGNED { i	(mA/cm2)}\n"
  "\n"
  "BREAKPOINT {\n"
  "	i = g*(v - erev)\n"
  "}\n"
  "\n"
  "COMMENT\n"
  "The passive channel is very simple but illustrates several features of\n"
  "the interface to NEURON. As a SCoP or hoc model the NEURON block is\n"
  "ignored.  About the only thing you can do with this as an isolated channel\n"
  "in SCoP is plot the current vs the potential. Notice that models require\n"
  "that all variables be declared, The calculation is done in the EQUATION\n"
  "block (This name may eventually be changed to MODEL).  The intended\n"
  "semantics of the equation block are that after the block is executed, ALL\n"
  "variables are consistent with the value of the independent variable.\n"
  "In this case, of course a trivial assignment statement suffices.\n"
  "In SCoP, INDEPENDENT gives the name and range of the independent variable,\n"
  "CONSTANT declares variables which generally do not change during\n"
  "solution of the EQUATION block and ASSIGNED declares variables which\n"
  "get values via assignment statements (as opposed to STATE variables whose\n"
  "values can only be determined by solving differential or simultaneous\n"
  "algebraic equations.)  The values of CONSTANTS are the default values\n"
  "and can be changed in SCoP.\n"
  "\n"
  "The NEURON block serves as the interface to NEURON. One has to imagine\n"
  "many models linked to NEURON at the same time. Therefore in order to\n"
  "avoid conflicts with names of variables in other mechanisms a SUFFIX\n"
  "is applied to all the declared names that are accessible from NEURON.\n"
  "Accessible CONSTANTS are of two types. Those appearing in the\n"
  "PARAMETER list become range variables that can be used in any section\n"
  "in which the mechanism is \"insert\"ed.  CONSTANT's that do not appear in\n"
  "the PARAMETER list become global scalars which are the same for every\n"
  "section.  ASSIGNED variables and STATE variables also become range variables\n"
  "that depend on position in a section.\n"
  "NONSPECIFIC_CURRENT specifies a list of currents not associated with\n"
  "any particular ion but computed by this model\n"
  "that affect the calculation of the membrane potential. I.e. a nonspecific\n"
  "current adds its contribution to the total membrane current.\n"
  "\n"
  "The following  neuron program is suitable for investigating the behavior\n"
  "of the channel and determining its effect on the membrane.\n"
  "create a\n"
  "access a\n"
  "nseg = 1\n"
  "insert Passive\n"
  "g_Passive=.001\n"
  "erev_Passive=0\n"
  "proc cur() {\n"
  "	axis(0,1,1,0,.001,1) axis()\n"
  "	plot(1)\n"
  "	for (v=0; v < 1; v=v+.01) {\n"
  "		fcurrent()\n"
  "		plot(v, i_Passive)\n"
  "	}\n"
  "	plt(-1)\n"
  "}	\n"
  "\n"
  "proc run() {\n"
  "	axis(0,3,3,0,1,1) axis()\n"
  "	t = 0\n"
  "	v=1\n"
  "	plot(1)\n"
  "	while (t < 3) {\n"
  "		plot(t,v)\n"
  "		fadvance()\n"
  "	}\n"
  "}\n"
  "/* the cur() procedure uses the fcurrent() function of neuron to calculate\n"
  "all the currents and conductances with all states (including v) held\n"
  "constant.  In the run() procedure fadvance() integrates all equations\n"
  "by one time step. In this case the Passive channel in combination with\n"
  "the default capacitance of 1uF/cm2 give a membrane with a time constant of\n"
  "1 ms. Thus the voltage decreases exponentially toward 0 from its initial\n"
  "value of 1.\n"
  "\n"
  "ENDCOMMENT\n"
  ;
#endif
