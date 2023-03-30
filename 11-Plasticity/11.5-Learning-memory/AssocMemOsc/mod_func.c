#include <stdio.h>
#include "hocdec.h"
#define IMPORT extern __declspec(dllimport)
IMPORT int nrnmpi_myid, nrn_nobanner_;

extern void _cal_reg();
extern void _expsid_reg();
extern void _kcRT03_reg();
extern void _kdr_reg();
extern void _nafPR_reg();
extern void _passiv_reg();
extern void _rcadecay_reg();
extern void _rkq_reg();

void modl_reg(){
	//nrn_mswindll_stdio(stdin, stdout, stderr);
    if (!nrn_nobanner_) if (nrnmpi_myid < 1) {
	fprintf(stderr, "Additional mechanisms from files\n");

fprintf(stderr," cal.mod");
fprintf(stderr," expsid.mod");
fprintf(stderr," kcRT03.mod");
fprintf(stderr," kdr.mod");
fprintf(stderr," nafPR.mod");
fprintf(stderr," passiv.mod");
fprintf(stderr," rcadecay.mod");
fprintf(stderr," rkq.mod");
fprintf(stderr, "\n");
    }
_cal_reg();
_expsid_reg();
_kcRT03_reg();
_kdr_reg();
_nafPR_reg();
_passiv_reg();
_rcadecay_reg();
_rkq_reg();
}
