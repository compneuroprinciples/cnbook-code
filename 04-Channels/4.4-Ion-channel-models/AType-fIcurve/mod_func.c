#include <stdio.h>
#include "hocdec.h"
#define IMPORT extern __declspec(dllimport)
IMPORT int nrnmpi_myid, nrn_nobanner_;

extern void _csk_reg();
extern void _cska_reg();
extern void _csl_reg();
extern void _csna_reg();

void modl_reg(){
	//nrn_mswindll_stdio(stdin, stdout, stderr);
    if (!nrn_nobanner_) if (nrnmpi_myid < 1) {
	fprintf(stderr, "Additional mechanisms from files\n");

fprintf(stderr," csk.mod");
fprintf(stderr," cska.mod");
fprintf(stderr," csl.mod");
fprintf(stderr," csna.mod");
fprintf(stderr, "\n");
    }
_csk_reg();
_cska_reg();
_csl_reg();
_csna_reg();
}
