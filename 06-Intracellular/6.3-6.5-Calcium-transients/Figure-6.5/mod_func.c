#include <stdio.h>
#include "hocdec.h"
#define IMPORT extern __declspec(dllimport)
IMPORT int nrnmpi_myid, nrn_nobanner_;

extern void _ca2pool_reg();
extern void _caJpump_reg();
extern void _cacum_reg();
extern void _cacur_reg();
extern void _cadif1_reg();

void modl_reg(){
	//nrn_mswindll_stdio(stdin, stdout, stderr);
    if (!nrn_nobanner_) if (nrnmpi_myid < 1) {
	fprintf(stderr, "Additional mechanisms from files\n");

fprintf(stderr," ca2pool.mod");
fprintf(stderr," caJpump.mod");
fprintf(stderr," cacum.mod");
fprintf(stderr," cacur.mod");
fprintf(stderr," cadif1.mod");
fprintf(stderr, "\n");
    }
_ca2pool_reg();
_caJpump_reg();
_cacum_reg();
_cacur_reg();
_cadif1_reg();
}
