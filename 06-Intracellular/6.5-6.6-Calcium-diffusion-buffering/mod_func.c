#include <stdio.h>
#include "hocdec.h"
#define IMPORT extern __declspec(dllimport)
IMPORT int nrnmpi_myid, nrn_nobanner_;

extern void _caJpump_reg();
extern void _cacur_reg();
extern void _cadif4_reg();
extern void _cadifeb4_reg();
extern void _cadifrb4_reg();

void modl_reg(){
	//nrn_mswindll_stdio(stdin, stdout, stderr);
    if (!nrn_nobanner_) if (nrnmpi_myid < 1) {
	fprintf(stderr, "Additional mechanisms from files\n");

fprintf(stderr," caJpump.mod");
fprintf(stderr," cacur.mod");
fprintf(stderr," cadif4.mod");
fprintf(stderr," cadifeb4.mod");
fprintf(stderr," cadifrb4.mod");
fprintf(stderr, "\n");
    }
_caJpump_reg();
_cacur_reg();
_cadif4_reg();
_cadifeb4_reg();
_cadifrb4_reg();
}
