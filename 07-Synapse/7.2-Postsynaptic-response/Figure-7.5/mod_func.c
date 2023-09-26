#include <stdio.h>
#include "hocdec.h"
#define IMPORT extern __declspec(dllimport)
IMPORT int nrnmpi_myid, nrn_nobanner_;

extern void _ampa2s_reg();
extern void _ampa5s_reg();

void modl_reg(){
	//nrn_mswindll_stdio(stdin, stdout, stderr);
    if (!nrn_nobanner_) if (nrnmpi_myid < 1) {
	fprintf(stderr, "Additional mechanisms from files\n");

fprintf(stderr," ampa2s.mod");
fprintf(stderr," ampa5s.mod");
fprintf(stderr, "\n");
    }
_ampa2s_reg();
_ampa5s_reg();
}
