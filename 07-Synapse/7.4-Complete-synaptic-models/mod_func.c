#include <stdio.h>
#include "hocdec.h"
#define IMPORT extern __declspec(dllimport)
IMPORT int nrnmpi_myid, nrn_nobanner_;

extern void _ANsyn_reg();
extern void _DEsyn_reg();
extern void _ampa2s_reg();
extern void _ampa5s_reg();
extern void _ampade_reg();
extern void _gsyn_reg();
extern void _stochstim_reg();

void modl_reg(){
	//nrn_mswindll_stdio(stdin, stdout, stderr);
    if (!nrn_nobanner_) if (nrnmpi_myid < 1) {
	fprintf(stderr, "Additional mechanisms from files\n");

fprintf(stderr," ANsyn.mod");
fprintf(stderr," DEsyn.mod");
fprintf(stderr," ampa2s.mod");
fprintf(stderr," ampa5s.mod");
fprintf(stderr," ampade.mod");
fprintf(stderr," gsyn.mod");
fprintf(stderr," stochstim.mod");
fprintf(stderr, "\n");
    }
_ANsyn_reg();
_DEsyn_reg();
_ampa2s_reg();
_ampa5s_reg();
_ampade_reg();
_gsyn_reg();
_stochstim_reg();
}
