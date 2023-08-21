# Continuum model of tubulin-driven neurite elongation

This model investigates the elongation over time of a single
developing neurite (axon or dendrite).  The dynamics of outgrowth,
determined by numerical integration of the model, are explored in
the paper (to be referred to as GLM):

Bruce P. Graham, Karen Lauchlan and Douglas R. McLean, "Dynamics
of outgrowth in a continuum model of neurite elongation", Journal
of Computational Neuroscience, to appear.

Our neurite growth model describes the elongation of a single,
unbranched neurite in terms of the rate of extension of the
microtubule cytoskeleton. The cytoskeleton is not explicitly
modelled, but its construction is assumed to depend on the
available free tubulin at the growing neurite tip. A site of
tubulin production in the cell body (soma) results in a flux of
tubulin across the soma/neurite interface from where tubulin is
transported along the neurite by active transport and diffusion.
A constant, slow active transport rate is assumed. At the
distal neurite tip tubulin is consumed by microtubule assembly,
which represents the average assembly of all available
microtubules. If assembly exceeds disassembly, then the
population of microtubules increases in length, on average.  
To incorporate the effect of
somatic tubulin concentration on tubulin synthesis into the model,
the flux of tubulin across the soma/neurite interface may be
continuously autoregulated by the tubulin concentration there.
Finally, tubulin is assumed everywhere to degrade with a fixed
time constant. For simplicity we assume that this single
``degradation'' process encompasses degradation of the tubulin
dimers, a constant consumption of tubulin by microtubule assembly
along the length of the neurite, and also allows for local
synthesis of new tubulin in the neurite leading to an apparent
increase in the tubulin half-life. Microtubule
assembly proximal to the neurite tip is assumed not to contribute
to neurite elongation.

Full mathematical details of the model and the algorithm for
its numerical solution using a finite difference scheme are
given in GLM. A novelty here is that the neurite length changes
dynamically, so stretching of the spatial discretization must
be incorporated.

Steady-state analysis of the model can be found in:
McLean & Graham, Proc. Roy. Soc. Lond. A. 460:2437-2456, 2004.
McLean, Lauchlan & Graham, WSEAS Trans. Biol. and Biomed.
2:98-105, 2005.

The numerical code was developed using Matlab version 7.

