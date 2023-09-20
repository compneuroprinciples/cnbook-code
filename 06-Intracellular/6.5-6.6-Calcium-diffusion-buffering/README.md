# Principles of Computational Modelling in Neuroscience (2nd Ed)

## Code examples for Chapter 6: Intracellular mechanisms

### 6.5 Calcium buffering and 6.6 Calcium diffusion

NEURON simulations of (un)buffered radial and longitudinal diffusion in dendrites.

Rebuild NEURON in this folder to add the required diffusion models to the simulator.

### 1. Calcium intracellular buffered radial diffusion

Load caradbuff.hoc into NEURON. A graphical user interface (GUI) will appear.

A "Buffer parameters" GUI window will allow you to alter diffusion rates and buffer characteristics. Click "Update buffer" each time you change a parameter value to apply your changes.

Running a simulation with the default parameters, which specify a slow buffer, produces results as in PCMN Figure 6.12. The two plot windows show the calcium concentration over time in the submembrane shell (top) and the dendrite core shell (bottom). The full kinetic model is shown by the black traces, with the blue trace showing the Excess Buffer Approximation (EBA) and the red trace showing the Rapid Buffer Approximation (RBA). 

### 2. Calcium intracellular longitudinal diffusion

Load calong.hoc into NEURON. A graphical user interface (GUI) will appear.

Running a simulation with the default parameters produces results as in PCMN Figure 6.11b. The three plot windows show the membrane voltage (top) the calcium concentration over time in the submembrane shell (middle) and the dendrite core shell (bottom) at different points along the dendrite, along with the equivalent quantities in a single compartment model (labelled "spine").