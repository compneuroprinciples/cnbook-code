# Principles of Computational Modelling in Neuroscience (2nd Ed)

## Code examples for Chapter 11: Plasticity

### 11.2.1 Spike-timing-dependent plasticity

#### NEURON simulation of the effect of STDP on spike timing.

Rebuild NEURON in this folder to add the required STDP model to the simulator.

Load STDPtiming.hoc into NEURON. A graphical user interface (GUI) will appear. The small "STDP parameters" window allows specifying some features of the STDP rule used in a simulation.

Run a simulation with the default STDP parameters that specify an initial input weight of 0.012, which is scaled by 10 (Max weight multiplier) to give a maximum allowed weight of 0.12, and divided by 10 (Min weight divisor) to give a minimum allowed weight of 0.0012. The LTP scaling (Potentiation rate) is 0.05, while the LTD scaling (Depression rate) is 0, so only LTP is applied during the simulation.

The voltage plot will show that the timing of the cell's action potential, relative to the inputs, changes over the course of the simulation.

Press the "Plot weights" button in the "STDP parameters" window to plot the weight values (y-axis) versus input number (x-axis) in the other, smaller Graph window.