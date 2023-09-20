# Principles of Computational Modelling in Neuroscience (2nd Ed)

## Code examples for Chapter 4: Models of active ion channels

### 4.4 Gating particle models of ion channels

#### f-I curves of types I and II

NEURON simulation of the effect of the A-type current on the f-I curve of a simple neuron. The code produces the results shown in PCMN figure 4.11.

Rebuild NEURON in this folder to add the required ion channel models to the simulator.

Load FItypes.hoc into NEURON. A GUI will appear.

These simulations will reproduce the results shown in PCMN Figure 4.11:

Panel (a): In the Parameters window, select "Type I parameters" and click "Set Type I threshold current", then run the simulation to see slow threshold spiking for a type I neuron.

Panel (b): Select "Type II parameters" and click "Set Type II threshold current", then run the simulation to see fast threshold spiking for a type II neuron.

Panel (c): Select  "Type I parameters"  and click Plot in the Grapher window. The plot panel in this window will eventually show the Type I f-I curve (you may need to right-click on this panel and select View->View=Plot to see the curve).

Panel (d): Select "Type II parameters"  and click Plot in the Grapher window to see the Type II f-I curve. 
