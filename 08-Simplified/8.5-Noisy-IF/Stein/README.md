# Principles of Computational Modelling in Neuroscience (2nd Ed)

## Code examples for Chapter 8: Simplified models of the neuron

### 8.5 Incorporating noise into integrate-and-fire neurons

NEURON simulation of the Stein model of excitatory-inhibitory balance

Load EIbalance.hoc into NEURON. A graphical user interface (GUI) will appear.

Run a simulation with the default parameters (NE=300, NI=150). The top plot window shows the subthreshold membrane potential (with spiking superimposed as short suprathreshold square-wave pulses). The bottom plot is a histogram of interspike intervals (ISIs). You should see results similar to those shown in PCMN Figure 8.15a,c. 

Change the input numbers to NE=18 and NI=0 and rerun the simulation. You should see results similar to those shown in PCMN Figure 8.15b,d.

