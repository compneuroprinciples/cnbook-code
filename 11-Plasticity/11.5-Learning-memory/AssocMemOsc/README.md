# Principles of Computational Modelling in Neuroscience (2nd Ed)

## Code examples for Chapter 11: Plasticity

### 11.5 Network models of learning and memory

#### NEURON simulation of associative memory in a network of spiking neurons.

Rebuild NEURON in this AssocMemOsc folder to add the required neuron model to the simulator.

Load AssocMem.hoc into NEURON. A graphical user interface (GUI) will appear. The small "Control" window allows specifying the strengths and time delays of the excitatory and inhibitory connections in the network.

Run simulations with the default parameters. You should see results similar to those in PCMN Figure 11.9.

The voltage graphs show the spiking activity of three example cells: the top plot shows the activity in a pattern cue cell, the middle plot shows recalled activity in a cell that is a part of the stored (cue) pattern and the bottom plot shows activity in a cell that is not part of the stored pattern.

You can click Spike Plot in the Control window to see a raster plot of the entire network activity.
