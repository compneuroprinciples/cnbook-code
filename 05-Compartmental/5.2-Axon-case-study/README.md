# Principles of Computational Modelling in Neuroscience (2nd Ed)

## Code examples for Chapter 5: Modelling neurons over space and time

### 5.2 Axon case study

NEURON simulations for action potential propagation along straight and branched axons.

#### 1. Axonal conduction speed

Load axon-spatial.hoc into NEURON. A graphical user interface (GUI) will appear.

A small GUI window will allow you to alter the temperature, axon diameter and number of numerical segments (compartments). Clicking "Calculate speed" after completion of a simulation will update the calculation of the conduction speed.

#### 2. Axonal branching

Load axon-branched.hoc into NEURON. A graphical user interface (GUI) will appear.

A small GUI window will allow you to alter the GR ratio. Note that you have to click "Calculate diameter" each time you alter the GR ratio to update the child section diameters. Clicking "Calculate speed" after completion of a simulation will update the calculation of the conduction speed.
