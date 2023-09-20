# Principles of Computational Modelling in Neuroscience (2nd Ed)

## Code examples for Chapter 9: Networks of neurons

### 9.3 Networks of integrate-and-fire neurons

NEURON simulation of the Amit and Brunel network model of excitatory-inhibitory balance.

Rebuild NEURON in this folder to add the required neuron models to the simulator.

Load EIbalnet.hoc into NEURON. A graphical user interface (GUI) will appear.

The simulation is set up to run a network of half the size (scale=0.5) of the network used to produce the results in PCMN Figure 9.8. It may still take several minutes to set up and run on a decent PC. Click Run in the small control window to run this half-scale model.

While the simulation is running, voltage plots from several example neurons will appear (excitatory in black, inhibitory in blue). 

Once the simulation has finished, the other output windows will be populated with various statistical measures of the neuronal firing rates. NOTE: you might need to resize Graph[4] slightly to see the full spike raster plot (as illustrated in PCMN Fig. 9.8e). These firing rates should be largely irregular in this configuration.
