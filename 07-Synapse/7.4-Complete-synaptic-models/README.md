# Principles of Computational Modelling in Neuroscience (2nd Ed)

## Code examples for Chapter 7: The synapse

### 7.4 Complete synaptic models

NEURON simulations of the postsynaptic response to stochastic vesicle release from single and multiple presynaptic release sites.

Rebuild NEURON in this folder to add the required synapse models to the simulator.

### 1. Synaptic responses in a stochastic multiple-release-site model

Load synstochmulti.hoc into NEURON. A graphical user interface (GUI) will appear.

The "Synapse parameters" GUI window will allow you to alter presynaptic vesicle release and recycling characteristics. Click "Update synapse" each time you change a parameter value to apply your changes.

Running a simulation with the default parameters will result in the plot window showing the postsynaptic EPSCs due to a train of presynaptic stimuli delivered at 100 Hz. This should be similar to the examples shown in PCMN Figure 7.12.

### 2. Synaptic responses in a stochastic single-release-site model

Load synstochsingle.hoc into NEURON. A graphical user interface (GUI) will appear.

The "Synapse parameters" GUI window will allow you to alter presynaptic vesicle release and recycling characteristics. Click "Update synapse" each time you change a parameter value to apply your changes.

Running a simulation with the default parameters will result in tthe top plot window showing the postsynaptic mEPSCs due to a train of presynaptic stimuli delivered at 100 Hz. You will likely see just a single mEPSC during the 100 ms running time, occuring at different times for each run of the simulation. The middle plot shows the vesicle release probability, which increases on each stimulation due to facilitation. The bottom plot shows the vesicle occupancy of the release site: this is initially 1, as there is a releasable vesicle at the release site, and goes to 0 if the vesicle releases on a stimulus, resulting in a mEPSC. The site could refill from the reserve pool supply following a release, but this is unlikely to happen in this short running time.