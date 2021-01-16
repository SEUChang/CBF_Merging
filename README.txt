A SimEvents Model for Real-time Traffic Merging

The real-time controllers for all CAVs are based on the Control Barrier Function (CBF) approach.


***************************************************************************************************
Instructions:

* Open 'SingleMerging.slx'
* If 'CAV' and 'INFO' entites are not preloaded, please run 'init_func.m' in MATLAB command line to load the CAV and INFO entities.
* Run the simulation

***************************************************************************************************
Note:

* In this version, the main and merging lanes use different CAVs generator, so two CAVs may come from these two different lanes at the same time (discretized time instant).
* If two CAVs come from two different lanes at the same time, there will be some problems because of the FIFO queue. When this happens, perturbation is enforced on the next arrival CAV (its position changes to -0.1 and the distance between i and i-1 changes to 0.1 instead of 0).
* MATLAB version: R2017a.

