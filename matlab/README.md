# MATLAB

Implement the suspension and sensor preprocessing here.

| Planned file | Responsibility |
| --- | --- |
| simulate_suspension.m | Predict moving payload travel and vertical payload-on-frame force from frame acceleration, moving mass, stiffness, and damping. Include a locked baseline with matched frame excitation. |
| fit_suspension.m | Estimate stiffness and damping from measured frame motion and relative payload travel. Check predictions against held-out runs. |
| prepare_trial_loads.m | Synchronize gait and interface measurements, transform vectors and points into the OpenSim ground frame, and build the load table. |
| export_opensim_loads.m | Write the load table and its ExternalLoads XML mapping according to the bridge contract. |

Start with a linear mass–spring–damper driven by measured vertical frame acceleration. Define travel relative to loaded equilibrium. Use upward-positive, gravity-removed acceleration and SI units. Keep payload-on-frame force as a hardware output. Wearer loads require interface measurements or a separately calibrated harness model.

See [Foissac et al.](https://pubmed.ncbi.nlm.nih.gov/19062021/) for the backpack characterization study and [MATLAB ode45](https://www.mathworks.com/help/matlab/ref/ode45.html) for a candidate solver.
