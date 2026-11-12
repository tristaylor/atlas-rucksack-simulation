# MATLAB-to-OpenSim handoff

Use one synchronized load table and one ExternalLoads.xml per trial. MATLAB prepares the files, and OpenSim applies the loads while running inverse dynamics.

| Data group | Example prefix | Applied to |
| --- | --- | --- |
| Left and right ground reaction | left_grf, right_grf | Corresponding foot segment |
| Left and right shoulder interface | left_shoulder, right_shoulder | Torso |
| Hip-belt interface | hip_belt | Pelvis |

Use the actual target body names in the selected scaled model. Each load group uses three force columns such as left_shoulder_force_x/y/z and three point columns such as left_shoulder_point_x/y/z. Include torque columns and an XML torque identifier when the trial provides a free moment. XML identifiers must match the common column prefixes, such as left_shoulder_force_. Express forces and points in the OpenSim ground frame for the first exporter and document the lab-to-model transform. Use forces on the wearer in newtons, points in meters, moments in newton-meters, and trial time in seconds.

Export an OpenSim .mot file with a header, unique tab-delimited column labels, and uniformly spaced times that overlap the inverse kinematics motion. Keep raw data separate from synchronized exports. Record the time offset or synchronization event, the model revision, and whether each interface load was measured or estimated.

A suspension prediction gives the vertical force of the moving payload on the pack frame. Estimating separate torso and pelvis loads requires interface measurements or a calibrated model of the harness. Inline strap tension needs a line of action. Belt pressure alone does not resolve shear.

[OpenSim motion file format](https://opensimconfluence.atlassian.net/wiki/spaces/OpenSim/pages/53089415) · [ExternalForce implementation](https://github.com/opensim-org/opensim-core/blob/main/OpenSim/Simulation/Model/ExternalForce.cpp)
