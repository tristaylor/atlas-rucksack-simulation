# Trial setups

Generate or save the following files per trial once the model and dataset are available.

| Setup | References |
| --- | --- |
| Scale setup | Generic model, static marker trial, participant measurements, output scaled model |
| Inverse kinematics setup | Scaled model, walking marker trajectories, marker weights, trial time range, output motion |
| ExternalLoads.xml | Exported load table, body names, force and point frames, and column prefixes |
| Inverse dynamics setup | Scaled model, inverse kinematics motion, ExternalLoads.xml, matching time range, output moments |

Check paths by opening one trial in the OpenSim GUI before adding batch automation.
