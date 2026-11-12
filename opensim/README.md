# OpenSim

Begin with the stock gait tutorial. Then scale a human model to the participant, use the trial's marker trajectories for inverse kinematics, and run inverse dynamics with the motion and matched external loads. Use the same model and processing choices for paired locked and moving-suspension trials.

- [models/](models/) records the upstream model and scaling inputs.
- [setups/](setups/) holds the specifications for trial-specific tool configurations.
- Write generated trial results under the ignored runs/ or results/ directories.

Compare net knee extension and lumbar flexion moments, checking joint coordinate names and signs against the selected model. Shoulder strap force is a direct sensor outcome.
