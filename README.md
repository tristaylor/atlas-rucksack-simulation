# ATLAS Rucksack Simulation

Study how ATLAS's moving suspension and hip belt affect load carriage. The MATLAB implementation models the pack hardware and exports calibrated forces for OpenSim to analyze alongside recorded wearer motion.

| Directory | Purpose |
| --- | --- |
| [matlab/](matlab/) | Suspension model, parameter fitting, and trial preprocessing |
| [bridge/](bridge/) | File contract for passing synchronized loads to OpenSim |
| [opensim/](opensim/) | Inverse-dynamics runner, synthetic validation, and paired trial comparison |
| [data/examples/](data/examples/) | Header-only input examples |
| [docs/](docs/) | Development and validation checklist |

## Getting started

1. Install MATLAB and [OpenSim](https://opensim.stanford.edu/). Base MATLAB is the intended starting point. Simscape Multibody can be added later if the mechanism needs 3D modeling.
2. Run `addpath('matlab'); demo_suspension(); smoke_test();` from the repository root. Run the [OpenSim synthetic demo](opensim/README.md#run-the-synthetic-check), then follow an official [OpenSim gait tutorial](https://opensimconfluence.atlassian.net/wiki/spaces/OpenSim/pages/53089741) to verify that scaling, inverse kinematics, and inverse dynamics work on your machine.
3. Read the [handoff contract](bridge/README.md) before exporting a walking trial. The [OpenSim runner](opensim/README.md#run-an-atlas-trial) consumes that export with the matching model and coordinate motion. [MATLAB scripting](https://opensimconfluence.atlassian.net/wiki/spaces/OpenSim/pages/53089380/Scripting%2Bwith%2BMatlab) is an optional automation step.
4. Follow the [development checklist](docs/validation.md). Record the MATLAB and OpenSim releases used for each trial.

See the [MATLAB guide](matlab/README.md) for calibration, parameter sweeps, and the trial export. The demo uses synthetic data. Hardware calibration and wearer analysis require measured trials.
