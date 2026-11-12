# ATLAS Rucksack Simulation

Scaffold for studying how ATLAS's moving suspension and hip belt affect load carriage. MATLAB will model the pack hardware. OpenSim will analyze the wearer using recorded motion and applied loads.

| Directory | Purpose |
| --- | --- |
| [matlab/](matlab/) | Suspension model, parameter fitting, and trial preprocessing |
| [bridge/](bridge/) | File contract for passing synchronized loads to OpenSim |
| [opensim/](opensim/) | Human model selection and trial setup |
| [data/examples/](data/examples/) | Header-only input examples |
| [docs/](docs/) | Development and validation checklist |

## Getting started

1. Install MATLAB and [OpenSim](https://opensim.stanford.edu/). Base MATLAB is the intended starting point. Simscape Multibody can be added later if the mechanism needs 3D modeling.
2. Run an official [OpenSim gait tutorial](https://opensimconfluence.atlassian.net/wiki/spaces/OpenSim/pages/53089741) to verify that scaling, inverse kinematics, and inverse dynamics work on your machine.
3. Read the [handoff contract](bridge/README.md) before implementing the exporter. The file-based handoff works with the OpenSim GUI or CLI. [MATLAB scripting](https://opensimconfluence.atlassian.net/wiki/spaces/OpenSim/pages/53089380/Scripting%2Bwith%2BMatlab) is an optional automation step.
4. Follow the [development checklist](docs/validation.md). Record the MATLAB and OpenSim releases used for each trial.

This scaffold contains documentation and input headers. Simulation and export routines are pending.
