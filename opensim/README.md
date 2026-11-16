# OpenSim inverse dynamics

This workflow takes participant-scaled OpenSim models, inverse-kinematics motion, and the MATLAB exporter’s synchronized loads. It calculates net generalized forces and moments and compares one locked trial with its matched active trial.

## Run the synthetic check

Use a Python environment with the OpenSim 4.6 bindings. From the repository root:

```sh
python -m venv .venv
source .venv/bin/activate
python -m pip install -r opensim/requirements.txt
python opensim/demo_inverse_dynamics.py --out runs/demo
```

Use a new output directory on each run. If pip has no wheel for your platform or Python version, install the [OpenSim Python bindings](https://opensimconfluence.atlassian.net/wiki/spaces/OpenSim/pages/53085346/Scripting+in+Python) in a supported environment. The demo uses an articulated fixture and known forces at fixed moment arms. It checks that the vertical residual is near zero and recovers the expected knee and lumbar moment differences. Its inputs are synthetic.

## Run an ATLAS trial

1. Scale a wearer-only model and run OpenSim inverse kinematics on the trial marker data. Save the coordinate motion as a .mot file with its `inDegrees` header and coordinate names from the scaled model. Record the model revision and inspect IK marker fit. The [gait tutorial](https://opensimconfluence.atlassian.net/wiki/spaces/OpenSim/pages/53084195/Part+I+Computing+Joint+Moments+with+OpenSim) walks through both steps.
2. Prepare the force plate and pack interface streams on the same trial clock, then use the [MATLAB exporter](../matlab/) to write each trial’s `ExternalLoads.xml` and `trial_loads.mot`. Choose the target body names from the scaled model. Use forces on the wearer, consistent ground-frame axes, and calibrated forces and application points for both feet, both shoulders and the hip belt.
3. Run inverse dynamics for each condition with the same participant model. The example assumes motion and loads overlap from 0.5 to 3.5 seconds. Choose the actual analysis window separately for each trial using matched walking conditions and gait segments. Omit `--window` when the load table spans the entire coordinate motion.

```sh
python opensim/run_inverse_dynamics.py \
  --model data/processed/p01_scaled.osim \
  --motion data/processed/p01_locked_ik.mot \
  --loads data/processed/p01_locked/ExternalLoads.xml \
  --window 0.5 3.5 --out runs/p01_locked

python opensim/run_inverse_dynamics.py \
  --model data/processed/p01_scaled.osim \
  --motion data/processed/p01_active_ik.mot \
  --loads data/processed/p01_active/ExternalLoads.xml \
  --window 0.5 3.5 --out runs/p01_active

python opensim/compare_trials.py \
  --locked runs/p01_locked --active runs/p01_active \
  --out runs/p01_comparison.csv
```

The CLI checks the load mapping, time coverage, finite samples and model coordinate names before launching OpenSim. Only coordinates explicitly locked in the model may be omitted. The tool uses the motion as supplied by default. If you filter coordinates in OpenSim, pass the same justified `--cutoff-hz` in both trials and document the upstream force processing. The runner excludes muscles and other actuators using OpenSim’s actuator keyword. Other forces are excluded by name or group. If OpenSim cannot address an enabled force through its exclusion settings, disable it explicitly in the analysis model. Use a wearer-only model with no pre-applied pack forces or foot contact forces implemented outside the ForceSet.

## Read the results

| Output | Use |
| --- | --- |
| `ID_Setup.xml` | Reopen the exact inverse-dynamics setup in OpenSim. It references local absolute paths. |
| `inverse_dynamics.sto` | Time series of generalized forces and moments along the model’s coordinates. Check coordinate signs and units against the model. |
| `summary.csv` | Signed extrema, RMS and the signed sample with the greatest magnitude. Ground-connected coordinates are marked for residual inspection. |
| `analysis.json` | Input paths, model checksum, OpenSim release, time window, coordinate filter and force exclusions. |
| `comparison.csv` | Active-minus-locked changes in the summary statistics. The tool checks the model checksum, OpenSim release and filter settings. |

Inspect residual waveforms, load locations and model fit before interpreting moment differences. The comparison summarizes the selected time windows. Match speed and payload, choose comparable strides and use the original time series for gait-event alignment or waveform analysis. Measure shoulder strap force directly and use inverse dynamics to evaluate how knee and lumbar moment demands change. Estimates of muscle force, joint contact force and injury risk require additional analysis and validation.

[OpenSim inverse dynamics documentation](https://opensimconfluence.atlassian.net/wiki/spaces/OpenSim/pages/53090053/How+to+Use+the+Inverse+Dynamics+Tool) · [OpenSim inverse dynamics implementation](https://github.com/opensim-org/opensim-core/blob/main/OpenSim/Tools/InverseDynamicsTool.cpp)

## Compare matched gait phases

Create one heel-strike CSV per condition using reviewed events on the same clock as the motion and loads. Include only consecutive, quality-controlled strides. Use the same event detection and review criteria in both trials.

```csv
side,time_s
left,0.61
right,1.14
left,1.69
right,2.21
left,2.76
```

```sh
python opensim/compare_gait_cycles.py \
  --locked runs/p01_locked --active runs/p01_active \
  --locked-events data/processed/p01_locked_strikes.csv \
  --active-events data/processed/p01_active_strikes.csv \
  --side left --columns knee_angle_l_moment lumbar_extension_moment \
  --out runs/p01_gait_left.csv
```

Each stride maps consecutive same-side heel strikes to 0–100% gait cycle. The output contains equal-stride-weighted means, active-minus-locked differences, within-trial stride SD when at least two strides are available, and stride count and mean duration for each condition. These are descriptive within-trial summaries. Check signs and report model-coordinate units. Left and right knee moments should be normalized to strikes for their respective sides. [Gait cycle reference](https://doi.org/10.1002/ksa.70067).

## Numerical and experimental validation

The synthetic demo checks an analytical force balance and moment differences. A second check used the [OpenSim Gait2392 reference walking trial](https://github.com/opensim-org/opensim-models/tree/d9b05d470b1a481c222372c85b75772faf8f7792/Pipelines/Gait2392_Simbody), with the same scaled model and 6 Hz coordinate filter in both runs. The ATLAS-format loads used the public foot forces, application points and free moments with a reconstructed uniform time grid and zero pack loads. Compared with the native OpenSim setup, peak absolute moment differences across the 67 matched frames were 0.0241 N·m for the left knee, 0.0301 N·m for the right knee and below output precision for lumbar extension. This checks the file handoff and solver against a walking dataset. ATLAS prototype performance requires paired measurements.

When filtering is enabled, OpenSim can resample motion and shift the output endpoints. Use the actual output time range in `analysis.json` when selecting heel strikes. Check pack and ground load alignment and root residuals before interpreting joint curves. Repeat the analysis with defensible filtering choices and report whether the conclusions change. The 6 Hz benchmark setting reproduces that public example. [OpenSim inverse dynamics best practices](https://opensimconfluence.atlassian.net/wiki/spaces/OpenSim/pages/53090497/_Inverse%2BDynamics%2BBest%2BPractices) · [Hicks et al.](https://pubmed.ncbi.nlm.nih.gov/25474098/) · [Kristianslund et al.](https://pubmed.ncbi.nlm.nih.gov/22227316/).

To repeat the public benchmark, check out the linked upstream revision and run:

```sh
python opensim/benchmark_gait2392.py \
  --upstream /path/to/opensim-models/Pipelines/Gait2392_Simbody \
  --out runs/gait2392_benchmark
```
