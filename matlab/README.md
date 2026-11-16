# ATLAS suspension and OpenSim export

From the repository root in MATLAB:

```matlab
addpath('matlab')
[comparison, tradeoffs, fit] = demo_suspension();
smoke_test();
```

The demo uses synthetic inputs and nominal hardware values. It fits stiffness and damping on one excitation, checks the fit on a second excitation, and compares 54 trial–hardware combinations. Base MATLAB is sufficient. Use `demo_suspension(false)` to skip the plots.

## Simulate and fit from bench measurements

Use the [frame-motion CSV template](../data/examples/frame_motion.csv). Put the moving assembly at its loaded equilibrium before starting the calibration trial, or supply the measured initial relative displacement and velocity as the last argument to the simulation and fit functions.

```matlab
cal = readtable('data/raw/calibration.csv');
heldOut = readtable('data/raw/validation.csv');

massKg = 15;              % Replace with measured moving assembly mass
initialGuess = [1800 65]; % Replace with bench estimates, in N/m and N s/m
fit = fit_suspension(cal.time_s, cal.frame_accel_mps2, ...
    cal.payload_travel_m, massKg, initialGuess);
assert(fit.exit_flag == 1, 'Review the calibration trial and parameter fit.');

prediction = simulate_suspension(heldOut.time_s, heldOut.frame_accel_mps2, ...
    massKg, fit.stiffness_Npm, fit.damping_Nspm);
travelRMSE = sqrt(mean((prediction.travel_m - heldOut.payload_travel_m).^2));
```

Use increasing time in seconds, vertical inertial frame acceleration in m/s² (upward positive, gravity removed), and relative payload travel in meters (upward positive from loaded equilibrium). Rotate IMU measurements into the laboratory vertical and remove gravity before using this code. The moving mass includes any hardware translating with the payload. The calibration should excite several frequencies. Check that the fit agrees across trials and that different starting guesses give consistent results. On a separate trial, compare predicted travel against measured travel and predicted payload-on-frame force against a bench load cell at the same load path. Fit uncertainty and hardware validity require measured data.

## Model and limits

The implementation adapts the single-degree-of-freedom approach used by [Foissac et al. (2009)](https://pubmed.ncbi.nlm.nih.gov/19062021/) and informed by the frequency comparison in [Li et al. (2016)](https://doi.org/10.1016/j.jbiomech.2016.10.037). The parameter fit and numerical implementation are ATLAS choices.

- The relative motion obeys `m*z_ddot + c*z_dot + k*z = -m*aFrame`. MATLAB uses [expm](https://www.mathworks.com/help/matlab/ref/expm.html) to propagate the linear state over each sample interval with linearly interpolated acceleration. The locked and active modes both receive the same prescribed frame acceleration.
- The active payload-on-frame force is `m*g - k*z - c*z_dot`, and the locked baseline is `m*(g + aFrame)`. These forces are downward positive and describe the moving assembly's contribution to frame loading.
- [fminsearch](https://www.mathworks.com/help/matlab/ref/fminsearch.html) minimizes squared travel error in log-parameter space. The numerical search is limited to four orders of magnitude in either direction from each initial guess. Report the convergence flag and held-out errors as well as the fitted values.
- `sweep_suspension` screens peak force after a user-selected start time and sampled travel over the entire trial. Positive peak reduction means a lower peak in the active case. Supply measured lower and upper stroke limits from the loaded equilibrium for the candidate configurations. Group candidates into separate sweeps if those limits change with their loaded equilibrium. Check travel excursions with a sufficiently dense sample grid. The linear model excludes friction, pack rotation, impacts, and changes to the wearer's gait. Treat settings that exceed travel limits as outside the model's valid operating range.

## Hand off measured loads to OpenSim

```matlab
loads = prepare_trial_loads(streams, time);
export_opensim_loads('runs/trial_01', time, loads);
```

`time` is a uniform trial-clock grid within the synchronized inverse-kinematics interval and coverage of every stream. Supply five calibrated stream structs. Each contains `prefix`, `body`, `time_s`, `clock_offset_s` (trial time minus sensor time), `force_N`, `point_m`, `rotation` (3×3 proper lab-to-ground rotation), and `translation_m` (three components in ground coordinates). `torque_Nm` is optional and is the free moment accompanying the point force. Each force, point, and torque array is N×3. See the [load names and units](../bridge/README.md). For a concrete example of these inputs, see `smoke_test.m`.

Provide forces on the wearer and their lines of action before export. This step assumes clocks already have known offsets and negligible drift over the trial. Align clocks and address drift upstream when needed. Apply anti-alias filtering upstream before downsampling, and resolve foot assignment, contact timing, and force-plate centers of pressure before interpolation. For sensor-local vectors, apply time-varying sensor orientation and calibration upstream. The bridge applies its static transform to express the calibrated trial loads in OpenSim ground coordinates. A point is rotated and translated. Force and free-moment vectors are rotated.

Point the trial's OpenSim inverse-dynamics setup at the generated `ExternalLoads.xml` and keep `trial_loads.mot` beside it. The pack simulation is a parallel hardware analysis. The OpenSim loads come from calibrated forces at the shoulder, belt, and feet. Use the matching wearer model, IK motion, and model body names, then check the force directions and dynamic residuals before interpreting joint moments.

## Choose settings across the operating range

```matlab
trial(1) = struct('time_s', cal.time_s, ...
    'frame_accel_mps2', cal.frame_accel_mps2, 'analysis_start_s', 1);
trial(2) = struct('time_s', heldOut.time_s, ...
    'frame_accel_mps2', heldOut.frame_accel_mps2, 'analysis_start_s', 1);
[ranked, cases] = optimize_suspension(trial, [10 15 20], ...
    [900 1200 1800], [40 65 100], [-0.045 0.05]);
response = suspension_frequency_response(linspace(0, 5, 201), ...
    15, ranked(1).stiffness_Npm, ranked(1).damping_Nspm);
plot(response.frequency_Hz, response.force_transmissibility)
```

The numerical values above are examples. Use measured frame traces that span the intended motions and a candidate grid that includes hardware tolerances. The optimizer puts candidates passing the sampled stroke check for every trial and mass first, then sorts by the largest worst-case reduction in peak payload-on-frame force. A negative reduction means that at least one scenario amplifies the peak. If every candidate fails the stroke check, the ranking does not yield a usable design. Inspect the minimum stroke margin, worst trial, worst mass and per-case dynamic force RMS before choosing a setting. `cases` is indexed by `ranked(i).candidate` and mass index.

A stroke callback `@(mass, stiffness, damping) ...` can return the measured limits relative to that candidate’s loaded equilibrium. Static sag consumes travel and can differ by mass, stiffness and preload. The check uses sampled extrema. Check the sensitivity to sample density and leave clearance for measurement error, fixture tolerances and unmodeled motion.

The frequency response reports the ratio of the active dynamic force amplitude to the locked dynamic force amplitude for a sinusoidal frame acceleration. Ratios greater than one indicate amplification. The complex response describes steady-state behavior around loaded equilibrium. Use time-domain replay for transients and mixed excitations. [Foissac et al.](https://pubmed.ncbi.nlm.nih.gov/19062021/) and [Li et al.](https://pubmed.ncbi.nlm.nih.gov/27825599/) motivate testing across frequencies rather than tuning at one cadence.
