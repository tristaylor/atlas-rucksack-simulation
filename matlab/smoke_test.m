function smoke_test(folder)
%SMOKE_TEST Exercise the hardware model and file handoff without real data.
% Optionally specify a folder for the synthetic OpenSim load files.
if nargin < 1
    folder = tempname;
    mkdir(folder);
    cleanup = onCleanup(@() rmdir(folder, 's')); %#ok<NASGU>
end

% Exact solution for an undamped system under a constant base acceleration.
% An irregular grid also exercises the variable-step propagation.
t = [0; 0.03; 0.17; 0.8; 1.32];
result = simulate_suspension(t, 2*ones(size(t)), 10, 1000, 0);
assert(max(abs(result.travel_m + 0.02*(1-cos(10*t)))) < 1e-10);
assert(max(abs(result.active_force_N - 10*(9.80665 + 2*(1-cos(10*t))))) < 1e-8);
assert(all(abs(result.locked_force_N - 10*(9.80665+2)) < 1e-10));

[~, tradeoffs, fit] = demo_suspension(false);
assert(fit.exit_flag == 1 && abs(fit.stiffness_Npm-1800) < 1 ...
    && abs(fit.damping_Nspm-65) < 0.1);
assert(fit.held_out_rmse_m < 1e-5 && fit.held_out_force_rmse_N < 0.05);
assert(numel(tradeoffs) == 54 && any([tradeoffs.within_sampled_travel_limits]) ...
    && any(~[tradeoffs.within_sampled_travel_limits]));
assert(any([tradeoffs.peak_reduction_N] > 0) && any([tradeoffs.peak_reduction_N] < 0));

prefixes = {'left_grf','right_grf','left_shoulder','right_shoulder','hip_belt'};
bodies = {'calcn_l','calcn_r','torso','torso','pelvis'};
t = (0:0.125:1).';
for index = 1:5
    streams(index) = struct('prefix',prefixes{index},'body',bodies{index}, ...
        'time_s',[-0.25; 0.75],'clock_offset_s',0.25, ...
        'force_N',[0 0 0; index 0 0],'point_m',[2 0 0; 2 0 0], ...
        'rotation',[0 -1 0; 1 0 0; 0 0 1],'translation_m',[1 2 3], ...
        'torque_Nm',[]);
end
streams(1).torque_Nm = [0 0 1; 0 0 1];
loads = prepare_trial_loads(streams, t);
assert(isequal(loads(1).point_m(5,:), [1 4 3]));
assert(isequal(loads(5).force_N(5,:), [0 2.5 0]));
assert(isequal(loads(1).torque_Nm(5,:), [0 0 1]));
export_opensim_loads(folder, t, loads);
motion = fileread(fullfile(folder, 'trial_loads.mot'));
assert(~isempty(strfind(motion, 'nColumns=34'))); %#ok<STREMP>
assert(~isempty(strfind(motion, 'left_grf_torque_z'))); %#ok<STREMP>
mapping = fileread(fullfile(folder, 'ExternalLoads.xml'));
assert(~isempty(strfind(mapping, '<torque_identifier>left_grf_torque_</torque_identifier>'))); %#ok<STREMP>
assert(~isempty(strfind(mapping, '<applied_to_body>pelvis</applied_to_body>'))); %#ok<STREMP>
fprintf('ATLAS synthetic smoke test passed.\n');
end
