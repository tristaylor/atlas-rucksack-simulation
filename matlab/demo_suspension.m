function [comparison, tradeoffs, fit] = demo_suspension(showPlot)
%DEMO_SUSPENSION Run calibration, held-out prediction and a hardware sweep.
% All inputs here are synthetic. Run with false for headless use.
if nargin < 1
    showPlot = true;
end
t = (0:0.01:8).';
mass = 15;
knownStiffness = 1800;
knownDamping = 65;
calibrationAccel = sin(2*pi*1.4*t) + 0.6*sin(2*pi*2.3*t);
calibration = simulate_suspension(t, calibrationAccel, mass, knownStiffness, knownDamping);
fit = fit_suspension(t, calibrationAccel, calibration.travel_m, mass, [1400, 95]);

validationAccel = 1.5*sin(2*pi*2*t);
heldOut = simulate_suspension(t, validationAccel, mass, knownStiffness, knownDamping);
comparison = simulate_suspension(t, validationAccel, mass, fit.stiffness_Npm, fit.damping_Nspm);
fit.held_out_rmse_m = sqrt(mean((comparison.travel_m - heldOut.travel_m).^2));
fit.held_out_force_rmse_N = sqrt(mean((comparison.active_force_N - heldOut.active_force_N).^2));

trials(1) = struct('time_s',t,'frame_accel_mps2',calibrationAccel,'analysis_start_s',2);
trials(2) = struct('time_s',t,'frame_accel_mps2',validationAccel,'analysis_start_s',2);
tradeoffs = sweep_suspension(trials, [10 15 20], [900 1800 2500], [40 65 100], [-0.05 0.05]);
fprintf('Synthetic fit: stiffness %.2f N/m, damping %.2f N s/m (exit flag %d)\n', ...
    fit.stiffness_Npm, fit.damping_Nspm, fit.exit_flag);
fprintf('Held-out travel RMSE %.3g m, force RMSE %.3g N\n', ...
    fit.held_out_rmse_m, fit.held_out_force_rmse_N);

if showPlot
    figure('Name','ATLAS suspension (synthetic input)');
    subplot(2,1,1)
    plot(t, comparison.travel_m*1000);
    ylabel('Payload travel (mm)')
    title('Held-out synthetic trial')
    subplot(2,1,2)
    plot(t, comparison.active_force_N, t, comparison.locked_force_N, '--');
    xlabel('Time (s)')
    ylabel('Payload-on-frame force (N)')
    legend('Suspension active', 'Suspension locked')
end
end
