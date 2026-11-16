function summary = sweep_suspension(trials, masses, stiffnesses, dampings, travelLimits)
%SWEEP_SUSPENSION Screen every hardware setting against every motion trial.
% Each trial has time_s, frame_accel_mps2 and analysis_start_s. Optional
% initial_state defaults to rest at loaded equilibrium. Limits are [min max]
% in meters from loaded equilibrium and may be asymmetric.
validateattributes(travelLimits, {'numeric'}, {'vector','real','finite','numel',2,'increasing'});
assert(~isempty(trials) && ~isempty(masses) && ~isempty(stiffnesses) ...
    && ~isempty(dampings), 'Supply trials and candidate hardware settings.');
row = 0;
summary = struct([]);
for trial = 1:numel(trials)
    input = trials(trial);
    validateattributes(input.analysis_start_s, {'numeric'}, {'scalar','real','finite'});
    window = input.time_s >= input.analysis_start_s;
    assert(nnz(window) >= 2, 'The analysis window needs at least two samples.');
    initialState = [0; 0];
    if isfield(input, 'initial_state') && ~isempty(input.initial_state)
        initialState = input.initial_state;
    end
    for mass = masses(:).'
        for stiffness = stiffnesses(:).'
            for damping = dampings(:).'
                result = simulate_suspension(input.time_s, input.frame_accel_mps2, ...
                    mass, stiffness, damping, initialState);
                row = row + 1;
                summary(row).trial = trial;
                summary(row).mass_kg = mass;
                summary(row).stiffness_Npm = stiffness;
                summary(row).damping_Nspm = damping;
                summary(row).natural_frequency_Hz = result.natural_frequency_Hz;
                active = result.active_force_N(window);
                locked = result.locked_force_N(window);
                summary(row).active_peak_N = max(active);
                summary(row).locked_peak_N = max(locked);
                summary(row).active_dynamic_rms_N = std(active, 1);
                summary(row).locked_dynamic_rms_N = std(locked, 1);
                summary(row).peak_reduction_N = summary(row).locked_peak_N ...
                    - summary(row).active_peak_N;
                summary(row).min_sampled_travel_m = min(result.travel_m);
                summary(row).max_sampled_travel_m = max(result.travel_m);
                summary(row).sampled_stroke_margin_m = min( ...
                    summary(row).min_sampled_travel_m - travelLimits(1), ...
                    travelLimits(2) - summary(row).max_sampled_travel_m);
                summary(row).within_sampled_travel_limits = ...
                    summary(row).sampled_stroke_margin_m >= 0;
            end
        end
    end
end
end
