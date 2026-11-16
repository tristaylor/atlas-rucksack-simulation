function [ranking, cases] = optimize_suspension(trials, masses, stiffnesses, dampings, limits)
%OPTIMIZE_SUSPENSION Rank candidate hardware by worst-case peak reduction.
% Trials have the same fields as sweep_suspension. Limits are a common
% [min max] stroke in meters, or a function @(mass,k,c) returning that pair
% relative to the loaded equilibrium for each candidate.
validateattributes(masses, {'numeric'}, {'vector','real','finite','positive','nonempty'});
validateattributes(stiffnesses, {'numeric'}, {'vector','real','finite','positive','nonempty'});
validateattributes(dampings, {'numeric'}, {'vector','real','finite','nonnegative','nonempty'});
assert(~isempty(trials), 'Supply at least one representative excitation trial.');
travelByCandidate = isa(limits, 'function_handle');
candidate = 0;
ranking = struct([]);
cases = cell(numel(stiffnesses)*numel(dampings), numel(masses));
for stiffness = stiffnesses(:).'
    for damping = dampings(:).'
        candidate = candidate + 1;
        screening = struct([]);
        for massIndex = 1:numel(masses)
            mass = masses(massIndex);
            stroke = limits;
            if travelByCandidate
                stroke = limits(mass, stiffness, damping);
            end
            validateattributes(stroke, {'numeric'}, ...
                {'vector','real','finite','numel',2,'increasing'});
            assert(stroke(1) < 0 && stroke(2) > 0, 'Stroke must span loaded equilibrium.');
            run = sweep_suspension(trials, mass, stiffness, damping, stroke);
            cases{candidate, massIndex} = run;
            screening = [screening, run]; %#ok<AGROW>
        end
        [worstReduction, worstIndex] = min([screening.peak_reduction_N]);
        ranking(candidate) = struct('candidate',candidate,'stiffness_Npm',stiffness, ...
            'damping_Nspm',damping,'all_within_sampled_stroke', ...
            all([screening.within_sampled_travel_limits]), ...
            'worst_peak_reduction_N',worstReduction, ...
            'median_peak_reduction_N',median([screening.peak_reduction_N]), ...
            'minimum_sampled_stroke_margin_m',min([screening.sampled_stroke_margin_m]), ...
            'worst_trial',screening(worstIndex).trial, ...
            'worst_mass_kg',screening(worstIndex).mass_kg);
    end
end
priority = [-double([ranking.all_within_sampled_stroke].'), ...
    -[ranking.worst_peak_reduction_N].', -[ranking.median_peak_reduction_N].'];
[~, order] = sortrows(priority, [1 2 3]);
ranking = ranking(order);
end
