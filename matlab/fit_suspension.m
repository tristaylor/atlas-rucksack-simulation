function fit = fit_suspension(time, frameAccel, travel, mass, guess, initialState)
%FIT_SUSPENSION Fit positive stiffness and damping to measured travel.
% guess = [stiffness (N/m), damping (N s/m)]. Provide measured initial
% displacement and velocity, or omit them for a trial started from rest.
if nargin < 6
    initialState = [0; 0];
end
validateattributes(travel, {'numeric'}, ...
    {'vector','real','finite','numel',numel(time)});
validateattributes(guess, {'numeric'}, {'vector','real','finite','positive','numel',2});
assert(numel(time) >= 4, 'Provide at least four calibration samples.');
travel = double(travel(:));
assert(max(abs(travel)) > 0, 'Calibration needs measurable travel.');
simulate_suspension(time, frameAccel, mass, guess(1), guess(2), initialState);

options = optimset('Display','off','TolX',1e-7,'TolFun',1e-10);
[logScale, ~, exitFlag] = fminsearch(@loss, [0, 0], options);
params = guess(:).' .* exp(logScale);
fit.stiffness_Npm = params(1);
fit.damping_Nspm = params(2);
fit.exit_flag = exitFlag;
fit.prediction = simulate_suspension(time, frameAccel, mass, ...
    params(1), params(2), initialState);
fit.rmse_m = sqrt(mean((fit.prediction.travel_m - travel).^2));

    function value = loss(logScale)
        % Keep the numerical search within four orders of the supplied guess.
        if any(abs(logScale) > log(1e4))
            value = Inf;
            return
        end
        candidate = guess(:).' .* exp(logScale);
        prediction = simulate_suspension(time, frameAccel, mass, ...
            candidate(1), candidate(2), initialState);
        value = mean(((prediction.travel_m - travel)/max(abs(travel))).^2);
        if ~isfinite(value)
            value = Inf;
        end
    end
end
