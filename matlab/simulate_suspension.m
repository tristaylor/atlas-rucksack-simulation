function result = simulate_suspension(time, frameAccel, mass, stiffness, damping, initialState)
%SIMULATE_SUSPENSION Base-excited vertical suspension and locked baseline.
% SI units. Acceleration and relative travel are upward-positive. Travel is
% zero at loaded equilibrium. Payload-on-frame forces are downward-positive.
if nargin < 6
    initialState = [0; 0];
end
validateattributes(time, {'numeric'}, {'vector','real','finite','increasing'});
validateattributes(frameAccel, {'numeric'}, ...
    {'vector','real','finite','numel',numel(time)});
validateattributes(mass, {'numeric'}, {'scalar','real','finite','positive'});
validateattributes(stiffness, {'numeric'}, {'scalar','real','finite','positive'});
validateattributes(damping, {'numeric'}, {'scalar','real','finite','nonnegative'});
validateattributes(initialState, {'numeric'}, {'vector','real','finite','numel',2});
assert(numel(time) >= 2, 'Provide at least two samples.');
time = double(time(:));
frameAccel = double(frameAccel(:));

% State = [relative displacement; relative velocity; base acceleration;
% acceleration slope]. Hold the slope constant within each sample interval.
system = [0, 1, 0, 0; -stiffness/mass, -damping/mass, -1, 0; ...
          0, 0, 0, 1; 0, 0, 0, 0];
steps = diff(time);
transition = expm(system * steps(1));
uniform = max(abs(steps - steps(1))) <= 1e-10 * steps(1);
state = zeros(2, numel(time));
state(:,1) = initialState(:);
for sample = 1:numel(steps)
    if ~uniform
        transition = expm(system * steps(sample));
    end
    slope = (frameAccel(sample+1) - frameAccel(sample)) / steps(sample);
    state(:,sample+1) = transition(1:2,:) * ...
        [state(:,sample); frameAccel(sample); slope];
end

gravity = 9.80665;
result.time_s = time;
result.travel_m = state(1,:).';
result.relative_velocity_mps = state(2,:).';
result.active_force_N = mass*gravity - stiffness*result.travel_m ...
    - damping*result.relative_velocity_mps;
result.locked_force_N = mass*(gravity + frameAccel);
result.natural_frequency_Hz = sqrt(stiffness/mass)/(2*pi);
result.damping_ratio = damping/(2*sqrt(stiffness*mass));
end
