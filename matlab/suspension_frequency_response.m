function response = suspension_frequency_response(frequency, mass, stiffness, damping)
%SUSPENSION_FREQUENCY_RESPONSE Linear steady-state response to base acceleration.
% Frequency in Hz; scalar hardware settings in SI units. Travel and force
% are complex gains per unit upward base acceleration. Force is downward
% positive and excludes static payload weight.
validateattributes(frequency, {'numeric'}, {'vector','real','finite','nonnegative'});
validateattributes(mass, {'numeric'}, {'scalar','real','finite','positive'});
validateattributes(stiffness, {'numeric'}, {'scalar','real','finite','positive'});
validateattributes(damping, {'numeric'}, {'scalar','real','finite','nonnegative'});
omega = 2*pi*double(frequency(:));
denominator = stiffness - mass*omega.^2 + 1i*damping*omega;
if damping == 0 && any(abs(real(denominator)) <= 8*eps(stiffness))
    error('No finite steady-state response at undamped resonance.');
end
response.frequency_Hz = frequency(:);
response.travel_per_accel_s2 = -mass ./ denominator;
response.dynamic_force_per_accel_kg = -(stiffness + 1i*damping*omega) ...
    .* response.travel_per_accel_s2;
response.force_transmissibility = abs(response.dynamic_force_per_accel_kg) / mass;
end
