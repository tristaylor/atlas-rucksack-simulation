function loads = prepare_trial_loads(streams, time)
%PREPARE_TRIAL_LOADS Align calibrated load streams on the trial clock.
% time is a uniform grid in seconds within the IK interval. Each stream
% defines time_s, clock_offset_s (trial = sensor + offset), force_N, point_m,
% rotation (lab to OpenSim ground), and translation_m (ground coordinates).
% Optional torque_Nm is a free moment at point_m. Vectors are N-by-3.
validateattributes(time, {'numeric'}, {'vector','real','finite','increasing'});
time = time(:);
assert(numel(time) >= 2, 'Provide at least two export times.');
assert(numel(streams) == 5, 'Supply both feet, both shoulders, and the hip belt.');
loads = struct([]);
for index = 1:numel(streams)
    stream = streams(index);
    validateattributes(stream.time_s, {'numeric'}, {'vector','real','finite','increasing'});
    validateattributes(stream.clock_offset_s, {'numeric'}, {'scalar','real','finite'});
    sampleTime = stream.time_s(:) + stream.clock_offset_s;
    assert(numel(sampleTime) >= 2 && time(1) >= sampleTime(1) ...
        && time(end) <= sampleTime(end), 'Export interval exceeds coverage of %s.', stream.prefix);
    rotation = stream.rotation;
    validateattributes(rotation, {'numeric'}, {'size',[3,3],'real','finite'});
    assert(norm(rotation*rotation.' - eye(3),'fro') < 1e-8 && det(rotation) > 0, ...
        'Rotation must be a proper orthonormal lab-to-ground transform.');
    validateattributes(stream.translation_m, {'numeric'}, {'vector','numel',3,'real','finite'});
    validateattributes(stream.force_N, {'numeric'}, ...
        {'size',[numel(sampleTime),3],'real','finite'});
    validateattributes(stream.point_m, {'numeric'}, ...
        {'size',[numel(sampleTime),3],'real','finite'});
    loads(index).prefix = stream.prefix;
    loads(index).body = stream.body;
    loads(index).force_N = interp1(sampleTime, stream.force_N, time, 'linear') * rotation.';
    loads(index).point_m = interp1(sampleTime, stream.point_m, time, 'linear') ...
        * rotation.' + reshape(stream.translation_m, 1, 3);
    loads(index).torque_Nm = [];
    if isfield(stream, 'torque_Nm') && ~isempty(stream.torque_Nm)
        validateattributes(stream.torque_Nm, {'numeric'}, ...
            {'size',[numel(sampleTime),3],'real','finite'});
        loads(index).torque_Nm = interp1(sampleTime, stream.torque_Nm, time, 'linear') ...
            * rotation.';
    end
end
end
