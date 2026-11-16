function export_opensim_loads(folder, time, loads)
%EXPORT_OPENSIM_LOADS Write trial_loads.mot and ExternalLoads.xml together.
% Call with one output folder per trial and target body names from the model.
validateattributes(time, {'numeric'}, {'vector','real','finite','increasing'});
time = time(:);
steps = diff(time);
assert(~isempty(steps) && max(abs(steps - mean(steps))) <= 1e-6*mean(steps), ...
    'Motion export requires at least two uniformly spaced times.');
required = {'left_grf','right_grf','left_shoulder','right_shoulder','hip_belt'};
prefixes = arrayfun(@(load) char(load.prefix), loads, 'UniformOutput', false);
assert(isequal(sort(prefixes(:)), sort(required(:))), ...
    'Provide exactly one load for each of the five documented prefixes.');
columns = {'time'};
values = time;
axes = {'x','y','z'};
for index = 1:numel(loads)
    load = loads(index);
    prefix = char(load.prefix);
    body = char(load.body);
    assert(~isempty(regexp(body, '^[a-zA-Z0-9_./-]+$', 'once')), ...
        'Invalid OpenSim body name for %s.', prefix);
    validateattributes(load.force_N, {'numeric'}, {'size',[numel(time),3],'real','finite'});
    validateattributes(load.point_m, {'numeric'}, {'size',[numel(time),3],'real','finite'});
    columns = [columns, strcat(prefix, '_force_', axes), strcat(prefix, '_point_', axes)];
    values = [values, load.force_N, load.point_m];
    if isfield(load, 'torque_Nm') && ~isempty(load.torque_Nm)
        validateattributes(load.torque_Nm, {'numeric'}, ...
            {'size',[numel(time),3],'real','finite'});
        columns = [columns, strcat(prefix, '_torque_', axes)];
        values = [values, load.torque_Nm];
    end
end
if ~isfolder(folder)
    [ok, message] = mkdir(folder);
    assert(ok, '%s', message);
end

mot = fopen(fullfile(folder, 'trial_loads.mot'), 'w');
assert(mot >= 0, 'Could not open the motion file.');
closeMotion = onCleanup(@() fclose(mot));
fprintf(mot, 'ATLAS trial loads\nversion=1\nnRows=%d\nnColumns=%d\ninDegrees=no\nendheader\n', size(values));
fprintf(mot, '%s\n', strjoin(columns, sprintf('\t')));
fprintf(mot, [repmat('%.17g\t',1,size(values,2)-1) '%.17g\n'], values.');
clear closeMotion

xml = fopen(fullfile(folder, 'ExternalLoads.xml'), 'w');
assert(xml >= 0, 'Could not open the ExternalLoads file.');
closeXml = onCleanup(@() fclose(xml));
fprintf(xml, ['<?xml version="1.0" encoding="UTF-8"?>\n' ...
    '<OpenSimDocument Version="40000">\n<ExternalLoads name="externalloads">\n<objects>\n']);
for index = 1:numel(loads)
    load = loads(index);
    prefix = char(load.prefix);
    fprintf(xml, ['  <ExternalForce name="%s">\n' ...
        '    <applied_to_body>%s</applied_to_body>\n' ...
        '    <force_expressed_in_body>ground</force_expressed_in_body>\n' ...
        '    <point_expressed_in_body>ground</point_expressed_in_body>\n' ...
        '    <force_identifier>%s_force_</force_identifier>\n' ...
        '    <point_identifier>%s_point_</point_identifier>\n'], ...
        prefix, char(load.body), prefix, prefix);
    if isfield(load, 'torque_Nm') && ~isempty(load.torque_Nm)
        fprintf(xml, '    <torque_identifier>%s_torque_</torque_identifier>\n', prefix);
    end
    fprintf(xml, '  </ExternalForce>\n');
end
fprintf(xml, ['</objects>\n<groups/>\n<datafile>trial_loads.mot</datafile>\n' ...
    '</ExternalLoads>\n</OpenSimDocument>\n']);
end
