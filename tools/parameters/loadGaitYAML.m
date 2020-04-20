function [ params ] = loadGaitYAML( behavior, gaitName )
if nargin < 2
    gaitName = [];
end

if ispc
    behaviorPath = strcat('params/', behavior.name, '/');
else
    behaviorPath = strcat(ros_resolve_local_url('package://cassie_opt/matlab/params/'), behavior.name, '/');
end


% If no name specified pull the newest gait..
if isempty(gaitName)
    listing = dir(behaviorPath);
    gaitName = listing(end).name;
end

    % % Filename is always params_(gaitName) - ensure it exists then load
    % fileName = strcat('params_', gaitName, '.yaml');
    % filePath = strcat(behaviorPath, gaitName, '/', fileName);
    % 
    % yaml = yaml_read_file(filePath, true, false);

% Take the only yaml file in the folder so you can rename the folder to whatever you like
filePath = strcat(behaviorPath, gaitName, '/');
tmp = dir([filePath, '*.yaml']);
fileName = strcat(filePath, tmp.name);

yaml = yaml_read_file(fileName, true, false);
params = yaml.domain;

end

