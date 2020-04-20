%% Function: loadBehavior
%
% Description: 
%
% Author: Jenna Reher, jreher@caltech.edu
%         Wenlong Ma, wma@caltech.edu
% ________________________________________

function behavior = loadBehavior(behaviorName, is_symmetric,...
                       delay_coriolis, omit_coriolis, do_export_behavior, do_export_model)

% Remove all other on the export path
p = strread(path,'%s','delimiter',':');
for i = 1:numel(p)
    if ~isempty(strfind(p{i}, 'cassie_opt\matlab\export'))
        rmpath(p{i});
    end
end

% Ensure the config is on path
%if ispc
    config_dir  = 'configuration/';
%else
%    config_dir = ros_resolve_local_url('package://cassie_opt/matlab/configuration/');
%end
addpath(config_dir);

% Load the behavior
behavior_path = strcat(config_dir, 'behaviors/');
addpath(behavior_path);

behavior = feval(strcat(behaviorName, '.behavior'));
behavior.init(is_symmetric, delay_coriolis, omit_coriolis);

% Export if requested
%%% Dynamics
robot = behavior.robotModel;
if strcmp(robot.States.x.label{3}, 'BaseRotY')
    baseType = '2D';
else
    baseType = '3D';
end
%if ispc
    export_path = strcat('export/', 'dynamics/', robot.Name, '/', baseType);
%else
%    export_path = strcat(ros_resolve_local_url('package://cassie_opt/matlab/'), 'export/', 'dynamics/', robot.Name, '/', baseType);
%end
if ~exist(export_path,'dir')
    mkdir(export_path);
end
addpath(export_path);
if do_export_model
    robot.Mmat.export(export_path);
    for i = 1:numel(robot.Fvec)
        robot.Fvec{i}.export(export_path);
    end
end

%%% Behavior
%if ispc
    export_path = strcat('export/', behaviorName, '/sim');
%else
%    export_path = strcat(ros_resolve_local_url('package://cassie_opt/matlab/'), 'export/', behaviorName, '/sim');
%end
if ~exist(export_path,'dir')
    mkdir(export_path);
end
addpath(export_path);

%%% this can handle odd number of edges
if do_export_behavior
    v = fields(behavior.vertices);
    for i = 1:numel(v)
        customCompileVertex(behavior.vertices.(v{i}), export_path);
    end
    
    if ~isempty(behavior.edges)
        e = fields(behavior.edges);
        for i =  1:numel(e)
            customCompileEdge(behavior.edges.(e{i}), export_path);
        end
    end
end

end