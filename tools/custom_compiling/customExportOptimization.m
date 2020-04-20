%% Function: customExportOptimization
%
% Description:
%   Compile expressions with customizible options
%
% Author: Jenna Reher, jreher@caltech.edu
% ______________________________________
function [ ] = customExportOptimization( behavior, nlp, do_export_optModel,...
                                         do_export_optBehavior,...
                                         do_export_optCost )
%% Model
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

if do_export_optModel
   compileCoriolis(nlp, export_path);
   compileConstraint(nlp, [], {'dynamics_equation'}, export_path, []);
end

if do_export_optBehavior
    compileDynamicsWithoutCoriolis(nlp, export_path); % Compile dynamical functions partially
end

%% Behavior
behaviorName = behavior.name;
%if ispc
    export_path = strcat('export/', behaviorName, '/opt');
%else
%    export_path = strcat(ros_resolve_local_url('package://cassie_opt/matlab/'), 'export/', behaviorName, '/opt');
%end
if ~exist(export_path,'dir')
    mkdir(export_path);
end
addpath(export_path);

if do_export_optBehavior
    compileConstraint(nlp,[],[], export_path,{'dynamics_equation'});
end

    % to compile a particular constraint
    % compileConstraint(nlp,[],{'nsfVel_baseVel_x'},export_path);

%% Cost Function
if do_export_optCost
    compileObjective(nlp, [],[], export_path);
end

end