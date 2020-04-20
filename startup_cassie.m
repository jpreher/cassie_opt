%% Script: startup_cassie
%
% Description: This script runs the startup procedure for the Cassie robot
%   simulator and optimization. The result is a behavior class which
%   contains all relevant information to the model and hybrid system. The
%   export flags control whether the symbolic expressions are generated,
%   while the "delay_coriolis" flag determines whether the backend actually
%   evaluates thefr symbolic expressions for the coriolis matrix (not
%   necessary unless compiling). The "is_symmetric" flag states whether you
%   want to apply a reset map such that the right leg is always stance, or
%   have the system actually swap legs. For optimization, set to "true".
%
% Author: Jenna Reher (jreher@caltech)    
%         Wenlong Ma (wma@caltech)
% ________________________________________
addpath(genpath('tools'));
addpath(genpath('simulator'));
addpath('modules/cassie_description/MATLAB');

%%% Control the symbolic expression export %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
delay_coriolis  = true;
omit_coriolis   = true;

   do_export_model = 0;
do_export_behavior = 0;
      is_symmetric = 1;

%%% Stepping (time-based) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% behaviorName = 'stepping_rigid_planar_v4';
behaviorName = 'stepping_v4';
%     behaviorName = 'walking_v4_planar';
    
%%  Run the associated behavior constructor %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
behavior = loadBehavior(behaviorName, is_symmetric, ...
                        delay_coriolis, omit_coriolis, do_export_behavior, do_export_model);