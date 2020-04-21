%% Script: setupOpt
%
% Description: This function establishes the optimization bounds, the
%    desired cost function, and creates the NLP used in the optimizer.
%
% Author: Jenna Reher, jreher@caltech.edu
% ________________________________________

function [ nlp ] = setupOpt( behavior, vd )
import([behavior.name, '.Constraints.*']);

% Modify model boundaries
model_bounds = behavior.robotModel.getLimits();

ind = behavior.robotModel.getJointIndices('BasePosX');
model_bounds.states.x.lb(ind)  = -0.5;
model_bounds.states.x.ub(ind)  =  0.5;

ind = behavior.robotModel.getJointIndices('BasePosY');
model_bounds.states.x.lb(ind)  = -0.5;
model_bounds.states.x.ub(ind)  =  0.5;

ind = behavior.robotModel.getJointIndices('BasePosZ');
model_bounds.states.x.lb(ind)  = 0.90; 
model_bounds.states.x.ub(ind)  = 0.95;
model_bounds.states.dx.lb(ind) = -0.75;   
model_bounds.states.dx.ub(ind) =  0.75;   

ind = behavior.robotModel.getJointIndices('BaseRotX');
model_bounds.states.x.lb(ind)  = -0.1;%-0.15;
model_bounds.states.x.ub(ind)  = 0.1;% 0.15;
model_bounds.states.dx.lb(ind)  = -0.1;%-0.1;
model_bounds.states.dx.ub(ind)  = 0.1;% 0.1;

ind = behavior.robotModel.getJointIndices('BaseRotY');
model_bounds.states.x.lb(ind)  = -0.1;
model_bounds.states.x.ub(ind)  =  0.1;
model_bounds.states.dx.lb(ind)  = -0.05;%;
model_bounds.states.dx.ub(ind)  = 0.05; % ;

ind = behavior.robotModel.getJointIndices('BaseRotZ');
model_bounds.states.dx.lb(ind)  = -0.05; %;
model_bounds.states.dx.ub(ind)  = 0.05;  % 0.05;

bounds = struct();
bounds.RightSS           = model_bounds;
bounds.LeftImpactRelabel = model_bounds;

% Floating base velocity
bounds.RightSS.params.vd = vd;

%%% Actuator Bounds %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bounds.RightSS.inputs.Control.u.lb(10) = 0;
bounds.RightSS.inputs.Control.u.ub(10) = 0;

%%% Time %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bounds.RightSS.time.t0.lb = 0;
bounds.RightSS.time.t0.ub = 0;
bounds.RightSS.time.t0.x0 = 0;
bounds.RightSS.time.tf.lb = 0.4;
bounds.RightSS.time.tf.ub = 0.4;
bounds.RightSS.time.tf.x0 = 0.4;

%%% Holonomic Constants %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bounds.RightSS.params.pRightSole.lb  = zeros(5,1);
bounds.RightSS.params.pRightSole.ub  = zeros(5,1);
bounds.RightSS.params.pSpringTransmissions.lb = zeros(2,1);
bounds.RightSS.params.pSpringTransmissions.ub = zeros(2,1);
bounds.RightSS.params.pRigidSprings.lb = zeros(2,1);
bounds.RightSS.params.pRigidSprings.ub = zeros(2,1);
%%% Constraint Wrench Forces %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bounds.RightSS.inputs.ConstraintWrench.fRightSole.lb  = [-150, -150,   0, -250, -250];
bounds.RightSS.inputs.ConstraintWrench.fRightSole.ub  = [+150, +150, 750, +250, +250];
bounds.RightSS.inputs.ConstraintWrench.fSpringTransmissions.lb = -10000;
bounds.RightSS.inputs.ConstraintWrench.fSpringTransmissions.ub =  10000;
bounds.RightSS.inputs.ConstraintWrench.fRigidSprings.lb = -10000;
bounds.RightSS.inputs.ConstraintWrench.fRigidSprings.ub =  10000;

%%% Phase Variable
bounds.RightSS.params.pposition.lb = [bounds.RightSS.time.tf.lb, 0];
bounds.RightSS.params.pposition.ub = [bounds.RightSS.time.tf.ub, 0];
bounds.RightSS.params.pposition.x0 = [bounds.RightSS.time.tf.x0, 0]; 
% bounds.RightSS.params.pposition.lb = [-0.15, -0.50]; 
% bounds.RightSS.params.pposition.ub = [ 0.50,  0.15]; 
% bounds.RightSS.params.pposition.x0 = [ 0.10, -0.10];
% bounds.RightSS.params.pvelocity.lb = bounds.RightSS.params.pposition.lb;
% bounds.RightSS.params.pvelocity.ub = bounds.RightSS.params.pposition.ub;
% bounds.RightSS.params.pvelocity.x0 = bounds.RightSS.params.pposition.x0;

%%% RD2 Outputs
% tmp = zeros(9,7);    
% tmp(1,:) = -0.5;
% tmp(2,:) = -0.5;
% tmp(3,:) = -0.15;
% tmp(4,:) = -0.15;
% tmp(5,:) = +0.20;
% tmp(6,:) = +0.20;
% tmp(7,:) = -0.80;
% tmp(8,:) = -2.80;
% bounds.RightSS.params.aposition.lb = tmp(:);
% 
% tmp = zeros(9,7);    
% tmp(1,:) = +0.5;
% tmp(2,:) = +0.5;
% tmp(3,:) = +0.15;
% tmp(4,:) = +0.15;
% tmp(5,:) = +1.20;
% tmp(6,:) = +1.20;
% tmp(7,:) = +0.80;
% tmp(8,:) = +2.80;
% bounds.RightSS.params.aposition.ub = tmp(:);
bounds.RightSS.params.aposition.lb = -10;
bounds.RightSS.params.aposition.ub = +10;

%%% Feedback PD gains %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bounds.RightSS.velocity.ep = 10;
bounds.RightSS.position.kp = 400;
bounds.RightSS.position.kd = 60;


%% Setup the NLP
behavior.vertices.r_SS.UserNlpConstraint  = str2func('right_SS_constraints');
behavior.edges.l_impact.UserNlpConstraint = str2func('left_impact_relabel_constraints');

num_grid.RightSS = 14;
nlp = HybridTrajectoryOptimization(behavior.name, behavior.hybridSystem, num_grid, ...
                                   [], 'EqualityConstraintBoundary', 1e-5);

nlp.configure(bounds);

% nlp.Phase(1).removeConstraint('y_position_RightSS');
% nlp.Phase(1).removeConstraint('d1y_position_RightSS');
% nlp.Phase(1).removeConstraint('position_output_dynamics');

%% Add a cost function (or lots)
% % Choose the cost type
weight = 1e1;
CostType = {'BaseMovement'};  %-SS
nlp = Opt.applyCost(behavior, nlp, CostType, weight, vd);

weight= 1e-2;
CostType = {'TorqueSquare'};    %-SS 
nlp = Opt.applyCost(behavior, nlp, CostType, weight);

weight= 1e2;
CostType = {'NSFMovement'};    %-SS 
nlp = Opt.applyCost(behavior, nlp, CostType, weight, vd);


nlp.update;

end
