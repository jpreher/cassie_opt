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
model_bounds.params.vd = vd;

ind = behavior.robotModel.getJointIndices('BasePosX');
model_bounds.states.x.lb(ind)  = -0.5;
model_bounds.states.x.ub(ind)  =  0.5;

ind = behavior.robotModel.getJointIndices('BasePosY');
model_bounds.states.x.lb(ind)  = -0.5;
model_bounds.states.x.ub(ind)  =  0.5;

ind = behavior.robotModel.getJointIndices('BasePosZ');
model_bounds.states.x.lb(ind)  = 0.75; 
model_bounds.states.x.ub(ind)  = 0.85;
model_bounds.states.dx.lb(ind) = -1.25;   
model_bounds.states.dx.ub(ind) =  1.25;   

ind = behavior.robotModel.getJointIndices('BaseRotX');
model_bounds.states.x.lb(ind)  = -0.15;
model_bounds.states.x.ub(ind)  =  0.15;
model_bounds.states.dx.lb(ind)  = -0.5;
model_bounds.states.dx.ub(ind)  =  0.5;

ind = behavior.robotModel.getJointIndices('BaseRotY');
model_bounds.states.x.lb(ind)  = -0.1;
model_bounds.states.x.ub(ind)  =  0.1;
model_bounds.states.dx.lb(ind)  = -0.5;
model_bounds.states.dx.ub(ind)  =  0.5;

ind = behavior.robotModel.getJointIndices('BaseRotZ');
model_bounds.states.dx.lb(ind)  = -0.50;
model_bounds.states.dx.ub(ind)  =  0.50;

%%% Time %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model_bounds.time.t0.lb = 0;
model_bounds.time.t0.ub = 0;
model_bounds.time.tf.lb = 0.60;
model_bounds.time.tf.ub = 0.90;
model_bounds.time.duration.lb = 0.30;
model_bounds.time.duration.ub = 0.45;

model_bounds.params.pposition.lb = [model_bounds.time.duration.lb, 0];
model_bounds.params.pposition.ub = [model_bounds.time.duration.ub, 0];

model_bounds.params.pSpringTransmissions.lb = zeros(2,1);
model_bounds.params.pSpringTransmissions.ub = zeros(2,1);
model_bounds.params.pRigidSprings.lb = zeros(2,1);
model_bounds.params.pRigidSprings.ub = zeros(2,1);

model_bounds.inputs.ConstraintWrench.fSpringTransmissions.lb = -10000;
model_bounds.inputs.ConstraintWrench.fSpringTransmissions.ub =  10000;
model_bounds.inputs.ConstraintWrench.fRigidSprings.lb = -10000;
model_bounds.inputs.ConstraintWrench.fRigidSprings.ub =  10000;

model_bounds.params.aposition.lb = -3;
model_bounds.params.aposition.ub =  3;

model_bounds.velocity.ep = 10;
model_bounds.position.kp = 400;
model_bounds.position.kd = 60;

% Populate domain specific parameters
bounds = struct();
bounds.RightSS     = model_bounds;
bounds.LeftSS      = model_bounds;
bounds.LeftImpact  = model_bounds;
bounds.RightImpact = model_bounds;

%%% Actuator Bounds %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bounds.RightSS.inputs.Control.u.lb(10) = 0;
bounds.RightSS.inputs.Control.u.ub(10) = 0;

%%% Holonomic Constants %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bounds.RightSS.params.pRightSole.lb  = zeros(5,1);
bounds.RightSS.params.pRightSole.ub  = zeros(5,1);

%%% Constraint Wrench Forces %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bounds.RightSS.inputs.ConstraintWrench.fRightSole.lb  = [-150, -150,   0, -250, -250];
bounds.RightSS.inputs.ConstraintWrench.fRightSole.ub  = [+150, +150, 750, +250, +250];


%% Setup the NLP
behavior.vertices.r_SS.UserNlpConstraint  = str2func('right_SS_constraints');
behavior.edges.l_impact.UserNlpConstraint = str2func('left_impact_relabel_constraints');

num_grid.RightSS = 14;
nlp = HybridTrajectoryOptimization(behavior.name, behavior.hybridSystem, num_grid, ...
                                   [], 'EqualityConstraintBoundary', 1e-5);

nlp.configure(bounds);

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
