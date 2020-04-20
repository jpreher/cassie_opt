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
model_bounds.states.x.lb(ind)  = 0.80; 
model_bounds.states.x.ub(ind)  = 0.95;
model_bounds.states.dx.lb(ind) = -0.75;   
model_bounds.states.dx.ub(ind) =  0.75;   

ind = behavior.robotModel.getJointIndices('BaseRotX');
model_bounds.states.x.lb(ind)  = 0;
model_bounds.states.x.ub(ind)  = 0;
model_bounds.states.dx.lb(ind)  = 0;
model_bounds.states.dx.ub(ind)  = 0;

ind = behavior.robotModel.getJointIndices('BaseRotY');
model_bounds.states.x.lb(ind)  = -0.07;
model_bounds.states.x.ub(ind)  =  0.07;
model_bounds.states.dx.lb(ind)  = -0.5;
model_bounds.states.dx.ub(ind)  =  0.5;

ind = behavior.robotModel.getJointIndices('BaseRotZ');
model_bounds.states.x.lb(ind)  = 0;
model_bounds.states.x.ub(ind)  = 0;
model_bounds.states.dx.lb(ind)  = 0;
model_bounds.states.dx.ub(ind)  = 0;

ind = behavior.robotModel.getJointIndices('LeftShinPitch');
model_bounds.states.x.lb(ind) = 0;
model_bounds.states.x.ub(ind) = 0;
model_bounds.states.dx.lb(ind) = 0;
model_bounds.states.dx.ub(ind) = 0;

% ind = behavior.robotModel.getJointIndices('LeftHeelSpring');
% model_bounds.states.x.lb(ind) = 0;
% model_bounds.states.x.ub(ind) = 0;
% model_bounds.states.dx.lb(ind) = 0;
% model_bounds.states.dx.ub(ind) = 0;

bounds = struct();
bounds.RightSS           = model_bounds;
bounds.LeftImpactRelabel = model_bounds;

% Floating base velocity
bounds.RightSS.params.vd = vd;

%%% Actuator Bounds %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
actuator_indices = [ 3, 4, 5,    8, 9, 10 ];
bounds.RightSS.inputs.Control.u.lb = model_bounds.inputs.Control.u.lb(actuator_indices);
bounds.RightSS.inputs.Control.u.ub = model_bounds.inputs.Control.u.ub(actuator_indices);

%%% Time %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bounds.RightSS.time.t0.lb = 0;
bounds.RightSS.time.t0.ub = 0;
bounds.RightSS.time.t0.x0 = 0;
bounds.RightSS.time.tf.lb = 0.34;
bounds.RightSS.time.tf.ub = 0.4;
bounds.RightSS.time.tf.x0 = 0.35;

%%% Holonomic Constants %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bounds.RightSS.params.pRightSole.lb  = zeros(3,1);
bounds.RightSS.params.pRightSole.ub  = zeros(3,1);
bounds.RightSS.params.pSpringTransmissions.lb = zeros(2,1);
bounds.RightSS.params.pSpringTransmissions.ub = zeros(2,1);
bounds.RightSS.params.pPlanarizedJoints.lb = [0, zeros(1,6)];
bounds.RightSS.params.pPlanarizedJoints.ub = [0.50, zeros(1,6)];
bounds.RightSS.params.pPlanarizedJoints.x0 = [0.24, zeros(1,6)];
bounds.RightSS.params.pRigidSprings.lb = [0; 0];%zeros(2,1);
bounds.RightSS.params.pRigidSprings.ub = [0; 0];%zeros(2,1);
%%% Constraint Wrench Forces %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bounds.RightSS.inputs.ConstraintWrench.fRightSole.lb  = [-150,   0, -250];%[-150, -150,   0, -250, -250];
bounds.RightSS.inputs.ConstraintWrench.fRightSole.ub  = [+150, 750, +250];%[+150, +150, 750, +250, +250];
bounds.RightSS.inputs.ConstraintWrench.fSpringTransmissions.lb = -100000;
bounds.RightSS.inputs.ConstraintWrench.fSpringTransmissions.ub =  100000;
bounds.RightSS.inputs.ConstraintWrench.fPlanarizedJoints.lb = -inf;
bounds.RightSS.inputs.ConstraintWrench.fPlanarizedJoints.ub = +inf;
bounds.RightSS.inputs.ConstraintWrench.fPlanarizedJoints.x0 = 0;
bounds.RightSS.inputs.ConstraintWrench.fRigidSprings.lb = -10000;
bounds.RightSS.inputs.ConstraintWrench.fRigidSprings.ub =  10000;

%%% Phase Variable
% bounds.RightSS.params.pposition.lb = [bounds.RightSS.time.tf.lb, 0];
% bounds.RightSS.params.pposition.ub = [bounds.RightSS.time.tf.ub, 0];
% bounds.RightSS.params.pposition.x0 = [bounds.RightSS.time.tf.x0, 0]; 
bounds.RightSS.params.pposition.lb = [-0.15, -0.50]; 
bounds.RightSS.params.pposition.ub = [0.50, 0.15]; 
bounds.RightSS.params.pposition.x0 = [0.10, -0.10];

%%% RD1 Outputs
bounds.RightSS.params.avelocity.lb = 0.2; % RD1
bounds.RightSS.params.avelocity.ub = 2.5; 
bounds.RightSS.params.avelocity.x0 = 0.75;

%%% RD2 Outputs
tmp = zeros(5,7);    
tmp(1,:) = +0.20;
tmp(2,:) = +0.20;
tmp(3,:) = -0.80;
tmp(4,:) = -2.80;
bounds.RightSS.params.aposition.lb = tmp(:);

tmp = zeros(5,7);    
tmp(1,:) = +1.20;
tmp(2,:) = +1.20;
tmp(3,:) = +0.80;
tmp(4,:) = +2.80;
bounds.RightSS.params.aposition.ub = tmp(:);

%%% Feedback PD gains %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bounds.RightSS.velocity.ep = 25;
bounds.RightSS.position.kp = 400;
bounds.RightSS.position.kd = 60;


%% Setup the NLP
behavior.vertices.r_SS.UserNlpConstraint  = str2func('right_SS_constraints');
behavior.edges.l_impact.UserNlpConstraint = str2func('left_impact_relabel_constraints');

num_grid.RightSS = 12;
nlp = HybridTrajectoryOptimization(behavior.name, behavior.hybridSystem, num_grid, ...
                                   [], 'EqualityConstraintBoundary', 1e-5);

nlp.configure(bounds);

%% Add a cost function (or lots)
% % Choose the cost type
weight = 1e1;
CostType = {'BaseMovement'};  %-SS
nlp = Opt.applyCost(behavior, nlp, CostType, weight, vd);

weight= 1e-2;
CostType = {'TorqueSquarePlanar'};    %-SS 
nlp = Opt.applyCost(behavior, nlp, CostType, weight);

weight= 1e2;
CostType = {'NSFMovement'};    %-SS 
nlp = Opt.applyCost(behavior, nlp, CostType, weight, vd);

% COT
r_stance = behavior.vertices.r_SS;
dx = r_stance.States.dx;
u  = r_stance.Inputs.Control.u;
p  = r_stance.Params.pposition;
pf = r_stance.Params.pposition(1) - r_stance.Params.pposition(2);
Mass = sum([behavior.robotModel.Links(:).Mass]);
mg = 9.81 * Mass; 
Be = r_stance.Gmap.Control.u;

% Not a true cost of transport...
cot = sqrt(sum((tovector(u)*(Be'*dx)).^2)).^2 / (mg * pf);
cot_fun = SymFunction(['cot_' r_stance.Name], cot, {u, dx, p});
addRunningCost(nlp.Phase(1), cot_fun, {'u', 'dx', 'pposition'});


nlp.update;

end
