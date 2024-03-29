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
model_bounds.params.T = 0.40;

ind = behavior.robotModel.getJointIndices('BasePosX');
model_bounds.states.x.lb(ind)  = -2.5;
model_bounds.states.x.ub(ind)  =  2.5;
model_bounds.states.dx.lb(ind)  = -4.5;
model_bounds.states.dx.ub(ind)  =  4.5;

ind = behavior.robotModel.getJointIndices('BasePosY');
model_bounds.states.x.lb(ind)  = -2.5;
model_bounds.states.x.ub(ind)  =  2.5;
model_bounds.states.dx.lb(ind)  = -4.5;
model_bounds.states.dx.ub(ind)  =  4.5;

ind = behavior.robotModel.getJointIndices('BasePosZ');
model_bounds.states.x.lb(ind)  = 0.80; 
model_bounds.states.x.ub(ind)  = 0.88;
model_bounds.states.dx.lb(ind) = -0.75;   
model_bounds.states.dx.ub(ind) =  0.75;   

ind = behavior.robotModel.getJointIndices('BaseRotX');
model_bounds.states.x.lb(ind)  = -0.05;
model_bounds.states.x.ub(ind)  =  0.05;
model_bounds.states.dx.lb(ind)  = -0.2;
model_bounds.states.dx.ub(ind)  =  0.2;

ind = behavior.robotModel.getJointIndices('BaseRotY');
model_bounds.states.x.lb(ind)  = 0;
model_bounds.states.x.ub(ind)  = 0.1;
model_bounds.states.dx.lb(ind)  = -0.25;
model_bounds.states.dx.ub(ind)  =  0.25;

ind = behavior.robotModel.getJointIndices('BaseRotZ');
model_bounds.states.x.lb(ind)  = -0.0;
model_bounds.states.x.ub(ind)  =  0.0;
model_bounds.states.dx.lb(ind)  = -0.0;
model_bounds.states.dx.ub(ind)  =  0.0;

ind = behavior.robotModel.getJointIndices('LeftHipYaw');
model_bounds.states.x.lb(ind)  = -0.0;
model_bounds.states.x.ub(ind)  =  0.0;
model_bounds.states.dx.lb(ind)  = -0.0;
model_bounds.states.dx.ub(ind)  =  0.0;

ind = behavior.robotModel.getJointIndices('RightHipYaw');
model_bounds.states.x.lb(ind)  = -0.0;
model_bounds.states.x.ub(ind)  =  0.0;
model_bounds.states.dx.lb(ind)  = -0.0;
model_bounds.states.dx.ub(ind)  =  0.0;

model_bounds.params.pSpringTransmissions.lb = zeros(2,1);
model_bounds.params.pSpringTransmissions.ub = zeros(2,1);
model_bounds.params.pRigidSprings.lb = zeros(2,1);
model_bounds.params.pRigidSprings.ub = zeros(2,1);

model_bounds.inputs.ConstraintWrench.fSpringTransmissions.lb = -10000;
model_bounds.inputs.ConstraintWrench.fSpringTransmissions.ub =  10000;
model_bounds.inputs.ConstraintWrench.fRigidSprings.lb = -10000;
model_bounds.inputs.ConstraintWrench.fRigidSprings.ub =  10000;

model_bounds.params.aposition.lb = -5;
model_bounds.params.aposition.ub =  5;

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
bounds.LeftSS.inputs.Control.u.lb(5) = 0;
bounds.LeftSS.inputs.Control.u.ub(5) = 0;

%%% Holonomic Constants %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bounds.RightSS.params.pRightSole.lb  = zeros(5,1);
bounds.RightSS.params.pRightSole.ub  = zeros(5,1);
bounds.LeftSS.params.pLeftSole.lb  = [-1.5, 0,   0, 0, 0];
bounds.LeftSS.params.pLeftSole.ub  = [+1.5, 1.5, 0, 0, 0];

%%% Constraint Wrench Forces %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bounds.RightSS.inputs.ConstraintWrench.fRightSole.lb  = [-150, -150,   0, -250, -40];
bounds.RightSS.inputs.ConstraintWrench.fRightSole.ub  = [+150, +150, 750, +250, +40];
bounds.LeftSS.inputs.ConstraintWrench.fLeftSole.lb  = [-150, -150,   0, -250, -40];
bounds.LeftSS.inputs.ConstraintWrench.fLeftSole.ub  = [+150, +150, 750, +250, +40];

%%% Time %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bounds.RightSS.time.t0.lb = 0;
bounds.RightSS.time.t0.ub = 0;
bounds.RightSS.time.tf.lb = model_bounds.params.T;
bounds.RightSS.time.tf.ub = model_bounds.params.T;

bounds.LeftSS.time.t0.lb = bounds.RightSS.time.tf.lb;
bounds.LeftSS.time.t0.ub = bounds.RightSS.time.tf.ub;
bounds.LeftSS.time.tf.lb = 2 * bounds.RightSS.time.tf.lb;
bounds.LeftSS.time.tf.ub = 2 * bounds.RightSS.time.tf.ub;

bounds.RightSS.params.pposition.lb = [bounds.RightSS.time.tf.lb, bounds.RightSS.time.t0.lb];
bounds.RightSS.params.pposition.ub = [bounds.RightSS.time.tf.ub, bounds.RightSS.time.t0.ub];
bounds.LeftSS.params.pposition.lb = [bounds.LeftSS.time.tf.lb, bounds.LeftSS.time.t0.lb];
bounds.LeftSS.params.pposition.ub = [bounds.LeftSS.time.tf.ub, bounds.LeftSS.time.t0.ub];

%% Setup the NLP
if isfield(behavior.vertices, 'l_SS')
    behavior.vertices.r_SS.UserNlpConstraint  = @constraintsSS; 
    behavior.vertices.l_SS.UserNlpConstraint  = @constraintsSS; 
    behavior.edges.l_impact.UserNlpConstraint = @left_impact_constraints; 
    behavior.edges.r_impact.UserNlpConstraint = @right_impact_constraints;
    num_grid.RightSS = 12;
    num_grid.LeftSS = 12;   
else
    behavior.vertices.r_SS.UserNlpConstraint  = @constraintsSS; 
    behavior.edges.l_impact.UserNlpConstraint = @left_impact_relabel_constraints; 
    num_grid.RightSS = 12;
    
end
    
nlp = HybridTrajectoryOptimization(behavior.name, behavior.hybridSystem, num_grid, ...
                                   [], 'EqualityConstraintBoundary', 1e-5);
nlp.configure(bounds);

% Add heuristic constraint on grf in middle of step
midnode = floor(nlp.Phase(1).NumNode/2);
lb = nlp.Phase(1).OptVarTable.fRightSole(midnode).LowerBound;
lb(3) = 300;
ub = nlp.Phase(1).OptVarTable.fRightSole(midnode).UpperBound;
nlp.Phase(1).OptVarTable.fRightSole(midnode).setBoundary(lb,ub);

midnode = floor(nlp.Phase(3).NumNode/2);
lb = nlp.Phase(3).OptVarTable.fLeftSole(midnode).LowerBound;
lb(3) = 300;
ub = nlp.Phase(3).OptVarTable.fLeftSole(midnode).UpperBound;
nlp.Phase(3).OptVarTable.fLeftSole(midnode).setBoundary(lb,ub);


%% Add a cost function (or lots)
% % Choose the cost type

if isfield(behavior.vertices, 'l_SS')
%     weight = [0; 0; 0; 20; 10; 20];
%     CostType = {'BaseMovement', 'BaseMovement'}; 
%     nlp = Opt.applyCost(behavior, nlp, CostType, weight, vd);

    weight = [20; 1; 30];
    CostType = {'Orientation', 'Orientation'};
    nlp = Opt.applyCost(behavior, nlp, CostType, weight, vd);

    weight= 1e-4;
    CostType = {'TorqueSquare', 'TorqueSquare'}; 
    nlp = Opt.applyCost(behavior, nlp, CostType, weight);
    
%     weight= 1e-3;
%     CostType = {'StanceMoment', 'StanceMoment'}; 
%     nlp = Opt.applyCost(behavior, nlp, CostType, weight);
    
%     weight= 1e-4;
%     CostType = {'NSFMovement', 'NSFMovement'};
%     nlp = Opt.applyCost(behavior, nlp, CostType, weight, vd);
else
    weight = 5e1;
    CostType = {'BaseMovement'};
    nlp = Opt.applyCost(behavior, nlp, CostType, weight, vd);
    
    weight= 1e-2;
    CostType = {'TorqueSquare'};
    nlp = Opt.applyCost(behavior, nlp, CostType, weight);
    
    weight= 1e2;
    CostType = {'NSFMovement'};
    nlp = Opt.applyCost(behavior, nlp, CostType, weight, vd);
end

nlp.update;

end
