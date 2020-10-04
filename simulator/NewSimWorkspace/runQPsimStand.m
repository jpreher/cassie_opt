%% Simulation parameters
clear;clc;

% Load gait parameter library
% Standing duration
duration = 1.5;
step_size = 1/1000;

% Construct inverse kinematics problem
problem = NonlinearLeastSquaresProblem(...
    LegIKFunction, ...
    CassieCore.motorPositionLowerLimit, ...
    CassieCore.motorPositionUpperLimit);
solver = NonlinearLeastSquaresSolver(problem);

% Set desired foot frames
problem.residualFunction.leftFootTransform  = Transform3d().translate([-0.02;  0.17; -0.80]);
problem.residualFunction.rightFootTransform = Transform3d().translate([-0.02; -0.17; -0.80]);

% Solve inverse kinematics
solver.solve;

% Set initial pelvis position and rotation
pelvisPosition = [0; 0; 0.80];
pelvisVelocity = [0; 0; 0];
pelvisRotation = [0; 0; 0]; % x,y,z

% Set pelvis safety catch height
pelvisCatchHeight = 0.0;

% Set encoder values
qm = solver.getSolution;
qj = [0; deg2rad(13) - qm(4); 0; deg2rad(13) - qm(9)];

q = zeros(22,1);
q(1:3) = pelvisPosition;
q(4:6) = pelvisRotation;

im = [7,8,9,10, 14,    15,16,17,18,  22];
q(im)= qm;
q([11,12, 19,20]) = qj;
dq = zeros(22,1);
x0 = [q;dq];


% Pre-update the dynamics
dyn = CassieEOM(false);
dyn.update(q, dq, true, true);


% Design the standing
param = struct();
param.controller = 'QPstand'; % ['PDstand', 'QPstand']
% param.controller = 'PDstand';
param.w_slack = 1e4;
param.yd = qm;
param.dyd = zeros(10,1);
param.yd_qp = [q(1:3);
               q(4:6)];
param.dyd_qp = zeros(6,1);

outputs = standOutputs();
controller = QPcontroller(outputs);


% loggerger allocation
logger = struct();
logger.param = struct('todo',0);
logger.flow = struct('t', [], 'q', [], 'dq', [],'u',[], 'ya', [], 'dya', [], 'yd', [], 'dyd', []);
logger.QP = struct('exitflag', [],'numiter',[]);


% Run the simulation
t0 = 0;
guardTriggered = false;
bs = '\b'; lastsize = 0;
while ~guardTriggered
    % Simulation settings
    options = odeset('MaxStep', 5e-3, 'RelTol',1e-6,'AbsTol',1e-6);
    
    % Generate a "dumb" crouch
    if t0 > 0.3 && t0 < 0.95
        param.yd_qp(3) = param.yd_qp(3) - step_size*0.2;
    end

    % Run simulation on small fixed size window
    % Compute QP standing controller
    dyn.update(q,dq);
    u = controller.update(q, dq, dyn, param);
    [t, x] = ode15s(@(t ,x) simDynamics(t, x, dyn, controller, param, u), [0, step_size], x0, options);
        
    % logger
    logger.flow.t   = [logger.flow.t,  t0 + t(end)];
    logger.flow.q   = [logger.flow.q,  x(end,1:22)'];
    logger.flow.dq  = [logger.flow.dq, x(end,23:44)'];
    logger.flow.u   = [logger.flow.u,  u];
    logger.flow.ya  = [logger.flow.ya, controller.outputs.ya];
    logger.flow.dya = [logger.flow.ya, controller.outputs.dya];
    logger.flow.yd  = [logger.flow.yd, param.yd_qp];
    logger.flow.dyd = [logger.flow.yd, param.dyd_qp];

    % logger for next loop
    t0 = logger.flow.t(end);
    x0(1:22)  = logger.flow.q(:,end);
    x0(23:44) = logger.flow.dq(:,end);
    q = x0(1:22);
    dq = x0(23:44);

    % Print current time
    lastsize = fprintf([repmat('\b', 1, lastsize) 't: %f / %f'], t0, duration);
    
    % Terminal condition check
    if logger.flow.t(end) >= duration
        guardTriggered = true;
        fprintf('\n');
    end
end




% Animate
orientation = zeros(4,length(logger.flow.t));
for i = 1:length(logger.flow.t)
    R = Rotation3d();
    R = R.rotZYX(flipud(logger.flow.q(4:6,i)));
    orientation(:,i) = getValue(getQuaternion(R));
end
iJoint = [CassieStateEnum.LeftShinPitch;
          CassieStateEnum.LeftTarsusPitch;
          CassieStateEnum.LeftToePitch;
          CassieStateEnum.RightShinPitch;
          CassieStateEnum.RightTarsusPitch;
          CassieStateEnum.RightToePitch];
VizState = [orientation;
            logger.flow.q(1:3,:);
            logger.flow.q(im,:);
            logger.flow.q(iJoint,:)];
viz = CassieVisualizer(logger.flow.t, VizState);




% Plot
figure(100)
plot(logger.flow.t, logger.flow.u);
title('Control Effort'); xlabel('Time (s)'); ylabel('u (Nm)');

figure(101)
plot(logger.flow.t, logger.flow.ya);
hold on; grid on;
plot(logger.flow.t, logger.flow.yd, '--');
title('Outputs'); xlabel('Time (s)'); ylabel('y');

