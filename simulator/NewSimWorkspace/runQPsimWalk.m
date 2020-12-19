%% Simulation parameters
clear;clc;

% Parameters
n_steps = 1;
step_size = 1/1000;
stance_leg = 'Right';
initial_speed = [0.0, 0]; % [vx0, vy0];


% Load gait parameter library
load('20200927');
gait_param = param_lib(initial_speed(1), initial_speed(2), stance_leg, nested_lib);
final_aposition = gait_param.aposition;

% Initial condition
im = [7,8,9,10, 14,    15,16,17,18, 22];
x0 = gait_param.x0;

q = x0(1:22);
dq = 0.* x0(23:44);

% Pre-update the dynamics
dyn = CassieEOM(true);
dyn.update(q, dq, ...
    strcmp(stance_leg, 'Left'), strcmp(stance_leg, 'Right'), ...
    strcmp(stance_leg, 'Right'),strcmp(stance_leg, 'Left'));


% Design the walking controller
param = GaitParams();
param.controller = 'QPwalk';
param.ep = 2e-1;
param.w_slack = 1e2;
param.stance_leg = stance_leg;
param.gait_param = gait_param;
param.timer = Timer();
param.phase = PhaseTime(flipud(gait_param.pposition));
param.interp_lib = nested_lib;
param.avg_vel = [0;0];
param.prev_avg_vel = initial_speed';
param.k_step = 0;
param.vd = initial_speed';
param.v_step_avg_allocator = [0;0];
param.v_step_avg_count = 0;
param.velocityLP = LowPass(2, 1/step_size, 0.35*2);
param.velocityLP.update(param.vd);
param.v_prev = param.vd;

param.sagittal_offset = -0.01;
param.Kp_x = 0.10;
param.Kp_y = 0.25;
param.Kd_x = 0;
param.Kd_y = 0;

outputs = walkOutputs(param);
controller = QPcontroller(outputs);
logger = CassieLogger();

% Store the current outputs
controller.outputs.update(q,dq,param);
yd_last = controller.outputs.ya;
dyd_last = controller.outputs.dya;

% Run the simulation
t0 = 0;
guardTriggered = false;
bs = '\b'; lastsize = 0;
prev_avg_vel = initial_speed';
options = odeset('MaxStep', 1e-3, 'RelTol',1e-6,'AbsTol',1e-6);

for k_step = 1:n_steps
    % Update params
    param.k_step = k_step;
    guardTriggered = false;
    midstepTriggered = false;
    
    % Complete the step
    while ~guardTriggered
        % Update the timer
        param.timer.update(t0);
        
        % Print current time
        fprintf(repmat('\b', 1, lastsize));
        lastsize = fprintf('\nt: %f\n', t0);
        
        % Run simulation on small fixed size window
        % Compute QP standing controller
        dyn.update(q, dq, ...
            strcmp(param.stance_leg, 'Left'), strcmp(param.stance_leg, 'Right'), ...
            strcmp(param.stance_leg, 'Right') ,strcmp(param.stance_leg, 'Left'));
        [u, F, delta] = controller.update(q, dq, dyn, param);
        Fv = dyn.Fspring - dyn.Cvec - dyn.Gvec;
        XiInv = dyn.Jc * (dyn.De \ transpose(dyn.Jc));
        Gv = dyn.Be * u;
        lambda = -XiInv \ (dyn.dJc * dq + dyn.Jc * (dyn.De \ (Fv + Gv)));
        
        [t, x] = ode15s(@(t ,x) simDynamics(t, x, dyn, controller, param, u), [t0, t0+step_size], x0, options);
        
        % Logger for next loop
        t0 = t(end);
        q = x(end,1:22)';
        dq = x(end,23:44)';
        
        % Update the actual velocity for tracking
        param.velocityLP.update(dq(1:2));
        param.v_step_avg_allocator = param.v_step_avg_allocator + dq(1:2);
        param.v_step_avg_count = param.v_step_avg_count + 1;
        
        % Log data
        nlog = length(t);
        logger.flow.t       = [logger.flow.t,       t'];
        logger.flow.q       = [logger.flow.q,       x(:,1:22)'];
        logger.flow.dq      = [logger.flow.dq,      x(:,23:44)'];
        logger.flow.u       = [logger.flow.u,       repmat(u, 1, nlog)];
        logger.flow.F       = [logger.flow.F,       repmat(lambda, 1, nlog)];
        logger.flow.FQP     = [logger.flow.FQP,     repmat(F, 1, nlog)];
        logger.flow.deltaQP = [logger.flow.deltaQP, repmat(delta, 1, nlog)];
        logger.flow.tau     = [logger.flow.tau,     repmat(param.phase.tau, 1, nlog)];
        logger.flow.dtau    = [logger.flow.dtau,    repmat(param.phase.dtau, 1, nlog)];
        logger.flow.ya      = [logger.flow.ya,      repmat(controller.outputs.ya, 1, nlog)];
        logger.flow.dya     = [logger.flow.dya,     repmat(controller.outputs.dya, 1, nlog)];
        logger.flow.yd      = [logger.flow.yd,      repmat(controller.outputs.yd, 1, nlog)];
        logger.flow.dyd     = [logger.flow.dyd,     repmat(controller.outputs.dyd, 1, nlog)];
        logger.flow.Veta    = [logger.flow.Veta,    repmat(controller.outputs.Veta, 1, nlog)];
        logger.flow.vel_des = [logger.flow.vel_des, repmat(param.vd, 1, nlog)];
        logger.flow.vel_avg = [logger.flow.vel_avg, repmat(param.velocityLP.filteredValue, 1, nlog)];
        
        % Check if the guard was triggered
        grd = checkGuard(t, q, param);
        if grd == 0
            % Apply impact map
            [q, dq] = applyGuard(q, dq, dyn, param);
            
            % Perform a motion transition
            % va = param.velocityLP.filteredValue(1);
            va = param.v_step_avg_allocator / param.v_step_avg_count;            
            param.gait_param = param_lib(param.velocityLP.filteredValue(1), param.velocityLP.filteredValue(2), ...
                param.stance_leg, nested_lib);
            controller.outputs.update(q,dq,param);
            yd_last = controller.outputs.ya;
            dyd_last = controller.outputs.dya;
            param.gait_param.aposition = computeStartTransition( param.phase.dtau, ...
                param.gait_param.pposition, param.gait_param.aposition, ...
                controller.outputs.ya, controller.outputs.dya);
            
%             % Compute Raibert for the end
%             yF   = bezier(param.gait_param.aposition, 1);
%             Kp_raibert = [0.38;  0.255];
%             Kd_raibert = [0.35;  0.15];
%             offset = -Kp_raibert .* (va - param.vd) - Kd_raibert .* (va - param.v_prev);
%             if strcmp(param.stance_leg, 'Right')
%                 offset(2) = clamp(offset(2), -1.0, 0);
%                 yF(1) = yF(1) - offset(2);
%             elseif strcmp(param.stance_leg, 'Left')
%                 offset(2) = clamp(offset(2), 0, 1.0);
%                 yF(1) = yF(1) - offset(2);
%             end
%             yF(7) = yF(7) + offset(1);
%             param.gait_param.aposition = computeEndTransition( param.phase.dtau, param.gait_param.aposition, yF );
            
            %param.v_prev = param.velocityLP.filteredValue;
            param.v_prev = param.v_step_avg_allocator / param.v_step_avg_count;
            
            length_missing = size(logger.flow.t,2) - size(logger.flow.vel_step_avg,2);
            logger.flow.vel_step_avg = [logger.flow.vel_step_avg, repmat(param.velocityLP.filteredValue, 1, length_missing)];
            
            
            
            % Reset things
            param.timer.reset(t0);
            guardTriggered = true;
            controller.new_step();
            param.v_step_avg_allocator  = [0;0];
            param.v_step_avg_count = 0;
        end
        
        %         if param.phase.tau > 0.50 && ~midstepTriggered
        %             midstepTriggered = true; 
        %             
        %             % Compute Raibert for the end
        %             yF   = bezier(param.gait_param.aposition, 1);
        %             Kp_raibert = [0.38;  0.255];
        %             Kd_raibert = [0.35;  0.15];
        %             offset = -Kp_raibert .* (param.velocityLP.filteredValue - param.vd) - Kd_raibert .* (param.velocityLP.filteredValue - param.v_prev);
        %             if strcmp(param.stance_leg, 'Right')
        %                 offset(2) = clamp(offset(2), -1.0, 0);
        %                 yF(1) = yF(1) - offset(2);
        %             elseif strcmp(param.stance_leg, 'Left')
        %                 offset(2) = clamp(offset(2), 0, 1.0);
        %                 yF(1) = yF(1) - offset(2);
        %             end
        %             yF(7) = yF(7) + offset(1);
        %             param.gait_param.aposition = computeEndTransition( param.phase.dtau, param.gait_param.aposition, yF );
        %             
        %             param.v_prev = param.velocityLP.filteredValue;
        %         end
        
        % Set the current walking to the current speed
        %param.gait_param = param_lib(param.velocityLP.filteredValue(1), param.stance_leg, interp_lib);
        %param.gait_param.aposition(:,1) = yd_last;
        %param.gait_param.aposition(:,2) = yd_last + dyd_last*0.35/6;
        %param.gait_param.aposition(9,:) = 0;
        
        x0(1:22)  = q;
        x0(23:44) = dq;
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

figure(102)
plot(logger.flow.t, logger.flow.dya);
hold on; grid on;
plot(logger.flow.t, logger.flow.dyd, '--');
title('Outputs'); xlabel('Time (s)'); ylabel('dy');

figure(103)
plot(logger.flow.t, logger.flow.Veta);
title('Lyapunov Function'); xlabel('Time (s)'); ylabel('V');

figure(104)
plot(logger.flow.t, logger.flow.tau);
title('Phase'); xlabel('Time (s)'); ylabel('tau');

figure(106)
plot(logger.flow.t, logger.flow.F,'--');
hold on; grid on;
plot(logger.flow.t, logger.flow.FQP);
title('Contact Forces'); xlabel('Time (s)'); ylabel('Force (N)');

% figure(107)
% subplot(2,1,1);
% plot(logger.flow.t, logger.flow.F(1:2,:));
% hold on; grid on;
% plot(logger.flow.t, logger.flow.FQP(1:2,:), '--');
% title('Spring Holonomic Forces');  ylabel('Achilles (N)');
% subplot(2,1,2);
% plot(logger.flow.t, logger.flow.F(3:4,:));
% hold on; grid on;
% plot(logger.flow.t, logger.flow.FQP(3:4,:), '--');
% ylabel('Rigid Swing (N)');
% xlabel('Time (s)');

figure(105)
subplot(2,1,1);
plot(logger.flow.t, logger.flow.vel_des(1,:), '--');
hold on; grid on;
% plot(logger.flow.t, logger.flow.vel_avg(1,:));
plot(logger.flow.t, logger.flow.vel_step_avg(2,:));
plot(logger.flow.t, logger.flow.dq(1,:));
title('Velocity Tracking'); xlabel('Time (s)'); ylabel('Velocity X (m/s)');
subplot(2,1,2);
plot(logger.flow.t, logger.flow.vel_des(2,:), '--');
hold on; grid on;
% plot(logger.flow.t, logger.flow.vel_avg(2,:));
plot(logger.flow.t, logger.flow.vel_step_avg(2,:));
plot(logger.flow.t, logger.flow.dq(2,:));
xlabel('Time (s)'); ylabel('Velocity Y (m/s)')















% Run simulation on small fixed size window
% Compute QP standing controller
%         dyn.update(q, dq, ...
%             strcmp(param.stance_leg, 'Left'), strcmp(param.stance_leg, 'Right'), ...
%             strcmp(param.stance_leg, 'Right') ,strcmp(param.stance_leg, 'Left'));
%         u = controller.update(q, dq, dyn, param);
%
%         [t, x] = ode15s(@(t ,x) simDynamics(t, x, dyn, controller, param, u), [0, step_size], x0, options);
%
%         % Logger
%         logger.flow.t    = [logger.flow.t,    t0 + t(end)];
%         logger.flow.q    = [logger.flow.q,    x(end,1:22)'];
%         logger.flow.dq   = [logger.flow.dq,   x(end,23:44)'];
%         logger.flow.u    = [logger.flow.u,    u];
%         logger.flow.tau  = [logger.flow.tau,  param.phase.tau];
%         logger.flow.dtau = [logger.flow.dtau, param.phase.dtau];
%         logger.flow.ya   = [logger.flow.ya,   controller.outputs.ya];
%         logger.flow.dya  = [logger.flow.dya,   controller.outputs.dya];
%         logger.flow.yd   = [logger.flow.yd,   controller.outputs.yd];
%         logger.flow.dyd  = [logger.flow.dyd,   controller.outputs.dyd];
%         logger.flow.Veta = [logger.flow.Veta, controller.outputs.Veta];
%         logger.flow.vel_des = [logger.flow.vel_des, param.vd];
% Track the average velocity
%v_step_avg_allocator = v_step_avg_allocator + dq(1:2);
%v_step_avg_count = v_step_avg_count + 1;

