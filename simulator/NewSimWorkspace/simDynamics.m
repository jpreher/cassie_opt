function dx = simDynamics(t, x, dyn, controller, param, u_override, logger)
im = [7,8,9,10, 14,    15,16,17,18,  22];
% Update the states
q = x(1:22);
dq = x(23:44);

% % Get the average velocity over the last step
% if ~isempty(logger.flow.t)
%     if ( (t - logger.flow.t(end)) > 0 )
%         param.v_step_avg_allocator = param.v_step_avg_allocator + dq(1:2);
%         param.v_step_avg_count = param.v_step_avg_count + 1;
%         
%         % Update the average speed
%         controller.v_avg_allocator = controller.v_avg_allocator + dq(1:2);
%         controller.v_avg_count = controller.v_avg_count + 1;
%     end
% end

% Update dynamics
dyn.update(q, dq, ...
    strcmp(param.stance_leg, 'Left'), strcmp(param.stance_leg, 'Right'), ...
    strcmp(param.stance_leg, 'Right') ,strcmp(param.stance_leg, 'Left'));

% Compute the control law
if isempty(u_override)
    if strcmp(param.controller, 'PDstand')
        % Compute PD control
        err = q(im) - param.gait_param.x0(im);
        derr = dq(im) - zeros(length(im),1);
        u = motorPD(err, derr);
        
    elseif strcmp(param.controller, 'QPstand') || strcmp(param.controller, 'QPwalk')
        % Compute QP controller
        param.timer.update(t);
        u = controller.update(q, dq, dyn, param);
        
    end
else
    u = u_override;
    
end

% Evaluate the closed loop dynamics with the non-constraint projected
% dynamics
% Generate the vector fields
Fv = + dyn.Fspring - dyn.Cvec - dyn.Gvec;
XiInv = dyn.Jc * (dyn.De \ transpose(dyn.Jc));
Gv = dyn.Be * u;
lambda = -XiInv \ (dyn.dJc * dq + dyn.Jc * (dyn.De \ (Fv + Gv)));

for i = 1:size(dyn.Jc,1)
    tol = 1e-3;
    if (norm(dyn.Jc(i,:) * dq) > tol)
        %         warning('The holonomic constraint at index %i violated', i);
    end
end

% Log if requested
if nargin > 6
    logger.flow.t    = [logger.flow.t,    t];
    logger.flow.q    = [logger.flow.q,    q];
    logger.flow.dq   = [logger.flow.dq,   dq];
    logger.flow.u    = [logger.flow.u,    u];
    logger.flow.F    = [logger.flow.F,    lambda];
    logger.flow.tau  = [logger.flow.tau,  param.phase.tau];
    logger.flow.dtau = [logger.flow.dtau, param.phase.dtau];
    logger.flow.ya   = [logger.flow.ya,   controller.outputs.ya];
    logger.flow.dya  = [logger.flow.dya,   controller.outputs.dya];
    logger.flow.yd   = [logger.flow.yd,   controller.outputs.yd];
    logger.flow.dyd  = [logger.flow.dyd,   controller.outputs.dyd];
    logger.flow.Veta = [logger.flow.Veta, controller.outputs.Veta];
    logger.flow.vel_des = [logger.flow.vel_des, param.vd];
end

% Final dynamics
dx = [dq;
    dyn.De \ (Fv + Gv + dyn.Jc' * lambda)];

end