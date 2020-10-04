function [q, dq, param] = applyGuard(q, dq, dyn, param)
if strcmp(param.stance_leg, 'Right')
    % Set appropriate params
    param.stance_leg = 'Left';
    
    % Perform a rigid body impact!
    q(CassieStateEnum.RightShinPitch) = 0;
    q(CassieStateEnum.RightHeelSpring) = 0;
    q(CassieStateEnum.RightTarsusPitch) = deg2rad(13) - q(CassieStateEnum.RightKneePitch);
    dyn.update(q, dq, ...
        strcmp(param.stance_leg, 'Left'), strcmp(param.stance_leg, 'Right'), ...
        strcmp(param.stance_leg, 'Right') ,strcmp(param.stance_leg, 'Left'));
    nImpConstr = size(dyn.Jc,1);
    A_imp = [dyn.De -dyn.Jc'; dyn.Jc zeros(nImpConstr)];
    b_imp = [dyn.De*dq; zeros(nImpConstr,1)];
    y_imp = A_imp\b_imp;
    ImpF = y_imp((22+1):end);
    dq = y_imp(1:22);
elseif strcmp(param.stance_leg, 'Left')
    % Set appropriate params
    param.stance_leg = 'Right';
    
    % Perform a rigid body impact!
    q(CassieStateEnum.LeftShinPitch) = 0;
    q(CassieStateEnum.LeftHeelSpring) = 0;
    q(CassieStateEnum.LeftTarsusPitch) = deg2rad(13) - q(CassieStateEnum.LeftKneePitch);
    dyn.update(q, dq, ...
        strcmp(param.stance_leg, 'Left'), strcmp(param.stance_leg, 'Right'), ...
        strcmp(param.stance_leg, 'Right') ,strcmp(param.stance_leg, 'Left'));
    nImpConstr = size(dyn.Jc,1);
    A_imp = [dyn.De -dyn.Jc'; dyn.Jc zeros(nImpConstr)];
    b_imp = [dyn.De*dq; zeros(nImpConstr,1)];
    y_imp = A_imp\b_imp;
    ImpF = y_imp((22+1):end);
    dq = y_imp(1:22);
else
    error('That is not a foot');
end
end

