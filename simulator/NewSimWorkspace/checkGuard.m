function [guardTriggered, isterminal, direction] = checkGuard(~, x, param)
q = x(1:22);

guardTriggered = 1;
if strcmp(param.stance_leg, 'Right')
    p_swf = frost_expr.constraints.p_leftSole_constraint(q);
    if (param.phase.tau > 0.5 && p_swf(3) <= 0) || param.phase.tau >= 1
        % Set appropriate params
        guardTriggered = 0;
    end
elseif strcmp(param.stance_leg, 'Left')
    p_swf = frost_expr.constraints.p_rightSole_constraint(q);
    if (param.phase.tau > 0.5 && p_swf(3) <= 0) || param.phase.tau >= 1
        % Set appropriate params
        guardTriggered = 0;
    end
else
    error('That is not a foot');
end
isterminal = 1;
direction = 0;
end

