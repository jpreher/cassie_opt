function stepWidth(bounds, nlp, stanceFoot)
T = bounds.params.T;
domain = nlp.Plant;
x = domain.States.x;
right_foot = domain.getCartesianPosition(domain.ContactPoints.RightSole);
left_foot  = domain.getCartesianPosition(domain.ContactPoints.LeftSole);
constraint = tomatrix(left_foot(1:2) - right_foot(1:2));
v_target = bounds.params.vd;

constraint_func = SymFunction(['step_distance_',domain.Name], constraint, {x});

lb = [0;0];
ub = [0;0];

% X Direction
% if v_target(1) < 0
%     ub(1) = max(-v_target(1)*T/4, -2*v_target(1)*T);
%     lb(1) = min(v_target(1)*T/4, 2*v_target(1)*T);
% else
%     ub(1) = max(v_target(1)*T/4, 2*v_target(1)*T);
%     lb(1) = min(-v_target(1)*T/4, -2*v_target(1)*T);
% end
if strcmp(stanceFoot, 'Left')
    lb(1) = min(v_target(1)*T/4, 2*v_target(1)*T);
    ub(1) = max(v_target(1)*T/4, 2*v_target(1)*T);
else
    lb(1) = min(- v_target(1)*T/4, - 2*v_target(1)*T);
    ub(1) = max(- v_target(1)*T/4, - 2*v_target(1)*T);
end

% Y Direction
if v_target(2) < 0
    ub(2) = 0.30 + max(-v_target(2)*T/4, -2*v_target(2)*T);
    lb(2) = 0.20 - min(-v_target(2)*T/4, -2*v_target(2)*T);
else
    ub(2) = 0.30 + max(v_target(2)*T/4, 2*v_target(2)*T);
    lb(2) = 0.20 - min(v_target(2)*T/4, 2*v_target(2)*T);
end

addNodeConstraint(nlp, constraint_func, {'x'}, 'first', lb, ub, 'NonLinear');

lb = [-2.5;
    0.14];
ub = [+2.5;
    1.5];
addNodeConstraint(nlp, constraint_func, {'x'}, 2:nlp.NumNode, lb, ub, 'NonLinear');
end

