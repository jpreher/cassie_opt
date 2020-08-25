function stepWidth(bounds, nlp, stanceFoot)
T = 0.40;
domain = nlp.Plant;
x = domain.States.x;
right_foot = domain.getCartesianPosition(domain.ContactPoints.RightSole);
left_foot  = domain.getCartesianPosition(domain.ContactPoints.LeftSole);
constraint = tomatrix(left_foot(1:2) - right_foot(1:2));
v_target = bounds.params.vd;

constraint_func = SymFunction(['step_distance_',domain.Name], constraint, {x});

if strcmp(stanceFoot, 'Left')
    lb = [min(v_target(1)*T/4, v_target(1)*T);
          0.20 + min(-v_target(2)*T/4, -v_target(2)*T)];
    ub = [max(v_target(1)*T/4, v_target(1)*T);
          1.00];
else
    lb = [min(- v_target(1)*T/4, - v_target(1)*T);
          0.20 + min(v_target(2)*T/4, v_target(2)*T)];
    ub = [max(- v_target(1)*T/4, - v_target(1)*T);
          1.00];
end


addNodeConstraint(nlp, constraint_func, {'x'}, 'first', lb, ub, 'NonLinear');

lb = [-1.5;
    0.12];
ub = [+1.5;
    0.75];
addNodeConstraint(nlp, constraint_func, {'x'}, 2:nlp.NumNode, lb, ub, 'NonLinear');
end

