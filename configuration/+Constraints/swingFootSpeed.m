function swingFootSpeed(bounds, nlp, stanceFoot)

domain = nlp.Plant;
x = domain.States.x;
dx = domain.States.dx;
v_target = bounds.params.vd;

if strcmp(stanceFoot, 'Left')
    p_foot = getCartesianPosition(domain, domain.ContactPoints.RightSole);
    v_foot = jacobian(p_foot, x) * dx;
    v_foot_fun = SymFunction(['nsf_velocity_',domain.Name], v_foot, {x,dx});
else 
    p_foot = getCartesianPosition(domain, domain.ContactPoints.LeftSole);
    v_foot = jacobian(p_foot, x) * dx;
    v_foot_fun = SymFunction(['nsf_velocity_',domain.Name], v_foot, {x,dx});
end

addNodeConstraint(nlp, v_foot_fun, {'x', 'dx'}, 1:3:nlp.NumNode, ...
    [-3.5, -0.25 + 1*v_target(2), -2.0], ...
    [3.5, +0.25 + 3*v_target(2), 2.0], ...
    'Nonlinear');

end

