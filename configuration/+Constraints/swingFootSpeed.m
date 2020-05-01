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

if v_target(2) < 0
    lby = -0.25 + 3.5*v_target(2);
    uby = +0.25 + 0.5*v_target(2);
else
    lby = -0.25 + 0.5*v_target(2);
    uby = +0.25 + 3.5*v_target(2);
end

addNodeConstraint(nlp, v_foot_fun, {'x', 'dx'}, 1:3:nlp.NumNode, ...
    [-3.5, lby, -2.0], ...
    [ 3.5, uby,  2.0], ...
    'Nonlinear');

end

