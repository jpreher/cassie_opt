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

if v_target(1) < 0
    lbx = -0.35 + 6.0*v_target(1);
    ubx = +0.35 - 1.5*v_target(1);
else
    lbx = -0.35 - 1.5*v_target(1);
    ubx = +0.35 + 6.0*v_target(1);
end

if v_target(2) < 0
    lby = -0.30 + 6.0*v_target(2);
    uby = +0.30 + 1.5*v_target(2);
else
    lby = -0.30 + 1.5*v_target(2);
    uby = +0.30 + 6.0*v_target(2);
end

addNodeConstraint(nlp, v_foot_fun, {'x', 'dx'}, 1:3:nlp.NumNode, ...
    [lbx, lby, -6.0], ...
    [ubx, uby,  6.0], ...
    'Nonlinear');

%%% End: "Impact velocity"
if v_target(1) > 0
    vxi_lb = -0.20 - 0.025 * v_target(1);
    vxi_ub = +0.05 + 0.10  * v_target(1);
elseif v_target(1) < 0
    vxi_lb = -0.05 + 0.10  * v_target(1);
    vxi_ub = +0.20 - 0.025 * v_target(1);
else
    vxi_lb = -0.05;
    vxi_ub = +0.05;
end
if v_target(2) > 0
    vyi_lb = -0.05 - 0.05 * v_target(2);
    vyi_ub = +0.05 + 0.075  * v_target(2);
elseif v_target(2) < 0
    vyi_lb = -0.05 + 0.075  * v_target(2);
    vyi_ub = +0.05 - 0.05 * v_target(2);
else
    vyi_lb = -0.05;
    vyi_ub = +0.05;
end

addNodeConstraint(nlp, v_foot_fun, {'x','dx'}, 'last', ...
    [vxi_lb, vyi_lb, -0.40], ...
    [vxi_ub, vyi_ub, -0.10],'Nonlinear');

end

