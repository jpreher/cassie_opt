function impactVelocity(bounds, nlp, stanceFoot)

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

%%% End: "Impact velocity"
if v_target(1) > 0
    vxi_lb = -0.20;
    vxi_ub = +0.05;
elseif v_target(1) < 0
    vxi_lb = +0.05;
    vxi_ub = +0.20;
else
    vxi_lb = -0.05;
    vxi_ub = +0.05;
end
if v_target(2) > 0
    vyi_lb = -0.15;
    vyi_ub = +0.05;
elseif v_target(2) < 0
    vyi_lb = -0.05;
    vyi_ub = +0.15;
else
    vyi_lb = -0.05;
    vyi_ub = +0.05;
end
addNodeConstraint(nlp, v_foot_fun, {'x','dx'}, 'last', ...
    [vxi_lb, vyi_lb, -0.30], ...
    [vxi_ub, vyi_ub, -0.05],'Nonlinear');


end

