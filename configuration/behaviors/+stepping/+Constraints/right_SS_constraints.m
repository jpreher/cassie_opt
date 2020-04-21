function right_SS_constraints(nlp, bounds, varargin)
    
    domain = nlp.Plant;
    x = domain.States.x;
    dx = domain.States.dx;
    
    %% Relative degree 1 outputs
    %domain.VirtualConstraints.velocity.imposeNLPConstraint(nlp, bounds.velocity.ep, 0);
    
    %% Relative degree 2 outputs
    domain.VirtualConstraints.position.imposeNLPConstraint(nlp, [bounds.position.kp, bounds.position.kd], [1, 1]);
    
    %% tau boundary [0,1]
    T_name = nlp.OptVarTable.T(1).Name;
    T      = SymVariable(lower(T_name), [nlp.OptVarTable.T(1).Dimension,1]);
    p_name = nlp.OptVarTable.pposition(1).Name;
    p      = SymVariable(lower(p_name), [nlp.OptVarTable.pposition(1).Dimension, 1]);
    tau_0  = SymFunction(['tau_0_', domain.Name], (T(1)-p(2))/(p(1)-p(2)), {T,p});
    tau_F  = SymFunction(['tau_F_', domain.Name], (T(2)-p(2))/(p(1)-p(2)), {T,p});
    addNodeConstraint(nlp, tau_0, {T_name,p_name}, 'first', 0, 0, 'Linear');
    addNodeConstraint(nlp, tau_F, {T_name,p_name}, 'last',  1, 1, 'Linear');
    
    %     p_name = nlp.OptVarTable.pvelocity(1).Name;
    %     p      = SymVariable(lower(p_name), [nlp.OptVarTable.pvelocity(1).Dimension, 1]);
    %     tau_0  = SymFunction(['tauv_0_', domain.Name], (T(1)-p(2))/(p(1)-p(2)), {T,p});
    %     tau_F  = SymFunction(['tauv_F_', domain.Name], (T(2)-p(2))/(p(1)-p(2)), {T,p});
    %     addNodeConstraint(nlp, tau_0, {T_name,p_name}, 'first', 0, 0, 'Linear');
    %     addNodeConstraint(nlp, tau_F, {T_name,p_name}, 'last',  1, 1, 'Linear');
    
%     tau = domain.VirtualConstraints.position.PhaseFuncs{1};
%     addNodeConstraint(nlp, tau, {'x','pposition'}, 'all',  0, 1, 'Nonlinear');  
%     addNodeConstraint(nlp, tau, {'x','pposition'}, 'last',  1, 1, 'Nonlinear');  
    
%     tauv = domain.VirtualConstraints.position.PhaseFuncs{1};

%     addNodeConstraint(nlp, tauv, {'x','pvelocity'}, 'all',  0, 1, 'Nonlinear'); 
%     addNodeConstraint(nlp, tauv, {'x','pvelocity'}, 'last',  1, 1, 'Nonlinear'); 
    
    %% Floating base velocity
    v_target = bounds.params.vd;

    dof = domain.numState;
    T0  = SymVariable('t0',  [2, 1]);
    TF  = SymVariable('tf',  [2, 1]);
    X0  = SymVariable('x0', [dof,1]);
    XF  = SymVariable('xF', [dof,1]);
    avg_vel = (XF(1:2,1) - X0(1:2,1)) / (TF(2) - T0(1));
    avg_vel_fun = SymFunction('average_velocity', avg_vel, {T0, TF, X0, XF});

    avg_vel_cstr = NlpFunction('Name','average_velocity',...
        'Dimension',2,...
        'lb', v_target,...
        'ub', v_target,...
        'Type','Nonlinear',...
        'SymFun', avg_vel_fun,...
        'DepVariables', [nlp.OptVarTable.T(1); ...
                         nlp.OptVarTable.T(end); ...
                         nlp.OptVarTable.x(1); ...
                         nlp.OptVarTable.x(end)]);
    addConstraint(nlp, 'average_velocity', 'last', avg_vel_cstr);
    
%     avg_vel = SymExpression([dx(1); dx(2)]);
%     avg_vel_func = SymFunction('floating_velocity', avg_vel, dx);
%     addIntegralConstraint(nlp, avg_vel_func, {'dx'}, ...
%         v_target, v_target,'Nonlinear');
    
    %% nsf pitch
    nsf_orient = domain.getCartesianPosition(domain.ContactPoints.LeftToe) - domain.getCartesianPosition(domain.ContactPoints.LeftHeel);
    pitch_constr_fun = SymFunction('nsf_pitch', nsf_orient(3), x);
    addNodeConstraint(nlp, pitch_constr_fun, {'x'}, 'all', 0, 0, 'Nonlinear');
    
    %% nsf yaw
    yaw_constr_fun = SymFunction('nsf_yaw', nsf_orient(2), x);
    addNodeConstraint(nlp, yaw_constr_fun, {'x'}, 'all', 0, 0, 'Nonlinear');
        
    %% nsf velocity
    p_left_foot = getCartesianPosition(domain, domain.ContactPoints.LeftSole);
    v_left_foot = jacobian(p_left_foot, x) * dx;
    v_left_foot_fun = SymFunction(['nsf_velocity_',domain.Name], v_left_foot, {x,dx});
    
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
    addNodeConstraint(nlp, v_left_foot_fun, {'x','dx'}, 'last', ...
        [vxi_lb, vyi_lb, -0.20], ...
        [vxi_ub, vyi_ub, -0.01],'Nonlinear');

    
	%%% NSF-X shoundn't swing too fast
    nsf_vel_x_fun = SymFunction('nsf_velocity_x', v_left_foot(1), {x, dx});
    addNodeConstraint(nlp, nsf_vel_x_fun, {'x', 'dx'}, 1:3:nlp.NumNode, -3.5, 3.5, 'Nonlinear');
    
    %%% NSF-Y shoundn't swing too fast
    nsf_vel_y_fun = SymFunction('nsf_velocity_y', v_left_foot(2), {x, dx});
    addNodeConstraint(nlp, nsf_vel_y_fun, {'x', 'dx'}, 1:3:nlp.NumNode, ...
        min(-0.25 + 1*v_target(2), +0.25 + 3*v_target(2)), ...
        +0.25 + 3*v_target(2), 'Nonlinear');
    
    %%% NSF-Z shoundn't swing too fast
    nsf_vel_z_fun = SymFunction('nsf_velocity_z', v_left_foot(3), {x, dx});
    addNodeConstraint(nlp, nsf_vel_z_fun, {'x', 'dx'}, 1:3:nlp.NumNode, -2.5, 2.5, 'Nonlinear');
    
    %% nsf clearance
    nsf_height = SymFunction(['nsf_height_', domain.Name], nlp.Plant.EventFuncs.nsf.ConstrExpr, {x});
    addNodeConstraint(nlp, nsf_height, {'x'}, floor(nlp.NumNode/4),   0.04, inf,'Nonlinear');
    addNodeConstraint(nlp, nsf_height, {'x'}, floor(nlp.NumNode/2),   0.14, inf,'Nonlinear');
    addNodeConstraint(nlp, nsf_height, {'x'}, floor(3*nlp.NumNode/4), 0.04, inf,'Nonlinear');
    
    %% step width
    T = nlp.OptVarTable.T(1).UpperBound(2);
    right_foot = domain.getCartesianPosition(domain.ContactPoints.RightSole);  
    left_foot  = domain.getCartesianPosition(domain.ContactPoints.LeftSole);  
    constraint = tomatrix(left_foot(1:2) - right_foot(1:2));
    
    constraint_func = SymFunction(['step_distance_',domain.Name], constraint, {x});
    lb = [min(- v_target(1)*T/4, - v_target(1)*T);
          min(0.18 - v_target(2)*T/4, 0.18 - v_target(2)*T)];
    ub = [max(- v_target(1)*T/4, - v_target(1)*T);
          max(0.25 - v_target(2)*T/4, 0.25 - v_target(2)*T)];
    addNodeConstraint(nlp, constraint_func, {'x'}, 'first', lb, ub, 'NonLinear');
    
    lb = [-1.5;
          0.22];
    ub = [+1.5;
          0.35];
    addNodeConstraint(nlp, constraint_func, {'x'}, 2:nlp.NumNode, lb, ub, 'NonLinear');
            
end