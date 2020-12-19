function [] = tauSS(nlp, isTime, singleDomain)

domain = nlp.Plant;
x = domain.States.x;
dx = domain.States.dx;

if isTime
    % tau boundary [0,~]
    T_name = nlp.OptVarTable.T(1).Name;
    T      = SymVariable(lower(T_name), [nlp.OptVarTable.T(1).Dimension,1]);
    p_name = nlp.OptVarTable.pposition(1).Name;
    p      = SymVariable(lower(p_name), [nlp.OptVarTable.pposition(1).Dimension, 1]);
    tau_0  = SymFunction(['tau_0_', domain.Name], (T(1)-p(2))/(p(1)-p(2)), {T,p});
    tau_F  = SymFunction(['tau_F_', domain.Name], (T(2)-p(2))/(p(1)-p(2)), {T,p});
    
    if singleDomain
        addNodeConstraint(nlp, tau_0, {T_name,p_name}, 'first', 0, 0, 'Linear');
        addNodeConstraint(nlp, tau_F, {T_name,p_name}, 'last',  1, 1, 'Linear');
    else
        addNodeConstraint(nlp, tau_0, {T_name,p_name}, 'first', 0, 1, 'Linear');
        addNodeConstraint(nlp, tau_F, {T_name,p_name}, 'last',  1, 1, 'Linear');
    end
    
else
    
    %     tau = domain.VirtualConstraints.velocity.PhaseFuncs{1};
    %     addNodeConstraint(nlp, tau, {'x','pposition'}, 'all',  0, 1, 'Nonlinear');
    %     addNodeConstraint(nlp, tau, {'x','pposition'}, 'last',  1, 1, 'Nonlinear');
    
    %     tauv = domain.VirtualConstraints.velocity.PhaseFuncs{1};
    %     addNodeConstraint(nlp, tauv, {'x','pvelocity'}, 'all',  0, 1, 'Nonlinear');
    %     addNodeConstraint(nlp, tauv, {'x','pvelocity'}, 'last',  1, 1, 'Nonlinear');
end

end

