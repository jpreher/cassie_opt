function walkingSpeed(bounds, nlp)

domain = nlp.Plant;
x = domain.States.x;
dx = domain.States.dx;
v_target = bounds.params.vd;
dof = domain.numState;

T0  = SymVariable('t0',  [2, 1]);
TF  = SymVariable('tf',  [2, 1]);
X0  = SymVariable('x0', [dof,1]);
XF  = SymVariable('xF', [dof,1]);

avg_vel = (XF(1:2,1) - X0(1:2,1)) / (TF(2) - T0(1));
avg_vel_fun = SymFunction(['average_velocity_',domain.Name], avg_vel, {T0, TF, X0, XF});

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


end

