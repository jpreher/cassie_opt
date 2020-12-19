% Params
param = GaitParams();
initial_speed = [0.0, 0]; % [vx0, vy0];
load('20200927');
stance_leg = 'Right';
gait_param = param_lib(initial_speed(1), initial_speed(2), stance_leg, nested_lib);
final_aposition = gait_param.aposition;
x0 = gait_param.x0;
q = x0(1:22);
q(4) = 0.4;

dq = x0(23:end);

% Function and solver
ik = SingleSupportIKsolver();

% Set values
ik.leftStance = false;
ik.X = bezier(gait_param.aposition,0);
ik.dX = dbezier(gait_param.aposition,0)/0.4;

% Solve over range
tau = 0 : 0.01 : 1;
dtau = 1/0.4;
x  = zeros(10,length(tau));
dx = zeros(10,length(tau));
for i = 1:length(tau)
    ik.X  = bezier(gait_param.aposition,tau(i));
    ik.dX = dbezier(gait_param.aposition,tau(i))*dtau;
    [x(:,i), dx(:,i)] = ik.evaluate(q,dq);
end

figure();
plot(tau,x);

figure();
plot(tau,dx);