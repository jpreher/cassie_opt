function [eigs] = eigen_value(hybrid_system, params)

x_plus = params{1}.x0;

x0 = x_plus(2:end);

J = jacobian(x0, hybrid_system);
eigs = abs(eig(J));

disp('The eigenvalues are:');
disp(eigs');
end

function [J] = jacobian(x, hybrid_system)
% computes the Jacobian of a function
n = length(x);
fprintf('Simulating nominal: ');
fx = fsl(x,hybrid_system);
eps=1.e-8; % could be made better
xperturb=x;
for i=1:n
    fprintf('Computing row %d of %d: ', i, n);
    xperturb(i)=x(i)+eps;
    % J(:,i)=(fsl(xperturb, hybrid_system, ref)-fx)/eps;
    J(:,i)=(fsl(xperturb, hybrid_system, ref)-fx)/eps;
    xperturb(i)=x(i);
end
end

function ret = fsl(x, hybrid_system)

x0 = [0;x];

tic
logger = hybrid_system.simulate(0, x0, [], [],'NumCycle', 1);
toc

guard = hybrid_system.Gamma.Edges.Guard{end};

% Compute reset map at the guard
xm = [logger(end).flow.states.x(:,end); logger(end).flow.states.dx(:,end)];
[~, x_f, ~]  = guard.calcDiscreteMap(0, xm);
    
ret = x_f(2:end);
end