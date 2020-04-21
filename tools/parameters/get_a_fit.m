%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% OBTAIN A_FIT FOR OPTIMIZATION INITIAL GUESS
%   Use nominal SLIP trajectories to get a_fit of a given domian, v 
% Author: Jenna Reher
% Lab: AMBER Lab
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ a_0 ] = get_a_fit( t_v, M,  q_v , do_plot)
if nargin < 4
    do_plot = false;
end

N = 1;      % Number of controlling outputs
a_0 = zeros(N+1, M+1);

% Extract the time.
t0 = t_v(1);
tf = t_v(end);
tau = (t_v - t0) / (tf - t0);
a_0(1,1) = tf - t0;

% Extract the angle information
q_h = q_v;

% Solver parameters
a0 = ones(M+1, 1);
tol = 1e-4;
tolX = 1e-4;
options = optimset('Display', 'none', 'MaxFunEvals', 200,'tolfun', ...
    tol, 'tolX', tolX, 'Algorithm', 'Levenberg-Marquardt');

% Fit h
[a_fit_h, ~, exitflag, ~, DP] = fsolve(@(a) abs(bezier_poly(tau, a) - q_h'), a0, options);

a_0 = a_fit_h';

if do_plot
    s = 0 : 0.01 : 1;
    for i = 1:length(s)
        b(i) = bezier(a_0,s(i));
    end
    figure(10);
    plot(s, b, '--');
    hold on;
    plot(tau, q_v);
    xlabel('tau');
    ylabel('fit');
    
end

end

