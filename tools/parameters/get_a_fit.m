%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% OBTAIN A_FIT FOR OPTIMIZATION INITIAL GUESS
%   Use nominal SLIP trajectories to get a_fit of a given domian, v 
%       Specific to the SLIP walker with pointmass feet.
% Author: Jake Reher
% Lab: AMBER Lab
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ a_0 ] = get_a_fit( t_v, M,  q_v )
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
options = optimset('Display', 'iter', 'MaxFunEvals', 200,'tolfun', ...
    tol, 'tolX', tolX, 'Algorithm', 'Levenberg-Marquardt');

% Fit h
[a_fit_h, ~, exitflag, ~, DP] = fsolve(@(a) abs(bezier_poly(tau, a) - q_h'), a0, options);

a_0 = a_fit_h';

end

