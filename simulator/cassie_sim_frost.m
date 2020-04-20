%% Script: simCassie
%
% Description: This is the main simulation function for running a FROST
%   based simulation. Simply provide a set of parameters and a behavior.
%
% Author: Jenna Reher, jreher@caltech.edu
%         Wenlong Ma, wma@caltech.edu
% ________________________________________

% function logger = simCassie(behavior, nSteps, gaitName)
function logger = cassie_sim_frost(behavior, nSteps, params)

if nargin<3
    gaitName = [];
    % params = loadGaitYAML(behavior);
end

if nargin < 2
    nSteps = 1;
end

%params = loadGaitYAML(behavior, gaitName);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Extract the parameters 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1 : height(behavior.hybridSystem.Gamma.Nodes)
    params{i}.kvelocity = 25;
    params{i}.kposition = [400, 60];
    params{i}.epsilon = 1e-6;
    behavior.hybridSystem = setVertexProperties(behavior.hybridSystem, ...
                            behavior.hybridSystem.Gamma.Nodes.Name{i}, ...
                            'Param', params{i});
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Run the simulator 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x0 = params{1}.x0;
behavior.hybridSystem.setOption('OdeSolver', @ode15s);

t0 = 0;
tf = 15;

tic
    logger = behavior.hybridSystem.simulate(t0, x0, tf, [],'NumCycle', nSteps);
toc

end