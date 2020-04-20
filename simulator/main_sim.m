%% Script: main_sim
gaitName = ['2018-04-24T15-32']; % gaitName = '2018-02-24T22-46';
logger = simCassie(behavior, 1, gaitName);

return;
%% plot the data
% Plot.plotData(behavior, logger, [], [], 'optsim', []);
% Plot.plotData_beta(behavior, logger, [], [], 'sim');
Plot.plotData(behavior, logger, [], [], 'sim');

%% simple animation
[gfc] = Plot.LoadAnimator(behavior, logger);

%% Save the simulation results for experiments  
% exportHardwareGait(behavior); %the old way
exportExp(behavior, gaitName);

return;
%% To run animation in RViz
% roslaunch cassie_model cassie_model_amber_rviz.launch
% roslaunch cassie_model cassie_model_rviz.launch
pause(2);
if startsWith(behavior.name,'run')
    animateRun_rviz(logger, 5);
else
    animate_rviz(logger, 7);
end