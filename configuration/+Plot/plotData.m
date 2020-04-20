%% Function: plotData
% 
% Description:
%   Plot data for simulation & optimization & experiments.
%   This is designed particularly for less-than-2-domain behaviors
%
%   Note: plotting is a very case-by-case functionality, a detailed
%         customization may be necessary for your application.
%
% Known issues:
%   - 'Holonomic constraints' setup must be the same order for both doamins
%   - Plotting option 'optsim' does not work for assymetric behaviors.
%   - Have to re-run 'genPlotCode(behavior)' everytime you change the model.
%   
%
% Color representations in the plots:
%       Red color is for         "optimization data"
%       Blue color is for        "simulation data"
%       magenta-dash line is for "desired data"
%       Black-dash line is for   "boundary data"
%
%       green-solid:  "left foot"
%       green-dash:   "right foot"
%
%
% Author: Wenlong Ma, wma@caltech.edu
%         Jenna Reher, jreher@caltech.edu
% _________________________________________________________________________
function [] = plotData( behavior, logger, nlp, sol, plotType, params)

if nargin < 3
    nlp = [];
end
if nargin < 4
    sol = [];
end
if nargin < 5
    plotType = 'sim'; %'opt' 'sim' 'optsim' 'exp'
    params = [];
end

% Additional plotting functions
if ispc
    baseDir = pwd;
else
    baseDir = ros_resolve_local_url('package://cassie_opt/matlab/');
end

export_path = strcat(baseDir, '/export/cassie_plot');
addpath(export_path);
if ~exist(export_path,'dir')
    genPlotCode(behavior);
end

%% Load Optimization results, whether to show opt results together with sim results
if strcmp(plotType, 'optsim')
    listing = dir(['params/', behavior.name]);
    gaitName = listing(end).name; %gaitName = '2018-02-02T10-22';
    load(['params/', behavior.name, '/', gaitName, '/optData.mat']);
    logOpt = data.logger;
    params = loadGaitYAML(behavior, gaitName);
end

%% Collect plotting comparisons
model_bounds = behavior.robotModel.getLimits();
nDomain = behavior.hybridSystem.Gamma.numnodes;
if strcmp(plotType, 'opt')
    nStep = 1;
else
    nStep = ceil(length(logger)/nDomain);
end
nVertex = nDomain * nStep;

% Color Map
N = 20; 
X = linspace(0,pi*3,1000); 
Y = bsxfun(@(x,n)sin(x+2*n*pi/N), X.', 1:N); 
C = linspecer(N);

if strcmp(plotType, 'opt')
    mColor = 'r';
else
    mColor = 'b';
end

com_data = cell(nVertex, 1);
spIndex = logger(1).plant.getJointIndices({'LeftShinPitch',...
           'LeftTarsusPitch', 'RightShinPitch', 'RightTarsusPitch'});
             
% Main plotting function
for k = 1:nVertex
    
    iDomain = mod(k + (nDomain-1), nDomain) + 1;    
    domain = behavior.hybridSystem.Gamma.Nodes.Domain{iDomain};
    
    %% Data Processing
    t   = logger(k).flow.t;
    x   = logger(k).flow.states.x;
    dx  = logger(k).flow.states.dx;
    ddx = logger(k).flow.states.ddx;
    u   = logger(k).flow.inputs.Control.u;
    if isfield(logger(k).flow.inputs, 'ConstraintWrench')
        wrenches = logger(k).flow.inputs.ConstraintWrench;
    else
        wrenches = [];
    end

%     % spring forces & position
%     Fs = zeros(4, length(t));
%     for i = 1:length(t)
%         tmp = springForce_plot(x(:,i),dx(:,i));
%         Fs(:,i) = tmp(spIndex);
%     end
    % Compute $COM$ data
    com = zeros(6, length(t));
    for i = 1:length(t)
        %com_pos = arrayfun(@(var)com_position(var(:,1)), x);
        com_pos = com_position(x(:,i));
        com_vel = com_velocity(x(:,i), dx(:,i));
        com(:,i) = [com_pos'; com_vel];
    end
    com_data{k} = com;
    
    % Compute $feet$ data
    clear RF_pos LF_pos RF_vel LF_vel
    for i = 1:length(t)
        RF_pos(:,i) = RightMidFoot_pos(x(:,i) );
        LF_pos(:,i) = LeftMidFoot_pos( x(:,i) );
        RF_vel(:,i) = RightMidFoot_vel(x(:,i), dx(:,i) );
        LF_vel(:,i) = LeftMidFoot_vel( x(:,i), dx(:,i) );
    end

    % Compute $slipOutputs$ data
    % clear slip_yd;
    % for i = 1:length(t)
    %     tmp1 = slipOutputs_Right(x(:,i));
    %     tmp2 = slipOutputs_Left(x(:,i));
    %     slip_yd(:,i) = [tmp1; tmp2]; 
    % end
    
    %%%%%%% Only plotting for sim %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if strcmp(plotType, 'sim')
        tau = logger(k).flow.tau_position;
        ya  = logger(k).flow.ya_position;
        yd  = logger(k).flow.yd_position; 
    end
    
    %%%%%%% Only plotting for opt %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if strcmp(plotType, 'opt')
        % Recover $tau$
        vc = domain.VirtualConstraints.position;
        tau = zeros(1,length(t));
        for i = 1:length(t)
            temp = vc.calcPhaseVariable(t(i), x(:,i), dx(:,i), params{iDomain}.pposition);
            tau(i) = temp{1};
        end
        
        % Recover $ya$ $yd$
        ya = [];
        yd = [];
        for i = 1:length(t)
            yaTemp = vc.calcActual(x(:,i), dx(:,i));
            ydTemp = vc.calcDesired(t(i), x(:,i), dx(:,i), params{iDomain}.aposition, params{iDomain}.pposition);
            ya(:,i) = yaTemp{1};
            yd(:,i) = yaTemp{1};
        end
    end
    
    %%%%%%% Plotting for opt vs. sim %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if strcmp(plotType, 'optsim')
        tOpt   = logOpt(k).flow.t;
        xOpt   = logOpt(k).flow.states.x;
        dxOpt  = logOpt(k).flow.states.dx;
        ddxOpt = logOpt(k).flow.states.ddx;
        uOpt   = logOpt(k).flow.inputs.Control.u;
        if isfield(logOpt(k).flow.inputs, 'ConstraintWrench')
            wrenchesOpt = logOpt(k).flow.inputs.ConstraintWrench;
        else
            wrenchesOpt = [];
        end
        
        % Compute COM vel data
        com_vel_opt =  zeros(3, length(tOpt));
        for i = 1:length(tOpt)
            %com_pos_opt = com_position(x(:,i));
            %com_vel_opt = com_velocity(xOpt(:,i), dxOpt(:,i));
            %comOpt(:,i) = [com_pos_opt'; com_vel_opt];
            com_vel_opt(:,i) = com_velocity(xOpt(:,i), dxOpt(:,i));
        end

        % Recover $tau$
        vc = domain.VirtualConstraints.position;
        tauOpt = zeros(1,length(tOpt));
        for i = 1:length(tOpt)
            temp = vc.calcPhaseVariable(tOpt(i), xOpt(:,i),dxOpt(:,i), params{iDomain}.pposition);
            tauOpt(i) = temp{1};
        end
        
        % Recover $ya$ $yd$
        yaOpt = [];
        ydOpt = [];
        for i = 1:length(tOpt)
            yaTemp = vc.calcActual(xOpt(:,i), dxOpt(:,i));
            ydTemp = vc.calcDesired(tOpt(i), xOpt(:,i), dxOpt(:,i), params{iDomain}.aposition, params{iDomain}.pposition);
            yaOpt(:,i) = yaTemp{1};
            ydOpt(:,i) = yaTemp{1};
        end
        
        % Recover feet data from opt
        clear RF_pos_opt LF_pos_opt RF_vel_opt LF_vel_opt
        for i = 1:length(tOpt)
            RF_pos_opt(:,i) = RightMidFoot_pos(xOpt(:,i) );
            LF_pos_opt(:,i) = LeftMidFoot_pos( xOpt(:,i) );
            RF_vel_opt(:,i) = RightMidFoot_vel(xOpt(:,i), dxOpt(:,i) );
            LF_vel_opt(:,i) = LeftMidFoot_vel( xOpt(:,i), dxOpt(:,i) );
        end
        
        tau = logger(k).flow.tau_position;
        ya  = logger(k).flow.ya_position;
        yd  = logger(k).flow.yd_position; 
        
    end
    
    %% Tau
    h = figure(100); 
        if k==1 
            clf; 
        end
    h.Name = 'tau'; h.WindowStyle = 'docked'; 
    
    %subplot(1,2,1)
    plot(t, tau, 'Color', mColor); hold on; grid on; title 'Tau';
    plot(t, tau, 'o', 'Color', mColor);
    xlabel 'Time (s)'; ylabel 'tau'; ylim([-0.1, 1.1]);
    
    if strcmp(plotType, 'optsim')
        plot(tOpt, tauOpt, 'r');
    end
    
    %subplot(1,2,2)
    %plot(t, 'Color', mColor); hold on; grid on;
    %plot(t, 'o'); hold on; grid on; 
    %xlabel 'node'; ylabel 'time'; title 'time';
    
    %% tmp
%     h = figure(3211); 
%         if k==1 
%             clf; 
%         end
%     h.Name = 'shit'; h.WindowStyle = 'docked';
%     subplot(2,1,1)
%         plot(t, x(2,:), 'Color', 'r'); hold on; %title('basePos Y');
%         plot(t, com(2,:), 'Color', 'b'); hold on; %title('comPos Y');
%         plot(t, LF_pos(2,:), 'Color', 'g'); hold on; %title('nsfPos Y');
%         legend('basePosY','comPosY','nsfPosY');
%     subplot(2,1,2)
%         plot(t, x(4,:), 'Color', 'r'); hold on; %title('basePos Y');
%         plot(t, com(5,:), 'Color', 'b'); hold on; %title('comVel Y');
%         plot(t, LF_vel(2,:), 'Color', 'g'); hold on; %title('nsfPos Y');
%         legend('baseVelY','comVelY','nsfVelY');
    
    %% States
    jointName = {behavior.robotModel.Joints.Name};
    iBase     = 1:6;
    iLeftLeg  = find(strncmpi(jointName,'Left', 4));
    iRightLeg = find(strncmpi(jointName,'Right',5));
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% base position %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %----------------------------------------------------------------------
    % Floating base
    h = figure(1011); 
        if k==1 
            clf; 
        end
    h.Name = 'b_pos'; h.WindowStyle = 'docked';
    hold on; grid on;
    
    for i = 1:6
        j = iBase(i);
        subplot(2,3,i); hold on; grid on;
        
        plot(t, x(j,:), 'Color', mColor);
        if strcmp(plotType, 'optsim')
            plot(tOpt, xOpt(j,:), 'r');
        end
        
        % Bounds
        lb = model_bounds.states.x.lb(j);
        ub = model_bounds.states.x.ub(j);
        plot([t(1), t(end)], [lb, lb], 'k--');
        plot([t(1), t(end)], [ub, ub], 'k--');
        
        % Label
        title(behavior.robotModel.States.x.label{j});
        xlabel('Time(s)'); ylabel('q (rad)');
    end
    
    %%% base velocity 
    h = figure(1012); 
        if k==1 
            clf; 
        end
    h.Name = 'b_vel'; h.WindowStyle = 'docked';
    hold on; grid on;
    
    for i = 1:6
        j = iBase(i);
        subplot(2,3,i); hold on; grid on;
        
        plot(t, dx(j,:), 'Color', mColor);
        if strcmp(plotType, 'optsim')
            plot(tOpt, dxOpt(j,:), 'r');
        end
        
        % Bounds
        lb = model_bounds.states.dx.lb(j);
        ub = model_bounds.states.dx.ub(j);
        plot([t(1), t(end)], [lb, lb], 'k--');
        plot([t(1), t(end)], [ub, ub], 'k--');
        
        % Label
        title(behavior.robotModel.States.dx.label{j});
        xlabel('Time(s)'); ylabel('dq (rad/s)');
    end
    
    %%% base accleration 
    h = figure(1013); 
        if k==1 
            clf; 
        end
    h.Name = 'b_acc'; h.WindowStyle = 'docked'; hold on; grid on;
    
    for i = 1:6
        j = iBase(i);
        subplot(2,3,i); hold on; grid on;
        
        plot(t, ddx(j,:), 'Color', mColor);
        if strcmp(plotType, 'optsim')
            plot(tOpt, ddxOpt(j,:), 'r');
        end
        
        % Label
        title(behavior.robotModel.States.ddx.label{j});
        xlabel('Time(s)'); ylabel('ddq (rad/s^2)');
    end
    
    h = figure(1014); 
        if k==1 
            clf; 
        end
    h.Name = 'b_pp'; h.WindowStyle = 'docked'; hold on; grid on;   
    for i = 1:6
        j = iBase(i);
        subplot(2,3,i); hold on; grid on;
        plot(x(j,:), dx(j,:), 'Color', mColor);
        if strcmp(plotType, 'optsim')
            plot(xOpt, dxOpt(j,:), 'r');
        end
        title(behavior.robotModel.States.ddx.label{j});
        xlabel('pos'); ylabel('vel (rad/s^2)');
    end
    
    h = figure(1015); 
        if k==1 
            clf; 
        end
    h.Name = 'b_cart'; h.WindowStyle = 'docked'; hold on; grid on;   
    subplot(1,2,1)
        %- BasePos
        plot(x(2,:), x(1,:), 'r', 'LineWidth', 1.5); hold on; grid on
        xlabel('y pos'); ylabel('x pos');
        set(gca, 'XDir','reverse');
    subplot(1,2,2)
        % com
        plot(com(2,:), com(1,:), '-.', 'LineWidth', 1.5, 'Color', 'r'); grid on
        xlabel('comPosY'); ylabel('comPosX');
        set(gca, 'XDir','reverse');
    %----------------------------------------------------------------------
    %%% base position %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Left Leg %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %----------------------------------------------------------------------
    h = figure(1021); 
        if k==1 
            clf; 
        end
    h.Name = 'll_pos'; h.WindowStyle = 'docked'; hold on; grid on;
    
    %%% left leg position
    n = numel(iLeftLeg);
    nrows = 2;
    ncols = ceil(n/nrows);
    
    for i = 1:numel(iLeftLeg)
        j = iLeftLeg(i);
        subplot(nrows, ncols, i); hold on; grid on;
        
        plot(t, x(j,:), 'Color', mColor);
        if strcmp(plotType, 'optsim')
            plot(tOpt, xOpt(j,:), 'r');
        end
        
        % Bounds
        lb = model_bounds.states.x.lb(j);
        ub = model_bounds.states.x.ub(j);
        plot([t(1), t(end)], [lb, lb], 'k--');
        plot([t(1), t(end)], [ub, ub], 'k--');
        
        % Label
        title(behavior.robotModel.States.x.label{j});
        xlabel('Time(s)'); ylabel('q (rad)');
    end
    
    %%% left leg velocity
    h = figure(1022); 
        if k==1 
            clf; 
        end
    h.Name = 'll_vel'; h.WindowStyle = 'docked';
    hold on; grid on;
    
    for i = 1:numel(iLeftLeg)
        j = iLeftLeg(i);
        subplot(nrows, ncols, i); hold on; grid on;
        
        plot(t, dx(j,:), 'Color', mColor);
        if strcmp(plotType, 'optsim')
            plot(tOpt, dxOpt(j,:), 'r');
        end
        
        % Bounds
        lb = model_bounds.states.dx.lb(j);
        ub = model_bounds.states.dx.ub(j);
        plot([t(1), t(end)], [lb, lb], 'k--');
        plot([t(1), t(end)], [ub, ub], 'k--');
        
        % Label
        title(behavior.robotModel.States.dx.label{j});
        xlabel('Time(s)');
        ylabel('dq (rad/s)');
    end
    
    %%% left leg acceleration
    h = figure(1023); 
        if k==1 
            clf; 
        end
    h.Name = 'll_acc'; h.WindowStyle = 'docked';
    hold on; grid on;
    
    for i = 1:numel(iLeftLeg)
        j = iLeftLeg(i);
        subplot(nrows, ncols, i); hold on; grid on;
        
        plot(t, ddx(j,:), 'Color', mColor);
        if strcmp(plotType, 'optsim')
            plot(tOpt, ddxOpt(j,:), 'r');
        end
        
        % Label
        title(behavior.robotModel.States.ddx.label{j});
        xlabel('Time(s)');
        ylabel('ddq (rad/s^2)');
    end
    %----------------------------------------------------------------------
    %%% Left Leg %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Right Leg %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %----------------------------------------------------------------------
    h = figure(1031); 
        if k==1 
            clf; 
        end
    h.Name = 'rl_pos'; h.WindowStyle = 'docked';
    hold on; grid on;
    
    % right leg position
    n = numel(iRightLeg);
    nrows = 2;
    ncols = ceil(n/nrows);
    
    for i = 1:numel(iRightLeg)
        j = iRightLeg(i);
        subplot(nrows, ncols, i); hold on; grid on;
        
        plot(t, x(j,:), 'Color', mColor);
        if strcmp(plotType, 'optsim')
            plot(tOpt, xOpt(j,:), 'r');
        end
        
        % Bounds
        lb = model_bounds.states.x.lb(j);
        ub = model_bounds.states.x.ub(j);
        plot([t(1), t(end)], [lb, lb], 'k--');
        plot([t(1), t(end)], [ub, ub], 'k--');
        
        % Label
        title(behavior.robotModel.States.x.label{j});
        xlabel('Time(s)'); ylabel('q (rad)');
    end
    
    % right leg velocity
    h = figure(1032); 
        if k==1 
            clf; 
        end
    h.Name = 'rl_vel'; h.WindowStyle = 'docked';
    hold on; grid on;
    
    for i = 1:numel(iRightLeg)
        j = iRightLeg(i);
        subplot(nrows, ncols, i); hold on; grid on;
        
        plot(t, dx(j,:), 'Color', mColor);
        if strcmp(plotType, 'optsim')
            plot(tOpt, dxOpt(j,:), 'r');
        end
        
        % Bounds
        lb = model_bounds.states.dx.lb(j);
        ub = model_bounds.states.dx.ub(j);
        plot([t(1), t(end)], [lb, lb], 'k--');
        plot([t(1), t(end)], [ub, ub], 'k--');
        
        % Label
        title(behavior.robotModel.States.dx.label{j});
        xlabel('Time(s)'); ylabel('dq (rad/s)');
    end
    
    % right leg acceleration
    h = figure(1033);
        if k==1 
            clf; 
        end
    h.Name = 'rl_acc'; h.WindowStyle = 'docked';
    hold on; grid on;
    
    for i = 1:numel(iRightLeg)
        j = iRightLeg(i);
        subplot(nrows, ncols, i); hold on; grid on;
        
        plot(t, ddx(j,:), 'Color', mColor);
        if strcmp(plotType, 'optsim')
            plot(tOpt, ddxOpt(j,:), 'r');
        end
        
        % Label
        title(behavior.robotModel.States.ddx.label{j});
        xlabel('Time(s)'); ylabel('ddq (rad/s^2)');
    end
    %----------------------------------------------------------------------
    %%% Right Leg %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %% Phase potrait, toBeAdded
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    %% COM position & velocity
    h = figure(120); 
        if k==1 
            clf; 
        end
    h.Name = 'COM'; h.WindowStyle = 'docked';
    subplot(3,2,1)
        plot(t, com(1,:), 'LineWidth', 2, 'Color', mColor); grid; hold on;
        title('comPos X'); %ylim([.6,1.2]);
    subplot(3,2,2)
        plot(t, com(4,:), 'LineWidth', 2, 'Color', mColor); grid; hold on;
        if strcmp(plotType, 'optsim')
            plot(tOpt, com_vel_opt(1,:), 'Color', 'r', 'LineWidth', 1);
        end
        title('comVel X'); %ylim([-2,2]);
    subplot(3,2,3)
        plot(t, com(2,:), 'LineWidth', 2, 'Color', mColor); grid; hold on;
        title('comPos Y'); %ylim([.6,1.2]);
    subplot(3,2,4)
        plot(t, com(5,:), 'LineWidth', 2, 'Color', mColor); grid; hold on;
        if strcmp(plotType, 'optsim')
            plot(tOpt, com_vel_opt(2,:), 'Color', 'r', 'LineWidth', 1);
        end
        title('comVel Y'); %ylim([-2,2]);
    subplot(3,2,5)
        plot(t, com(3,:), 'LineWidth', 2, 'Color', mColor); grid; hold on;
        title('comPos Z'); %ylim([.8,1.2]);
    subplot(3,2,6)
        plot(t, com(6,:), 'LineWidth', 2, 'Color', mColor); grid; hold on;
        if strcmp(plotType, 'optsim')
            plot(tOpt, com_vel_opt(3,:), 'Color', 'r', 'LineWidth', 1);
        end
        title('comVel Z'); %ylim([-2,2]);
    
    h = figure(121); 
    if(k==1) clf; end
        h.Name = 'COM_pp'; h.WindowStyle = 'docked';
        subplot(3,1,1)
            plot(com(1,:), com(4,:), 'LineWidth', 2, 'Color', mColor); grid; hold on;
            xlabel('comPos X'); ylabel('comVel X');
        subplot(3,1,2)
            plot(com(2,:), com(5,:), 'LineWidth', 2, 'Color', mColor); grid; hold on;
            xlabel('comPos Y'); ylabel('comVel Y');
        subplot(3,1,3)
            plot(com(3,:), com(6,:), 'LineWidth', 2, 'Color', mColor); grid; hold on;
            xlabel('comPos Z'); ylabel('comVel Z');
    
    h = figure(122); 
        if k==1 
            clf; 
        end
	h.Name = 'COM vs Base'; h.WindowStyle = 'docked';
    subplot(3,2,1)
    plot(t, com(1,:)-x(1,:)); hold on; title('com-x - basePosX')
    subplot(3,2,3)
    plot(t, com(2,:)-x(2,:)); hold on; title('com-y - basePosY')
    subplot(3,2,5)
    plot(t, com(3,:)-x(3,:)); hold on; title('com-z - basePosZ')
    subplot(3,2,2)
    plot(t, com(4,:)-x(4,:)); hold on; title('com-x - baseVelX')
    subplot(3,2,4)
    plot(t, com(5,:)-x(5,:)); hold on; title('com-y - baseVelY')
    subplot(3,2,6)
    plot(t, com(6,:)-x(6,:)); hold on; title('com-z - baseVelZ')
    
    
    %% Joint torques (motor-end torque)
    h = figure(130); 
    if(k==1) clf; end
    h.Name = 'u'; h.WindowStyle = 'docked';
    hold on; grid on;
    
    %Nu = 10; %length(u(:,1));
    %nrows = 2;
    %ncols = 5; %ceil(Nu/nrows);
    
    % This is hard-coded, need to fix.
    % Always verify this in 'setupOpt.m'
    if endsWith(logger(k).plant.Name,'SS')
        u_index = 1:9;
    elseif endsWith(logger(k).plant.Name,'FL')
        u_index = 1:10;
	elseif endsWith(logger(k).plant.Name,'DS')
        u_index = [ 1:4, 6:10 ];
    else
        disp('problem foo');
    end
    
    for i = 1:size(u_index,1)
        subplot(2, 5, u_index(i)); 
        plot(t, u(u_index(i),:), 'Color', mColor); hold on; grid on;
        
        if strcmp(plotType, 'optsim')
            plot(tOpt, uOpt(u_index(i),:), 'Color', mColor, 'LineStyle','--');
        end
        
        % Bounds
        lb = model_bounds.inputs.Control.u.lb(u_index(i));
        ub = model_bounds.inputs.Control.u.ub(u_index(i));
        plot([t(1), t(end)], [lb, lb], 'k--'); hold on;
        plot([t(1), t(end)], [ub, ub], 'k--');
        
        % Label
        xlabel('Time(s)'); ylabel('u (Nm)'); title(i);
    end
    
%     %% springs
%     h = figure(131); 
%     if(k==1) clf; end
%     h.Name = 'springF'; h.WindowStyle = 'docked';
%     
%     subplot(2,2,1)
%     plot(t, Fs(1,:), '-', 'Color', mColor); hold on; grid on;
%     title 'LeftShinPitch'; xlabel 'time'; ylabel 'force';
%     subplot(2,2,3)
%     plot(t, Fs(2,:), '--', 'Color', mColor); hold on; grid on;
%     title 'LeftHeelSpring'; xlabel 'time'; ylabel 'force';
%     subplot(2,2,2)
%     plot(t, Fs(3,:), '-', 'Color', mColor); hold on; grid on;
%     title 'RightShinPitch'; xlabel 'time'; ylabel 'force';
%     subplot(2,2,4)
%     plot(t, Fs(4,:), '--', 'Color', mColor); hold on; grid on;
%     title 'RightHeelSpring'; xlabel 'time'; ylabel 'force';
%     
    
    %% Outputs
    % this is hard coded, need to improve
    h = figure(132); 
    if(k==1) clf; end
    h.Name = 'ya_yd'; h.WindowStyle = 'docked';

    % This is hard-coded, need to fix.
    % Always verify this
    if endsWith(logger(k).plant.Name,'SS')
        indexY = 1:5;
    elseif endsWith(logger(k).plant.Name,'FL')
        indexY = 1:10; %fix this
    elseif endsWith(logger(k).plant.Name,'DS')
        indexY = 1:6; %[ 1 2 3 4 6 7];
    else
        disp('problem foo');
    end

    for iy = 1:length(indexY)
        subplot(2,5,indexY(iy))
        plot(t, ya(iy,:)); hold on; grid on;
        plot(t, yd(iy,:), 'b--'); hold on;
        title(logger(k).plant.VirtualConstraints.position.OutputLabel(iy));
    end

    % output errors
    h = figure(133); 
    if(k==1) clf; end
    h.Name = 'y_error'; h.WindowStyle = 'docked';
    for iy = 1:length(indexY)
        subplot(2,5,indexY(iy))
        plot(t, ya(iy,:)-yd(iy,:)); hold on; grid on;
        title(logger(k).plant.VirtualConstraints.position.OutputLabel(iy));
    end

    
%     %% ROM related subjects
%     if strcmp(domain.Name, 'RightSS')
%        ur = zeros(3, length(t));
%        for i = 1:length(t)
%            ur(:, i) = ur_t_RightSS(x(:,i), dx(:,i),...
%                          logger(k).static.params.pposition,...
%                          logger(k).static.params.aposition,...
%                          u(:,i),...
%                          t(i),...
%                          logger(k).static.params.prom);
%        end
%        h = figure(134);
%        if(k==2) clf; end
%        h.Name = 'u ROM'; h.WindowStyle = 'docked';
%        subplot(2,2,1)
%            plot(t, ur(1,:),  '-r'); hold on
%            plot(t,  u(1,:),  '-b', 'LineWidth', 2); hold on
%            legend('rom pitch', 'full pitch');
%        subplot(2,2,2)
%            plot(t, ur(2,:),  'r'); hold on
%            plot(t,  u(2,:),  'b', 'LineWidth', 2); hold on
%            legend('rom roll', 'full roll');
%         subplot(2,2,[3:4])
%            plot(t, ur(3,:),  'r'); hold on
%            plot(t,  u(3,:),  'b', 'LineWidth', 2); hold on
%            legend('rom force', 'full torque');
%            title('leg length');
%     end
%     
%     %%% ROM control disturbance
%     if strcmp(domain.Name, 'RightSS')
%        u_dist = zeros(1, length(t));
%        for i = 1:length(t)
%            %%% u_dist_norm_RightSS_interval?
%            u_dist(:, i) = u_dist_norm_RightSS(x(:,i), dx(:,i),...
%                      logger(k).static.params.pposition,...
%                      logger(k).static.params.aposition,...
%                      u(:,i), [t(1), t(end)],...
%                      logger(k).static.params.prom,...
%                      i, length(t));
%        end
%               
%        h = figure(135); 
%        if(k==2) clf; end
%        h.Name = 'u dist'; h.WindowStyle = 'docked';
%        plot(t,u_dist, '--'); hold on; grid on;
%        plot(t,u(1,:),  '-',  'Color', 'r'); hold on
%        plot(t,u(2,:),  '-',  'Color', 'r'); hold on
%        legend('disturbance u', 'u roll', 'u pitch');
%     end
%     
%     
%     h = figure(136);
%     if(k==2) clf; end
%     h.Name = 'ROM angles'; h.WindowStyle = 'docked';
%     subplot(3,3,1)
%         plot(t, yd(1,:)); hold on;
%         title('y(1)');
%     subplot(3,3,4)
%         plot(t, yd(2,:)); hold on;
%         title('y(2)');
%     subplot(3,3,7)
%         plot(t, yd(3,:)); hold on;
%         title('ASLIP leg length');
%         
%     subplot(3,3,2)
%         plot(t, x(domain.getJointIndices('BaseRotY'),:)); hold on;
%         title('BaseRotY');
%     subplot(3,3,5)
%         plot(t, x(domain.getJointIndices('BaseRotX'),:)); hold on;
%         title('BaseRotX');
%         
%     subplot(3,3,3)
%         plot(t, yd(1,:)+ x(domain.getJointIndices('BaseRotY'),:)); hold on;
%         title('ASLIP leg pitch');
%     subplot(3,3,6)
%         plot(t, yd(2,:)+ x(domain.getJointIndices('BaseRotX'),:)); hold on;
%         title('ASLIP leg roll');
%     subplot(3,3,9)
%         plot(t, yd(3,:)); hold on;
%         title('ASLIP leg length');

    
    %% Holonomic Constraints
    %%% Constraint Wrenches %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if ~isempty(wrenches) && k<3
        w_fields = fields(wrenches);
        for w = 1:numel(w_fields)
            wName = w_fields{w};
            wrench = wrenches.(wName);

            h = figure(150+w);
            if(k==1) clf; end
            h.Name = wName; h.WindowStyle = 'docked';
            hold on; grid on;

            nW = length(wrench(:,1));
            nrows = 2;
            ncols = ceil(nW/nrows);
            for j = 1:nW
                subplot(nrows,ncols,j); hold on; grid on;
                plot(t, wrench(j,:), 'Color', mColor);

                if strcmp(plotType, 'optsim')
                    wOpt = wrenchesOpt.(wName);
                    plot(tOpt, wOpt(j,:), 'r');
                end

                xlabel('Time (s)');
                ylabel('CWrench (Nm)|(N)');
                title(j);
            end
        end

        %%% Holonomic violations %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        h_fields = fields( logger(k).plant.HolonomicConstraints );
        for w = 1:numel(h_fields)

            hName = h_fields{w};        
            % Compute Holonomic data & plot it
            holo_pos = [];
            for i = 1:length(t)
                holo_func = logger(k).plant.HolonomicConstraints.(hName);
                holo_pos(:,i) = holo_func.calcConstraint(x(:,i));
            end

            h = figure(160+w); 
            if(k==1) clf; end
            h.Name = ['H', h_fields{w}]; h.WindowStyle = 'docked';
            hold on; grid on;

            nH = length(holo_pos(:,1));
            nrows = 2;
            ncols = ceil(nH/nrows);
            for j = 1:nH
                subplot(nrows,ncols,j); hold on; grid on;
                plot(t, holo_pos(j,:), 'Color', mColor);
                xlabel('Time (s)');
                ylabel('Holo violation');
                title(j);
            end
        end
    end
 
    
    %% Feet Positions
    h = figure(200); 
    if(k==1) clf; end
    h.Name = 'feet'; h.WindowStyle = 'docked';
    subplot(3,2,1)
        plot(t, RF_pos(1,:), 'LineWidth', 2, 'Color', mColor); hold on
        plot(t, LF_pos(1,:), 'LineWidth', 2, 'Color', 'g'); hold on
        if strcmp(plotType, 'optsim')
            plot(tOpt, LF_pos_opt(1,:), 'LineWidth', 2, 'Color', 'r'); hold on
        end
        legend('rightFoot(stance)', 'leftFoot(non-stance)', 'Location','best');
        title('foot pos X');
    subplot(3,2,2)
        plot(t, RF_vel(1,:), 'LineWidth', 2, 'Color', mColor); hold on
        plot(t, LF_vel(1,:), 'LineWidth', 2, 'Color', 'g'); hold on
        if strcmp(plotType, 'optsim')
            plot(tOpt, LF_vel_opt(1,:), 'LineWidth', 2, 'Color', 'r'); hold on
        end
        legend('rightFoot(stance)', 'leftFoot(non-stance)', 'Location','best');
        title('foot vel X');
    subplot(3,2,3)
        plot(t, RF_pos(2,:), 'LineWidth', 2, 'Color', mColor); hold on
        plot(t, LF_pos(2,:), 'LineWidth', 2, 'Color', 'g'); hold on
        %plot(t, tau, 'k'); grid on
        legend('rightFoot(stance)', 'leftFoot(non-stance)',  'Location','best');
        title('foot pos Y');
    subplot(3,2,4)
        plot(t, RF_vel(2,:), 'LineWidth', 2, 'Color', mColor); hold on
        plot(t, LF_vel(2,:), 'LineWidth', 2, 'Color', 'g'); hold on
        %plot(t, tau, 'k'); grid on
        legend('rightFoot(stance)', 'leftFoot(non-stance)');
        title('foot vel Y');
    subplot(3,2,5)
        plot(t, RF_pos(3,:), 'LineWidth', 2, 'Color', mColor); hold on
        plot(t, LF_pos(3,:), 'LineWidth', 2, 'Color', 'g'); hold on
        %plot(t, tau, 'k'); grid on
        legend('rightFoot(stance)', 'leftFoot(non-stance)');
        title('foot pos Z');
    subplot(3,2,6)
        plot(t, RF_vel(3,:), 'LineWidth', 2, 'Color', mColor); hold on
        plot(t, LF_vel(3,:), 'LineWidth', 2, 'Color', 'g'); hold on
        legend('rightFoot(stance)', 'leftFoot(non-stance)');
        title('foot vel Z');
    
    %%%%% NSF clearance %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % only plot this to check optimization results.
    if strcmp(plotType, 'opt')
        % clearanceInd = [18,   26]; 
        % clearanceZ   = [0.13, 0.018];
        h = figure(201); 
        if(k==1) clf; end
        h.Name = 'clearance'; h.WindowStyle = 'docked';
        subplot(2,2,1)
            plot(LF_pos(1,:), LF_pos(3,:), 'LineWidth', 2); 
            grid on; hold on
            if strcmp(plotType, 'opt')
                plot(LF_pos(1,:), LF_pos(3,:), 'o'); hold on;
            end
            % plot(LF_pos(1, clearanceInd), clearanceZ, 'X', 'LineWidth', 3); % clearance
            % for i = 1:length(nlp.Phase(1).ConstrTable{:,36})
            %     if ~isempty(nlp.Phase(1).ConstrTable{i,36}.LowerBound)
            %         disp(num2str(i));
            %         disp(num2str(nlp.Phase(1).ConstrTable{i,36}.LowerBound));
            %     end
            % end
            xlabel 'nsf-pos-X'; title 'nsf-pos-Z';
        subplot(2,2,3)
            plot(LF_pos(1,:), LF_vel(3,:), 'LineWidth', 2); grid on; hold on
            if strcmp(plotType, 'opt')
                plot(LF_pos(1,:), LF_vel(3,:), 'o'); hold on;
            end
            xlabel 'nsf-pos-X'; title 'nsf-vel-Z';
        subplot(2,2,2)
            plot(t, LF_pos(3,:),'LineWidth', 2); grid on; hold on
            if strcmp(plotType, 'opt')
                plot(t, LF_pos(3,:),'o'); hold on
            end
            % plot(t(clearanceInd), clearanceZ, 'X', 'LineWidth', 3); % clearance
            xlabel 'time(s)'; legend('nsf-vel-Z');
        subplot(2,2,4)
            plot(t, LF_vel(3,:),'LineWidth', 2); grid on; hold on
            if strcmp(plotType, 'opt')
                plot(t, LF_vel(3,:),'o');
            end
            xlabel 'time(s)'; legend('nsf-vel-Z');
    end
    
    %% %%% Feet touch down placement %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    h = figure(210); 
    if(k==1) clf; end
    h.Name = 'touchDown'; h.WindowStyle = 'docked';
    subplot(3,2,1)
        plot(t, LF_pos(1,:),    'LineWidth', 2,  'Color', 'g'); hold on
        plot(t, x(1,:),         'LineWidth', 2,  'Color', mColor); hold on
        plot(t, com(1,:), '-.', 'LineWidth', 1.5, 'Color', mColor); grid on
        legend('left (nsf) X', 'Base X', 'COM X'); xlabel time;
        title 'nsf-X vs. com-X';
    subplot(3,2,3)
        plot(t, LF_pos(2,:),    'LineWidth', 2,  'Color', 'g'); hold on
        plot(t, x(2,:),         'LineWidth', 2,  'Color', mColor); hold on
        plot(t, com(2,:), '-.', 'LineWidth', 1.5, 'Color', mColor); grid on
        legend('left (nsf) Y', 'Base Y', 'COM Y'); xlabel time;
        title 'nsf-Y vs. com-Y';
    subplot(3,2,5)
        plot(t, x(3,:),         'LineWidth', 2,  'Color', mColor); hold on
        plot(t, com(3,:), '-.', 'LineWidth', 1.5, 'Color', mColor); grid on
        legend('Base Z', 'COM Z'); xlabel time;
        title 'nsf-Z vs. com-Z';
    subplot(3,2,[2,4,6])
        %-left foot
        plot(LF_pos(2,:), LF_pos(1,:), 'g', 'LineWidth', 2); hold on
            set(gca, 'XDir','reverse'); % quiver(zeros(2,1),zeros(2,1),[.1;0],[0;.1])
        %if strcmp(plotType, 'opt')
        %    plot(LF_pos(2,:), LF_pos(1,:), 'o', 'Color', 'g'); hold on
        %end
        %-right foot
        plot(RF_pos(2,:), RF_pos(1,:), 'g', 'LineWidth', 3, 'LineStyle','--'); hold on
        %if strcmp(plotType, 'opt')
        %    plot(RF_pos(2,:), RF_pos(1,:), 'o', 'Color', 'g'); hold on
        %end
        %- BasePos
        plot(x(2,:), x(1,:), 'r', 'LineWidth', 1.5); hold on
        %if strcmp(plotType, 'opt')
        %    plot(x(2,:), x(1,:), 'o', 'Color', 'r'); hold on
        %end
        %- COM
        plot(com(2,:), com(1,:), '-.', 'LineWidth', 1.5, 'Color', 'r'); grid on
        
        legend('left (nsf)', 'right (sf)', 'BasePos', 'COM');
        xlabel Y-pos; ylabel X-pos; title('FOOT vs Base Placement ');
       
end %-end of plotting


% save('com_data.mat', 'com_data');


end