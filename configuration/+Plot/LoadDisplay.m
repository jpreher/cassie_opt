function robot_disp = LoadDisplay(behavior, varargin)
    robot = behavior.robotModel;
    skipExport = true;
    
    root_path = pwd;
    export_path = fullfile(root_path, 'export', behavior.name, 'animator');
    if ~exist(export_path,'dir')
        mkdir(export_path);
        skipExport = false;
    end
    addpath(export_path);
    
    if nargin > 3
        options = varargin;
    else        
        options = {'UseExported', true, 'ExportPath', export_path, 'SkipExporting', skipExport};
    end

    f = figure(1000);clf;
    robot_disp = frost.Animator.Display(f, robot, options{:});
    
    % Add the toe contact points as black spheres
    leftToe = robot.ContactPoints.LeftToe;
    leftHeel = robot.ContactPoints.LeftHeel;
    rightToe = robot.ContactPoints.RightToe;
    rightHeel = robot.ContactPoints.RightHeel;
    
    leftToeItem = frost.Animator.Sphere(robot_disp.axs, robot, leftToe, leftToe.Name, options{:});
    leftHeelItem = frost.Animator.Sphere(robot_disp.axs, robot, leftHeel, leftHeel.Name, options{:});
    rightToeItem = frost.Animator.Sphere(robot_disp.axs, robot, rightToe, rightToe.Name, options{:});
    rightHeelItem = frost.Animator.Sphere(robot_disp.axs, robot, rightHeel, rightHeel.Name, options{:});
    
    robot_disp.addItem(leftToeItem);
    robot_disp.addItem(leftHeelItem);
    robot_disp.addItem(rightToeItem);
    robot_disp.addItem(rightHeelItem);
    
    % Add the 4-bar linkage
%     left_heel = robot.LeftHeelFrame;
%     left_heel_end = robot.LeftHeelEndFrame;
%     right_heel = robot.RightHeelFrame;
%     right_heel_end = robot.RightHeelEndFrame;
    
%     leftHeelItem = frost.Animator.Lines(robot_disp.axs, robot, left_heel, left_heel_end, left_heel.Name, options{:});
%     rightHeelItem = frost.Animator.Lines(robot_disp.axs, robot, right_heel, right_heel_end, right_heel.Name, options{:});
%     
%     robot_disp.addItem(leftHeelItem);
%     robot_disp.addItem(rightHeelItem);
    


    % Create the 4-bar linkage
    % Show items: >> keys(robot_disp.items)
%     key = keys(robot_disp.items);
%     right_joints = find(strncmpi(key,'Joint_Right',11));
%     right_links  = find(strncmpi(key,'Link_Right', 10));
%     left_joints  = find(strncmpi(key,'Joint_Right',10));
%     left_links   = find(strncmpi(key,'Link_Right', 9));
%     item = robot_disp.items('Joint_joint1');
%     
%     for i = 1:numel(right_joints)
%         
%     end
%     
    
    set(robot_disp.axs,'XLim',[-2,2]);
    view(robot_disp.axs,[0,0]);
    
    %     item = robot_disp.items('EndEff');
    %     item.radius = 0.01;
    %     item = robot_disp.items('Joint_joint1');
    %     item.radius = 0.015;
    %     item = robot_disp.items('Joint_joint2');
    %     item.radius = 0.015;
    %     item = robot_disp.items('Link_link1_to_joint2');
    %     item.radius = 0.01;
    robot_disp.update(zeros(robot.numState,1));
end