%% Function: genPlotCode
% 
% Description:
%   To generate / compile some functionalities that are used ONLY for plotting.
%
% Author: Jenna Reher, jreher@caltech.edu
%         Wenlong Ma, wma@caltech.edu
% ______________________________________

function [] = genPlotCode(behavior)

obj = behavior.robotModel;
x = obj.States.x;
dx = obj.States.dx;

%% Added these for later use to 'cassie_hardware'.
% if ispc
    baseDir = pwd;
% else
%     baseDir = ros_resolve_local_url('package://cassie_opt/matlab/');
% end
export_path = strcat(baseDir, '/export/cassie_plot');
if ~exist(export_path,'dir')
    mkdir(char(export_path));
end

com_SF = SymFunction('com_position', obj.getComPosition, {x});
comV_SF = SymFunction('com_velocity', jacobian(com_SF,x)*dx, {x, dx});
com_SF.export(export_path);
comV_SF.export(export_path);

% obj.fs_fun.export(exportPath);
spring_func = SymFunction('springForce_plot', obj.fs_fun, {x,dx} );
spring_func.export(export_path);

RF_pos = SymFunction('RightMidFoot_pos', ...
   obj.ContactPoints.RightSole.computeCartesianPosition, ...
   {x});
RF_pos.export(export_path);

LF_pos = SymFunction('LeftMidFoot_pos', ...
   obj.ContactPoints.LeftSole.computeCartesianPosition, ...
   {x});
LF_pos.export(export_path);

RF_vel = SymFunction('RightMidFoot_vel', ...
         jacobian(obj.ContactPoints.RightSole.computeCartesianPosition, x) * dx, ...
         {x, dx});
RF_vel.export(export_path);

LF_vel = SymFunction('LeftMidFoot_vel', ...
         jacobian(obj.ContactPoints.LeftSole.computeCartesianPosition, x) * dx, ...
         {x, dx});
LF_vel.export(export_path);
end
