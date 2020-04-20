function [ ] = animate_rviz( logger, scale_factor )

if nargin < 2
    scale_factor = 1;
end

% pause(1);

% Initialize ROS
% rosshutdown
% rosinit

% Setup the publishers
joint_state = rospublisher('/model_pub/joint_state','sensor_msgs/JointState');
odom_state = rospublisher('/model_pub/odom_state','nav_msgs/Odometry');

% Setup the messages
joint_state_msg = rosmessage(joint_state);
odom_state_msg = rosmessage(odom_state);

% Establish the constant message parameters
joint_state_msg.Name = logger(1).plant.States.x.label(7:end);
odom_state_msg.Header.FrameId = '/world';
odom_state_msg.ChildFrameId = '/BaseRotZLink';

joint_state_msg.Velocity = zeros(length(joint_state_msg.Name),1);
joint_state_msg.Effort   = zeros(length(joint_state_msg.Name),1);

for i = 1:length(logger)
    % Extract and check if animatable domain
    hasFlow = true;
    try 
        logger(i).flow;
    catch
        hasFlow = false;
    end
    
    % If the domain is continuous, animate
    if hasFlow
        calcs = logger(i).flow;
%         t  = logger(i).flow.t;
%         qb = logger(i).flow.states.x(1:6,   :);
%         q  = logger(i).flow.states.x(7:end, :);
        hz = 200;
        
        [~,qb]  = even_sample(calcs.t, calcs.states.x(1:6, :), hz);
        [t,q]   = even_sample(calcs.t, calcs.states.x(7:end, :), hz);

        for k = 1:length(t)
            % Odom Position
            odom_state_msg.Pose.Pose.Position.X = qb(1,k);
            odom_state_msg.Pose.Pose.Position.Y = qb(2,k);
            odom_state_msg.Pose.Pose.Position.Z = qb(3,k);
            
            % Odom Orientation
            rot = rotx(radtodeg(qb(4,k))) * roty(radtodeg(qb(5,k))) * rotz(radtodeg(qb(6,k)));
            quat = rotm2quat(rot);
            odom_state_msg.Pose.Pose.Orientation.W = quat(1);
            odom_state_msg.Pose.Pose.Orientation.X = quat(2);
            odom_state_msg.Pose.Pose.Orientation.Y = quat(3);
            odom_state_msg.Pose.Pose.Orientation.Z = quat(4);
            
            % Joint Positions
            joint_state_msg.Position = q(:,k);
            
            % Get current time
            t_now = rostime('now');
            % Send floating base data
            odom_state_msg.Header.Stamp = t_now;
            send(odom_state,odom_state_msg);
            % Send joint data
            joint_state_msg.Header.Stamp = t_now;
            send(joint_state,joint_state_msg);
            pause(scale_factor/hz);
%             pause(0.001);
%             pause(0.1);
%             pause(1);
        end
    end
end




end

