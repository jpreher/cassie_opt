%% Function: SingleSupport
%
% Description: Generates the domain structure for 3D single-support.
%
% Author: Jenna Reher, jreher@caltech.edu
% ________________________________________

function [ domain ] = SingleSupport( domain, stanceLeg, ~ )
    % Get the current leg
    leg = stanceLeg;
    assert(strcmp(leg,'Left') || strcmp(leg,'Right'), 'The leg must be specified as Left or Right');
    
    % States
    x = domain.States.x;
    dx = domain.States.dx;
    
    % Leg configuration as inverted pendulum
    % Toe Pitch Position
    left_tp_frame = domain.Joints(getJointIndices(domain, 'LeftFootPitch'));
    p_ltp = domain.getCartesianPosition(left_tp_frame)';
    p_ltp = p_ltp.subs(x(1:6), zeros(6,1));

    right_tp_frame = domain.Joints(getJointIndices(domain, 'RightFootPitch'));
    p_rtp = domain.getCartesianPosition(right_tp_frame)';
    p_rtp = p_rtp.subs(x(1:6), zeros(6,1));

    % Derive the leg shapes
    left_com_frame = domain.Joints(getJointIndices(domain, 'LeftHipPitch'));
    p_CoM = domain.getCartesianPosition(left_com_frame)';
    p_CoM = p_CoM.subs(x(1:6), zeros(6,1));
    
    vector = p_ltp - p_CoM;
    vector = vector.subs(x('LeftHipYaw'), 0);
    vector = vector.subs(x('LeftHipRoll'), 0);
    vector = vector.subs(x('LeftShinPitch'), 0);
    vector = vector.subs(x('LeftTarsusPitch'), deg2rad(13) - x('LeftKneePitch'));
    vector = eval_math_fun('Simplify', vector);
    vector = eval_math_fun('Chop', vector);
    left_pitch = atan2(-vector(1), -vector(3));

    left_legLength = vector(1).^2 + vector(2).^2 + vector(3).^2;
    left_legLength = eval_math_fun('Simplify', left_legLength);
    left_legLength = eval_math_fun('Chop', left_legLength);
    left_legLength = sqrt(left_legLength);
    
    right_com_frame = domain.Joints(getJointIndices(domain, 'RightHipPitch'));
    p_CoM = domain.getCartesianPosition(right_com_frame)';
    p_CoM = p_CoM.subs(x(1:6), zeros(6,1));
    
    vector = p_rtp - p_CoM;
    vector = vector.subs(x('RightHipYaw'), 0);
    vector = vector.subs(x('RightHipRoll'), 0);
    vector = vector.subs(x('RightShinPitch'), 0);
    vector = vector.subs(x('RightTarsusPitch'), deg2rad(13) - x('RightKneePitch'));
    vector = eval_math_fun('Simplify', vector);
    vector = eval_math_fun('Chop', vector);
    right_pitch = atan2(-vector(1), -vector(3));

    right_legLength = vector(1).^2 + vector(2).^2 + vector(3).^2;
    right_legLength = eval_math_fun('Simplify', right_legLength);
    right_legLength = eval_math_fun('Chop', right_legLength);
    right_legLength = sqrt(right_legLength);
    
    % Case for which foot is stance
    if strcmp(leg, 'Right')
        nsl = 'Left';
        p_smf = getCartesianPosition(domain, domain.ContactPoints.RightSole);
        p_nsmf = getCartesianPosition(domain, domain.ContactPoints.LeftSole);
        p_toe  = getCartesianPosition(domain, domain.ContactPoints.LeftToe);
        p_heel = getCartesianPosition(domain, domain.ContactPoints.LeftHeel);
        s_ll = right_legLength;
        s_lp = right_pitch;
        ns_ll = left_legLength;
        ns_lp = left_pitch;
        
        pelvis_frame = domain.Joints(getJointIndices(domain,'RightHipPitch'));
        p_pel = getCartesianPosition(domain, pelvis_frame);
    else
        nsl = 'Right';
        p_smf = getCartesianPosition(domain, domain.ContactPoints.LeftSole);
        p_nsmf = getCartesianPosition(domain, domain.ContactPoints.RightSole);
        p_toe  = getCartesianPosition(domain, domain.ContactPoints.RightToe);
        p_heel = getCartesianPosition(domain, domain.ContactPoints.RightHeel);
        ns_ll = right_legLength;
        ns_lp = right_pitch;
        s_ll = left_legLength;
        s_lp = left_pitch;
        
        pelvis_frame = domain.Joints(getJointIndices(domain,'LeftHipPitch'));
        p_pel = getCartesianPosition(domain, pelvis_frame);
    end
        
    %% Add contact
    addZMP = true;
    domain = CustomContact.CassieFootCustom(domain, leg, addZMP);
            
    %% Add event: nsf reaches ground
    % height of non-stance foot
    h_nsf = UnilateralConstraint(domain, p_nsmf(3), 'nsf', 'x');
    domain = addEvent(domain, h_nsf);
        
    %% Deal with spring dynamics
    % Check if we applied behavior spring force
    % If yes, make swing leg rigid
    % If no, manually add spring force, make swing leg very high damping
    if strcmp(domain.Fvec{end}.Name, 'Ge_vec_cassie_v4')
        % No spring force was added
        k_knee = 2300;
        b_knee = 4.6;
        k_ankle = 2000;
        b_ankle = 4.0;
        fs = SymExpression(zeros(domain.numState,1));
        
        %%% Knee spring
        fs(getJointIndices(domain, [nsl, 'ShinPitch']),1) = -domain.States.x([nsl, 'ShinPitch']) * k_knee * 3.0 - domain.States.dx([nsl, 'ShinPitch']) * b_knee * 15.0;
        fs(getJointIndices(domain, [leg, 'ShinPitch']),1) = -domain.States.x([leg, 'ShinPitch']) * k_knee - domain.States.dx([leg, 'ShinPitch']) * b_knee;
        %%% Heel Spring
        fs(getJointIndices(domain, [nsl, 'AchillesSpring']), 1) = -domain.States.x([nsl, 'AchillesSpring']) * k_ankle * 3.0 - domain.States.dx([nsl, 'AchillesSpring']) * b_ankle * 15.0;
        fs(getJointIndices(domain, [leg, 'AchillesSpring']), 1) = -domain.States.x([leg, 'AchillesSpring']) * k_ankle - domain.States.dx([leg, 'AchillesSpring']) * b_ankle;
        
        fs_fun = SymFunction(['spring_forces_', domain.Name], fs, {x, dx});
        domain.appendDriftVector(fs_fun);
        
    else
        % Swing leg is rigid
        rigid_springs = [x([nsl, 'ShinPitch']);
                         x([nsl, 'AchillesSpring'])];
        hol_trans_constraint = HolonomicConstraint(domain, rigid_springs, 'RigidSprings', 'DerivativeOrder',2);
        domain = addHolonomicConstraint(domain, hol_trans_constraint);
    end
    
    %% Define outputs!!!
    % Phase Variable - Time
    t = SymVariable('t');
    p = SymVariable('p',[2,1]);
    tau = (t-p(2))/(p(1)-p(2));
    p_hip = p_pel(1) - p_smf(1);
    p_hip = p_hip.subs(x(1:6), zeros(6,1));
    p_hip = p_hip.subs(x('LeftShinPitch'), 0);
    p_hip = p_hip.subs(x('LeftTarsusPitch'), deg2rad(13) - x('LeftKneePitch'));
    p_hip = p_hip.subs(x('RightShinPitch'), 0);
    p_hip = p_hip.subs(x('RightTarsusPitch'), deg2rad(13) - x('RightKneePitch'));
    %     p_hip = eval_math_fun('Simplify', p_hip);
    
    deltaPhip = linearize(p_hip, x);
    %     p = SymVariable('p',[2,1]);
    %     tau = (deltaPhip-p(2))/(p(1)-p(2));
    
    %% Relative degree one output: linearized hip velocity
%     v_hip = jacobian(deltaPhip, x)*dx;
%     y1 = VirtualConstraint(domain, v_hip,'velocity','DesiredType','Constant',...
%         'RelativeDegree',1,'OutputLabel',{'vhip'},'PhaseType','TimeBased',...
%         'PhaseVariable', tau, ...
%         'PhaseParams', p, ...
%         'Holonomic',false);
%     domain = addVirtualConstraint(domain,y1);
    
    % RD2 Position
    y = [x('BaseRotX');
         x('BaseRotY');
         x([leg, 'HipYaw']);
         s_ll; 
         ns_ll;
         ns_lp;
         x([nsl, 'HipRoll']);
         x([nsl, 'HipYaw']);
         p_heel(3) - p_toe(3)];
    
    ylbl= {'BaseRoll', ...
           'BasePitch', ...
           [leg, 'HipYaw'], ...
           [leg, 'LegLength'], ...
           [nsl, 'LegLength'], ...
           [nsl, 'LegPitch'],...
           [nsl, 'HipRoll'], ... 
           [nsl, 'HipYaw'], ...
           'nsf_cartY'};
    
    % Assign to domain
    y2 = VirtualConstraint(domain, y, 'position', ...
                           'DesiredType',    'Bezier', ...
                           'PolyDegree',     6,...
                           'RelativeDegree', 2, ...
                           'PhaseType',      'TimeBased', ...
                           'PhaseVariable',  tau, ...
                           'PhaseParams',    p, ...
                           'OutputLabel',    {ylbl}, ...
                           'Holonomic',      true);
    domain = addVirtualConstraint(domain,y2);
    
    
end

