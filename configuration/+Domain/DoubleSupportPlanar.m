%% Function: SingleSupport
%
% Description: Generates the domain structure for 3D single-support.
%
% Author: Jenna Reher, jreher@caltech.edu
% ________________________________________

function [ domain ] = DoubleSupportPlanar( domain, stanceLeg, ~ )
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
    end
        
    %% Add contact
    addZMP = true;
    domain = CustomContact.CassieFootCustom(domain, nsl, addZMP);
    domain = CustomContact.CassieFootCustom(domain, leg, addZMP);
            
    %% Add event: Ground-reaction-force of left-foot reaches Zero. (Holonomic)
    if strcmp(leg, 'Right')
        nsf_constraint = domain.Inputs.ConstraintWrench.fLeftSole;
    else
        nsf_constraint = domain.Inputs.ConstraintWrench.fRightSole;
    end
    
    F_nsf = UnilateralConstraint(domain, nsf_constraint(3), ...
                        'groundForce', ['f',nsl, 'Sole']);
    domain = addEvent(domain, F_nsf);
        
    %% Define outputs!!!
    % Phase Variable - Time
    t = SymVariable('t');
    p = SymVariable('p',[2,1]);
    tau = (t-p(2))/(p(1)-p(2));
    
    hip_frame = domain.Joints(getJointIndices(domain,'RightHipPitch'));
    p_pel = getCartesianPosition(domain, hip_frame); 
    p_hip = p_pel(1) - p_smf(1);
    p_hip = p_hip.subs(x('BaseRotX'), 0);
    p_hip = p_hip.subs(x('BaseRotZ'), 0);
    deltaPhip = linearize(p_hip, x);
%     p = SymVariable('p',[2,1]);
%     tau = (deltaPhip-p(2))/(p(1)-p(2));
    
    %% Relative degree one output: linearized hip velocity
    v_hip = jacobian(deltaPhip, x)*dx;
    y1 = VirtualConstraint(...
        domain, v_hip,'velocity','DesiredType','Constant',...
        'RelativeDegree',1,'OutputLabel',{'vhip'},'PhaseType','TimeBased',...
        'Holonomic',false);
    %'PhaseVariable', tau, ...
    %'PhaseParams', p, ...
    domain = addVirtualConstraint(domain,y1);
    
    %% RD2 Position
    y = [...
        ns_ll;
        s_ll;
        ns_lp;
        x('BaseRotY')];
    
     ylbl= {...
         [nsl, 'LegLength'], ...
         [leg, 'LegLength'], ...
         [nsl, 'LegPitch'], ...
         'BaseRotY'};
     
    % Assign to domain
    y2 = VirtualConstraint(domain, y, 'position', ...
        'DesiredType',    'Bezier', ...
        'PolyDegree',     6,...
        'RelativeDegree', 2, ...
        'PhaseType',      'TimeBased', ...%'StateBased', ...
        'PhaseVariable',  tau, ...
        'PhaseParams',    p, ...
        'OutputLabel',    {ylbl}, ...
        'Holonomic',      true);
    domain = addVirtualConstraint(domain,y2);
    
end

