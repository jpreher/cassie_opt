classdef CassieEOM < handle % codegen
    
    properties
        De
        Cvec
        Gvec
        Be
        
        Dc
        Cc
        
        P
        Q
        R
        Sc
        Su
        E
        
        Fcontact
        Fspring
        
        Jc
        JCc
        Jcontact
        Jspring
        JspringLock
        
        dJc
        dJCc
        dJcontact
        dJspring
        dJspringLock
        
        leftContact
        rightContact
        
        useCoriolis
    end
    
    methods
        function obj = CassieEOM(useCoriolis)
            if nargin < 1
                useCoriolis = false;
            end
            obj.useCoriolis = useCoriolis;
            
            obj.De       = zeros(22,22);
            obj.Cvec     = zeros(22,1);
            obj.Gvec     = zeros(22,1);
            obj.Dc       = zeros(22,22);
            obj.Cc       = zeros(22,22);
            obj.P        = zeros(22,22);
            obj.Q        = zeros(22,22);
            obj.R        = zeros(22,2);
            obj.Sc       = zeros(2,22);
            obj.Su       = zeros(20,22);
            obj.Fcontact = zeros(12,1);
            obj.Fspring  = zeros(22,1);
            obj.Jcontact = zeros(10,22);
            obj.Jspring  = zeros(2,22);
            obj.Jc       = zeros(9,22);
            obj.Jc       = zeros(9,22);
            obj.JCc      = zeros(11,22);
            obj.dJCc     = zeros(11,22);
            
            obj.leftContact = false;
            obj.rightContact = false;
            
            if coder.target('MATLAB')
                obj.Be = frost_expr.dynamics.Be_cassie_v4(zeros(22,1));
            else
                % Unsupported right now
                error('Codegen not yet working on this');
            end
        end
        
        function [] = update(obj, q, dq, isLeftContact, isRightContact, leftLegSpringLock, rightLegSpringLock)
            
            if nargin < 4
                isLeftContact = obj.leftContact;
            else
                obj.leftContact = isLeftContact;
            end
            if nargin < 5
                isRightContact = obj.rightContact;
            else
                obj.rightContact = isRightContact;
            end
            if nargin < 6
                leftLegSpringLock = false;
            end
            if nargin < 7
                rightLegSpringLock = false;
            end
            
            obj.updateUnpinnedDynamics(q,dq);
            obj.updateContactConstraints(q, dq, isLeftContact, isRightContact, leftLegSpringLock, rightLegSpringLock);
            
        end
        
        function [] = updateContactConstraints(obj, q, dq, isLeftContact, isRightContact, leftLegSpringLock, rightLegSpringLock)
            
            if coder.target('MATLAB')
                obj.JspringLock = [frost_expr.constraints.J_left_fixed_constraint(q);
                                   frost_expr.constraints.J_right_fixed_constraint(q)];
                obj.dJspringLock = [frost_expr.constraints.Jdot_left_fixed_constraint(q,dq);
                                    frost_expr.constraints.Jdot_right_fixed_constraint(q,dq)];
                
                obj.Jcontact = [frost_expr.constraints.J_leftSole_constraint(q)
                                frost_expr.constraints.J_rightSole_constraint(q)];
                obj.Jspring = frost_expr.constraints.J_achilles_constraint(q);
                
                obj.dJcontact = [frost_expr.constraints.Jdot_leftSole_constraint(q, dq)
                                frost_expr.constraints.Jdot_rightSole_constraint(q, dq)];
                obj.dJspring = frost_expr.constraints.Jdot_achilles_constraint(q, dq);
            else
                % Unsupported right now
                error('Codegen not yet working on this');
            end
            
            % Build constraint matrix
            % Possibly bad, but resizes every loop
            obj.Jc = obj.Jspring;
            obj.dJc = obj.dJspring;
            
            obj.JCc = obj.Jspring;
            obj.dJCc = obj.dJspring;
            
            if leftLegSpringLock
                obj.Jc = [obj.Jc; obj.JspringLock(1:2,:)];
                obj.dJc = [obj.dJc; obj.dJspringLock(1:2,:)];
                
                obj.JCc = [obj.JCc; obj.JspringLock];
                obj.dJCc = [obj.dJCc; obj.dJspringLock];
            end
            if rightLegSpringLock
                obj.Jc = [obj.Jc; obj.JspringLock(3:4,:)];
                obj.dJc = [obj.dJc; obj.dJspringLock(3:4,:)];
                
                obj.JCc = [obj.JCc; obj.JspringLock];
                obj.dJCc = [obj.dJCc; obj.dJspringLock];
            end
            
             if isLeftContact
                obj.Jc = [obj.Jc; obj.Jcontact(1:5,:)]; %[1:3,5:6]
                obj.dJc = [obj.dJc; obj.dJcontact(1:5,:)];
                
                obj.JCc = [obj.JCc; obj.Jcontact(1:5,:)]; %[1:3,5:6]
                obj.dJCc = [obj.dJCc; obj.dJcontact(1:5,:)];
            end
            if isRightContact
                obj.Jc = [obj.Jc; obj.Jcontact(6:10,:)]; %[7:9,11:12]
                obj.dJc = [obj.dJc; obj.dJcontact(6:10,:)];
                
                obj.JCc = [obj.JCc; obj.Jcontact(6:10,:)]; %[7:9,11:12]
                obj.dJCc = [obj.dJCc; obj.dJcontact(6:10,:)];
            end
            
            % QR decomposition of constraints
            [obj.Q, R0, obj.E] = qr(obj.Jc', 'vector');
            k = size(obj.Jc,1);  % nconstraints
            n = 22;              % nstate
            obj.R = R0(1:k, :);
            obj.Sc = [eye(k,k),     zeros(k,n-k)];
            obj.Su = [zeros(n-k,k), eye(n-k,n-k)];
            
            % Aghili method
            %obj.P  = eye(22) - obj.Jspring \ obj.Jspring;
            %obj.Dc = obj.De + obj.P * obj.De - (obj.P * obj.De)';
            %obj.Cc = -obj.De * ( obj.Jspring \ obj.JdotSpring );
        end
        
        function [] = updateUnpinnedDynamics(obj, q, dq)
            if coder.target('MATLAB')
                obj.De = frost_expr.dynamics.De_cassie_v4(q);% + 0.1 * eye(22);
                obj.Gvec = frost_expr.dynamics.Ge_cassie_v4(q);
                obj.Jspring = frost_expr.constraints.J_achilles_constraint(q);
                
                if obj.useCoriolis
                    obj.Cvec = frost_expr.dynamics.Ce_cassie_v4(q,dq) * dq;
                else
                    obj.Cvec = zeros(22,1);
                end
            else
                % Unsupported right now
                error('Codegen not yet working on this');
            end
            
            % Spring forces
            k_knee = 2300;
            b_knee = 4.6; 
            k_ankle = 2000;
            b_ankle = 4.0;
            
            %%% Knee spring
            obj.Fspring(CassieStateEnum.RightShinPitch) = -q(CassieStateEnum.RightShinPitch) * k_knee - dq(CassieStateEnum.RightShinPitch) * b_knee;
            obj.Fspring(CassieStateEnum.LeftShinPitch)  = -q(CassieStateEnum.LeftShinPitch)  * k_knee - dq(CassieStateEnum.LeftShinPitch)  * b_knee;
            %%% Heel Spring
            obj.Fspring(CassieStateEnum.RightHeelSpring) = -q(CassieStateEnum.RightHeelSpring) * k_ankle - dq(CassieStateEnum.RightHeelSpring) * b_ankle;
            obj.Fspring(CassieStateEnum.LeftHeelSpring)  = -q(CassieStateEnum.LeftHeelSpring)  * k_ankle - dq(CassieStateEnum.LeftHeelSpring)  * b_ankle;
            
        end
               
    end
end

