classdef CassieDynamics < ...
        matlab.System & ...
        matlab.system.mixin.Propagates ...
        %
    % NOTE: When renaming the class name Untitled, the file name
    % and constructor name must be updated to use the class name.
    %
    % This template includes most, but not all, possible properties, attributes,
    % and methods that you can implement for a System object in Simulink.
    
    % Public, tunable properties
    properties
        
    end
    
    % Public, non-tunable properties
    properties(Nontunable)
        
    end
    
    properties(DiscreteState)
        
    end
    
    % Pre-computed constants
    properties(Access = private)
        eom
    end
    
    methods
        % Constructor
        function obj = CassieDynamics()
            obj.eom = CassieEOM(false);
        end
    end
    
    methods(Access = protected)
        %% Common functions
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
        end
        
        function [dX, lambda, stance] = stepImpl(obj, t, X, u, tau, stance_leg)
            % States
            q = X(1:22);
            dq = X(23:44);
            
            % Check the impact and update if necessary
            stance = stance_leg;
            if stance_leg > 0
                p_swf = frost_expr.constraints.p_leftSole_constraint(q);
                if (tau > 0.5 && p_swf(3) <= 0)% || param.phase.tau >= 1
                    % Set appropriate params
                    stance = -1;
                    
                    % Perform a rigid body impact!
                    q(CassieStateEnum.RightShinPitch) = 0;
                    q(CassieStateEnum.RightHeelSpring) = 0;
                    q(CassieStateEnum.RightTarsusPitch) = deg2rad(13) - q(CassieStateEnum.RightKneePitch);
                    obj.eom.update(q, dq, ...
                        stance_leg < 0, stance_leg > 0, ...
                        stance_leg > 0, stance_leg < 0);
                    nImpConstr = size(obj.eom.Jc,1);
                    A_imp = [obj.eom.De -obj.eom.Jc'; obj.eom.Jc zeros(nImpConstr)];
                    b_imp = [obj.eom.De*dq; zeros(nImpConstr,1)];
                    y_imp = A_imp\b_imp;
                    ImpF = y_imp((22+1):end);
                    dq = y_imp(1:22);
                end
            elseif stance_leg < 0
                p_swf = frost_expr.constraints.p_rightSole_constraint(q);
                if (tau > 0.5 && p_swf(3) <= 0)% || param.phase.tau >= 1
                    % Set appropriate params
                    stance = 1;
                    
                    % Perform a rigid body impact!
                    q(CassieStateEnum.LeftShinPitch) = 0;
                    q(CassieStateEnum.LeftHeelSpring) = 0;
                    q(CassieStateEnum.LeftTarsusPitch) = deg2rad(13) - q(CassieStateEnum.LeftKneePitch);
                    obj.eom.update(q, dq, ...
                        stance_leg < 0, stance_leg > 0, ...
                        stance_leg > 0, stance_leg < 0);
                    nImpConstr = size(obj.eom.Jc,1);
                    A_imp = [obj.eom.De -obj.eom.Jc'; obj.eom.Jc zeros(nImpConstr)];
                    b_imp = [obj.eom.De*dq; zeros(nImpConstr,1)];
                    y_imp = A_imp\b_imp;
                    ImpF = y_imp((22+1):end);
                    dq = y_imp(1:22);
                end
            end
            
            
            % Update dynamics
            obj.eom.update(q, dq, ...
                stance_leg < 0, stance_leg > 0, ...
                stance_leg > 0, stance_leg < 0);
            
            % Evaluate the closed loop dynamics with the non-constraint projected
            % dynamics
            % Generate the vector fields
            Fv = obj.eom.Fspring - obj.eom.Cvec - obj.eom.Gvec;
            XiInv = obj.eom.Jc * (obj.eom.De \ transpose(obj.eom.Jc));
            Gv = obj.eom.Be * u;
            lambda = -XiInv \ (obj.eom.dJc * dq + obj.eom.Jc * (obj.eom.De \ (Fv + Gv)));
            
            % Final dynamics
            dX = [dq;
                obj.eom.De \ (Fv + Gv + obj.eom.Jc' * lambda)];
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
        %% Simulink functions
        function [name_1, name_2, name_3, name_4, name_5] = getInputNamesImpl(~)
            %GETINPUTNAMESIMPL Return input port names for System block
            name_1 = 't';
            name_2 = 'X';
            name_3 = 'u';
            name_4 = 'tau';
            name_5 = 'stance';
        end % getInputNamesImpl
        
        function [name_1, name_2, name_3] = getOutputNamesImpl(~)
            %GETOUTPUTNAMESIMPL Return output port names for System block
            name_1 = 'dX';
            name_2 = 'lambda';
            name_3 = 'stance';
        end % getOutputNamesImpl
        
        % PROPAGATES CLASS METHODS ============================================
        function [sz_1, sz_2, sz_3] = getOutputSizeImpl(~)
            %GETOUTPUTSIZEIMPL Get sizes of output ports.
            sz_1 = [44, 1];
            sz_2 = [9, 1];
            sz_3 = [1, 1];
        end % getOutputSizeImpl
        
        function [dt_1, dt_2, dt_3] = getOutputDataTypeImpl(~)
            %GETOUTPUTDATATYPEIMPL Get data types of output ports.
            dt_1 = 'double';
            dt_2 = 'double';
            dt_3 = 'double';
        end % getOutputDataTypeImpl
        
        function [cp_1, cp_2, cp_3] = isOutputComplexImpl(~)
            %ISOUTPUTCOMPLEXIMPL Complexity of output ports.
            cp_1 = false;
            cp_2 = false;
            cp_3 = false;
        end % isOutputComplexImpl
        
        function [flag_1, flag_2, flag_3] = isOutputFixedSizeImpl(~)
            %ISOUTPUTFIXEDSIZEIMPL Fixed-size or variable-size output ports.
            flag_1 = true; % True = fixed size
            flag_2 = true;
            flag_3 = true;
        end % isOutputFixedSizeImpl
        
    end
    
end
