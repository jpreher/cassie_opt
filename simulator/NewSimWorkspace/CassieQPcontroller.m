classdef CassieQPcontroller < ...
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
        stance_leg
        gait_param
        param
        outputs
        controller
    end
    
    methods
        % Constructor
        function obj = CassieQPcontroller()
            obj.eom = CassieEOM(false);
            
            stance_leg = 'Right';
            initial_speed = [0;0];
            load('gaitLib_interp.mat');
            obj.gait_param = param_lib(initial_speed(1), stance_leg, interp_lib);
            
            obj.param = GaitParams();
            obj.param.controller = 'QPwalk';
            obj.param.ep = 1e-3;
            obj.param.w_slack = 1e3;
            obj.param.stance_leg = stance_leg;
            obj.param.gait_param = obj.gait_param;
            obj.param.timer = Timer();
            obj.param.phase = PhaseTime(flipud(obj.gait_param.pposition));
            obj.param.interp_lib = interp_lib;
            obj.param.avg_vel = [0;0];
            obj.param.prev_avg_vel = initial_speed';
            obj.param.k_step = 0;
            obj.param.vd = initial_speed';
            obj.param.v_step_avg_allocator = [0;0];
            obj.param.v_step_avg_count = 0;
            
            obj.outputs = walkOutputs(obj.param);
            obj.controller = QPcontroller(obj.outputs);
        end
    end
    
    methods(Access = protected)
        %% Common functions
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
        end
        
        function [u, tau, ya, yd, dya, dyd] = stepImpl(obj, t, X, stance_leg)
            % States
            q = X(1:22);
            dq = X(23:44);
            
            % Update dynamics
            obj.eom.update(q, dq, ...
                stance_leg < 0, stance_leg > 0, ...
                stance_leg > 0, stance_leg < 0);
            
            % Update parameters if stance changes
            if strcmp(obj.param.stance_leg, 'Right')
                if stance_leg < 0
                    obj.param.stance_leg = 'Left';
                    % Perform a motion transition
                    obj.param.gait_param = param_lib(obj.param.avg_vel(1), obj.param.stance_leg, obj.param.interp_lib);
                    obj.controller.outputs.update(q,dq,obj.param);
                    obj.param.gait_param.aposition = computeStartTransition( obj.param.phase.dtau, ...
                        obj.param.gait_param.pposition, obj.param.gait_param.aposition, ...
                        obj.controller.outputs.ya, obj.controller.outputs.dya, [1, 3:7]);
                    % TODO
                    avg_vel = [0; 0];
                    prev_avg_vel = [0; 0];
                    
                    % Compute Raibert for the end
                    yF   = bezier(obj.param.gait_param.aposition, 1);
                    Kp_raibert = [0.38;  0.255];
                    Kd_raibert = [0.35;  0.15];
                    offset = -Kp_raibert .* (avg_vel - obj.param.vd) - Kd_raibert .* (avg_vel - prev_avg_vel);
                    if obj.param.stance_leg > 0
                        offset(2) = clamp(offset(2), -1.0, 0);
                        yF(1) = yF(1) - offset(2);
                    elseif obj.param.stance_leg < 0
                        offset(2) = clamp(offset(2), 0, 1.0);
                        yF(1) = yF(1) - offset(2);
                    end
                    yF(7) = yF(7) + offset(1);
                    obj.param.gait_param.aposition = computeEndTransition( obj.param.phase.dtau, obj.param.gait_param.aposition, yF );
                end
            elseif strcmp(obj.param.stance_leg, 'Left')
                if stance_leg > 0
                    obj.param.stance_leg = 'Right';
                    % Perform a motion transition
                    obj.param.gait_param = param_lib(obj.param.avg_vel(1), obj.param.stance_leg, obj.param.interp_lib);
                    obj.controller.outputs.update(q,dq,obj.param);
                    obj.param.gait_param.aposition = computeStartTransition( obj.param.phase.dtau, ...
                        obj.param.gait_param.pposition, obj.param.gait_param.aposition, ...
                        obj.controller.outputs.ya, obj.controller.outputs.dya, [1, 3:7]);
                    % TODO
                    avg_vel = [0; 0];
                    prev_avg_vel = [0; 0];
                    
                    % Compute Raibert for the end
                    yF   = bezier(obj.param.gait_param.aposition, 1);
                    Kp_raibert = [0.38;  0.255];
                    Kd_raibert = [0.35;  0.15];
                    offset = -Kp_raibert .* (avg_vel - obj.param.vd) - Kd_raibert .* (avg_vel - prev_avg_vel);
                    if obj.param.stance_leg > 0
                        offset(2) = clamp(offset(2), -1.0, 0);
                        yF(1) = yF(1) - offset(2);
                    elseif obj.param.stance_leg < 0
                        offset(2) = clamp(offset(2), 0, 1.0);
                        yF(1) = yF(1) - offset(2);
                    end
                    yF(7) = yF(7) + offset(1);
                    obj.param.gait_param.aposition = computeEndTransition( obj.param.phase.dtau, obj.param.gait_param.aposition, yF );
                end
            end
            
            obj.param.timer.update(t);
            u = obj.controller.update(q, dq, obj.eom, obj.param);
            
            % Update outputs
            tau = obj.param.phase.tau;
            ya = obj.controller.outputs.ya;
            yd = obj.controller.outputs.yd;
            dya = obj.controller.outputs.dya;
            dyd = obj.controller.outputs.dyd;
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
        %% Simulink functions
        function [name_1, name_2, name_3] = getInputNamesImpl(~)
            %GETINPUTNAMESIMPL Return input port names for System block
            name_1 = 't';
            name_2 = 'X';
            name_3 = 'stance';
        end % getInputNamesImpl
        
        function [name_1, name_2, name_3, name_4, name_5, name_6] = getOutputNamesImpl(~)
            %GETOUTPUTNAMESIMPL Return output port names for System block
            name_1 = 'u';
            name_2 = 'tau';
            name_3 = 'ya';
            name_4 = 'yd';
            name_5 = 'dya';
            name_6 = 'dyd';
        end % getOutputNamesImpl
        
        % PROPAGATES CLASS METHODS ============================================
        function [sz_1, sz_2, sz_3, sz_4, sz_5, sz_6] = getOutputSizeImpl(~)
            %GETOUTPUTSIZEIMPL Get sizes of output ports.
            sz_1 = [10, 1];
            sz_2 = [1, 1];
            sz_3 = [9, 1];
            sz_4 = [9, 1];
            sz_5 = [9, 1];
            sz_6 = [9, 1];
        end % getOutputSizeImpl
        
        function [dt_1, dt_2, dt_3, dt_4, dt_5, dt_6] = getOutputDataTypeImpl(~)
            %GETOUTPUTDATATYPEIMPL Get data types of output ports.
            dt_1 = 'double';
            dt_2 = 'double';
            dt_3 = 'double';
            dt_4 = 'double';
            dt_5 = 'double';
            dt_6 = 'double';
        end % getOutputDataTypeImpl
        
        function [cp_1, cp_2, cp_3, cp_4, cp_5, cp_6] = isOutputComplexImpl(~)
            %ISOUTPUTCOMPLEXIMPL Complexity of output ports.
            cp_1 = false;
            cp_2 = false;
            cp_3 = false;
            cp_4 = false;
            cp_5 = false;
            cp_6 = false;
        end % isOutputComplexImpl
        
        function [flag_1, flag_2, flag_3, flag_4, flag_5, flag_6] = isOutputFixedSizeImpl(~)
            %ISOUTPUTFIXEDSIZEIMPL Fixed-size or variable-size output ports.
            flag_1 = true; % True = fixed size
            flag_2 = true;
            flag_3 = true;
            flag_4 = true;
            flag_5 = true;
            flag_6 = true;
        end % isOutputFixedSizeImpl
        
    end
    
end
