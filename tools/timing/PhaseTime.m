classdef PhaseTime < PhaseVariable
    % PhaseTime
    %   Class for time based phase variable.
    
    properties
        pActual  % Phase value
        dpActual % Phase derivative value
    end
    
    methods
        function obj = PhaseTime(phaseRange)
            % Construct
            obj@PhaseVariable(phaseRange);
            
            % Initialize
            obj.pActual = 0;
            obj.dpActual = 0;
        end
        
        function [] = update(obj, timer)
            % Do all the things
            obj.calcP(timer);
            obj.calcTauDTau(obj.pActual, obj.dpActual);
            obj.tau = clamp(obj.tau, 0, 1.25);
        end
        
        function [] = calcP(obj, timer)
            % We are time based
            obj.pActual  = timer.timerScaled;
            obj.dpActual = timer.timeScale; % Slew rate the dtau
        end
                    
    end
    
end

