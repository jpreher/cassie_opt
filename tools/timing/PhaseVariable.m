classdef PhaseVariable < handle
    % PhaseVariable
    %   Template class for phase variables.
   
    properties
        phaseRange % Range of phase
        tau        % Current phase value on [0,1]
        dtau       % Derivative of phase
    end
    
    methods
        function obj = PhaseVariable(phaseRange)     
            % Constructor.
            
            % Reconfigure everything
            obj.reconfigure(phaseRange);
        end
        
        function [] = reconfigure(obj, phaseRange)
            % Reconfigure the phase variable with a new range parameter.
            
            % Assign the phase range
            if nargin < 2
                obj.phaseRange = [0,1];
            else
                obj.phaseRange = phaseRange;
            end
                                    
            % Assign the user facing attributes
            obj.tau = 0;
            obj.dtau = 0;
            
        end
        
        function [] = calcTauDTau(obj, pActual, dpActual)
            % Compute the phase and its derivative.
            
            obj.calcTau(pActual);
            obj.calcDTau(dpActual);
        end
        
        function [] = calcTau(obj, pActual)
            % Compute the phase.
            
            % Get upper and lower phase variable values
            p0 = obj.phaseRange(1);
            pf = obj.phaseRange(2);
            
            % Compute
            obj.tau = (pActual - p0) / (pf - p0);
            
            % Clamp tau on bezier region of validity
            obj.tau = clamp(obj.tau, 0, 1.25);
        end
        
        function [] = calcDTau(obj, dpActual)
            % Compute the phase derivative.
            
            % Get upper and lower phase variable values
            p0 = obj.phaseRange(1);
            pf = obj.phaseRange(2);

            % Linear scale with time
            obj.dtau = dpActual / (pf - p0);
            
        end
        
        
    end
    
end

