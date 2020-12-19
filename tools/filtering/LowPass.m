classdef LowPass < handle
    % LowPass
    %   A basic implementation of a first-order low pass filter.
    
    properties
        frequency        % Filter update frequency (Hz)
        filter_bandwidth % Filter bandwidth
        alpha            % Filter constant
        filteredValue    % Current filtered value 
        hasFirst         % Flag which determines if the filter has been initialized
        lastValue        % Previous filtered value 
    end
    
    methods
        function obj = LowPass(size, frequency, bandwidth)
            % Construct the filter.
            
            obj.filteredValue = zeros(size, 1);
            obj.lastValue = zeros(size, 1);
            obj.setBandwidth(frequency, bandwidth);   
            obj.reset;
        end
        
        function [] = reset(obj)
            % Hard reset on filtered value, next update will re-initialize
            % the value with the raw value.
            
            obj.hasFirst = false;
            obj.filteredValue(:) = 0;
            obj.lastValue(:) = 0;
        end
        
        function [] = setBandwidth(obj, frequency, bandwidth)
            % Set the filter bandwidth and update frequency.
            % These values will determine the filter constant.
            
            obj.frequency = frequency;
            obj.filter_bandwidth = bandwidth;
            obj.alpha = (1 / frequency) / (1 / bandwidth + 1 / frequency);
        end
        
        function [] = update(obj, raw)
            % Takes a raw value and updates the filteredValue member
            % according to the set bandwidth and frequency.
            
            % If the filter has not yet been run, then initialize the class
            % with the raw value.
            if ~obj.hasFirst
                obj.filteredValue = raw;
                obj.lastValue = raw;
                obj.hasFirst = true;
            end
            
            obj.lastValue = obj.filteredValue;
            
            % Update the filtered value
            obj.filteredValue = obj.alpha * raw + (1 - obj.alpha) * obj.filteredValue;
        end
            
    end
    
end