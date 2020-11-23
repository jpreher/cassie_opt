classdef ExponentialSmoothingFilter < handle
    % ExponentialSmoothingFilter
    %   Smoothing with exponential decay on past measurements
    %   A form of low-pass filter
    %   See: https://en.wikipedia.org/wiki/Exponential_smoothing
    %   Time constant is time to reach 1-1/e (63.2%) of original signal
    % Author: Jake Reher (jreher@caltech.edu)
    
    properties
        value
        alpha
        maximumHistory
        hasFirst
        measurements
        nMeasurements
    end
    
    methods
        function obj = ExponentialSmoothingFilter(dataLen, sampleFrequency, timeConstant, maximumHistory)
            % Construct the filter.
            
            obj.value = zeros(dataLen, 1);
            obj.maximumHistory = maximumHistory;
            obj.measurements = zeros(dataLen, maximumHistory);
            obj.updateFilterConstant(sampleFrequency, timeConstant);
            obj.reset();
        end
        
        function [] = updateFilterConstant(obj, sampleFrequency, timeConstant)
            obj.alpha = 1 - exp(-1/(sampleFrequency*timeConstant));
        end
        
        function [] = reset(obj)
            % Hard reset on filtered value, next update will re-initialize
            % the value with the raw value.
            
            obj.hasFirst = false;
            obj.value(:) = 0;
            obj.measurements(:,:) = 0;
            obj.nMeasurements = 0;
        end
        
        function [] = update(obj, raw)
            % Takes a raw value and updates the filter
                        
            % Determine where the last value in the new measurement array
            % will lie
            if (obj.nMeasurements >= obj.maximumHistory)
                endIndex = obj.nMeasurements-1;
            else
                endIndex = obj.nMeasurements;
                obj.nMeasurements = obj.nMeasurements + 1;
            end
            
            % Shift the measurement vector and insert new measurement
            for i = endIndex : -1 : 1
                obj.measurements(:,i+1) = obj.measurements(:,i);
            end
            obj.measurements(:,1) = raw;
            
            % Perform filtering
            if obj.nMeasurements <= 1
                obj.value = obj.measurements(:,1);
            else
                sum = 0.*obj.value;
                N = obj.nMeasurements;
                for j = 1 : obj.nMeasurements-1
                    sum = sum + ( obj.alpha * (1-obj.alpha)^(j-1) ) .* obj.measurements(:,j);
                end
                obj.value = sum + (1-obj.alpha)^(N-1) .* obj.measurements(:,N);                
            end
        end
            
    end
    
end