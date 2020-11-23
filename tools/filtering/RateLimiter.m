classdef RateLimiter < handle
    %RATELIMITER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        rate_min
        rate_max
        prev_value
    end
    
    methods
        function obj = RateLimiter(rate_min, rate_max)
            obj.setLimits(rate_min, rate_max);
            obj.reset();
        end
        
        function [] = setLimits(obj, rate_min, rate_max)
            obj.rate_min = rate_min;
            obj.rate_max = rate_max;
        end
        
        function [] = reset(obj, value)
            if nargin < 2
                obj.prev_value = zeros(size(obj.rate_min));
            else
                obj.prev_value = value;
            end
        end
        
        function value = update(obj, dt, cur)
            value = obj.clamp(cur, ...
                              obj.prev_value + dt * obj.rate_min, ...
                              obj.prev_value + dt * obj.rate_max);
            obj.prev_value = value;
        end
        
        function value = getValue(obj)
            value = obj.prev_value;
        end
        
        function cur = clamp(~, cur, lower, upper)
            for i = 1:length(cur)
                if cur(i) < lower(i)
                    cur(i) = lower(i);
                elseif cur(i) > upper(i)
                    cur(i) = upper(i);
                end
            end
        end
        
    end
    
end