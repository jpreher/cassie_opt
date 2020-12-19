classdef Timer < handle
    % Timer
    %   Timer which can be used to track both a real-time system and a
    %   time-scaled system concurrently. Timescale is one by assumption and
    %   must manually be changed.
    
    properties 
        t0            % Value of initializing timestamp.
        timeScale     % Scaling factor for timer
        tPrevious     % Last timestamp
        dtRealTime    % Size of last real-time step
        dtScaled      % Size of last time step with scaling applied
        timerRealTime % Real-time timer value
        timerScaled   % Scaled timer value
        hasFirst
    end
    
    methods
        function obj = Timer
            % Construct the class and initialize all of the class values to
            % zero. Time scale is always 1 on construction and must be
            % manually changed.
            
            obj.timeScale = 1;
            obj.reset();
        end
        
        function [] = overrideTimeScale(obj, timeScale)
            % Manually override the timescale.
            
            obj.timeScale = clamp(timeScale, 0, 2);
        end
                
        function [] = reset(obj, time)
            % Hard reset the timer class. Does not affect the timescale.
            
            if nargin < 2
                obj.hasFirst = false;
                time = 0;
            end
            
            obj.t0            = time;
            obj.tPrevious     = time;
            obj.dtRealTime    = 0;
            obj.dtScaled      = 0;
            obj.timerRealTime = 0;
            obj.timerScaled   = 0;
        end
        
        function [] = startTimer(obj, currentTime)
            % Reset the timer, then initialize the counter to start from
            % the supplied timestamp.
            
            obj.reset();
            obj.t0        = currentTime;
            obj.tPrevious = currentTime;
        end
        
        function [] = update(obj, currentTime)   
            % Update the timer according to the supplied timestamp.
            
            if ~obj.hasFirst
                obj.t0        = currentTime;
                obj.tPrevious = currentTime;
                obj.hasFirst = true;
            end
            
            obj.dtRealTime = currentTime - obj.tPrevious;
            obj.dtScaled   = (currentTime - obj.tPrevious) * obj.timeScale;
            obj.timerRealTime = currentTime - obj.t0;
            obj.timerScaled = obj.timerScaled + obj.dtScaled;
            obj.tPrevious = currentTime;
        end
        
    end
    
end

