classdef CassieLogger < handle
    %UNTITLED5 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        flow = [];
        QP = [];
    end
    
    methods
        function obj = CassieLogger()
            obj.flow = struct('t', [], 'q', [], 'dq', [],'u',[], 'F', [], ...
                'tau', [], 'dtau', [], 'ya', [], 'dya', [], 'yd', [], 'dyd', [], 'Veta', [], ...
                'vel_des', [], 'vel_avg', [], 'vel_step_avg', [], 'deltaQP', [], 'FQP', []);
            obj.QP = struct('exitflag', [],'numiter',[]);
        end
    end
end

