classdef SingleSupportIKsolver < handle
    %WALKIKSOLVER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        IkFunction
        IkSolver
                
        X
        dX
        ddX
        
        rotor_lb
        rotor_ub
        
        leftStance
        
        iRotorMap
    end
    
    methods
        function obj = SingleSupportIKsolver()
            obj.IkFunction = SingleSupportIKfun();
            obj.IkSolver = LevenbergMarquardtSolver(obj.IkFunction, ...
                CassieCore.motorPositionLowerLimit(1:9), ...
                CassieCore.motorPositionUpperLimit(1:9));
            
            obj.X = zeros(9,1);
            obj.dX = zeros(9,1);
            
            obj.leftStance = false;
            
            obj.iRotorMap = [CassieStateEnum.LeftHipRoll; 
                             CassieStateEnum.LeftHipYaw;
                             CassieStateEnum.LeftHipPitch;
                             CassieStateEnum.LeftKneePitch;
                             CassieStateEnum.LeftToePitch;
                             CassieStateEnum.RightHipRoll;
                             CassieStateEnum.RightHipYaw;
                             CassieStateEnum.RightHipPitch;
                             CassieStateEnum.RightKneePitch;
                             CassieStateEnum.RightToePitch];
                         
            obj.rotor_lb = [-0.3491   -0.3840   -0.8727   -2.7227   -2.4435   -0.3491   -0.3840   -0.8727   -2.7227   -2.4435];
            obj.rotor_ub = [0.3491    0.3840    1.3963   -0.7330   -0.6109    0.3491    0.3840    1.3963   -0.7330   -0.6109];
        end
        
        function [x, dx, ddx] = evaluate(obj, q, dq)
            % Get active motors (turn off stance)
            x0 = q(obj.iRotorMap);
            dx0 = dq(obj.iRotorMap);
            if obj.leftStance
                iActive = [1:4, 6:10];
                iPassive = 5;
            else
                iActive = 1:9;
                iPassive = 10;
            end
            obj.IkFunction.qcur = q;
            obj.IkFunction.dqcur = q;
            obj.IkFunction.setDesired(obj.X, obj.leftStance);
            
            % Run solver
            obj.IkSolver.lb = obj.rotor_lb(iActive);
            obj.IkSolver.ub = obj.rotor_ub(iActive);
            obj.IkSolver.solve(x0(iActive));
            
            % Extract result
            x = zeros(10,1);
            x(iActive) = obj.IkSolver.getSolution();
            x(iPassive) = x0(iPassive);
            
            % Compute desired velocities
            if nargout > 1
                J = obj.IkFunction.evaluateJacobian(x(iActive));
                dx = zeros(10,1);
                dx(iActive) = J \ obj.dX;
                dx(iPassive) = dx0(iPassive);
            end
            if nargout > 2
                dJ = obj.IkFunction.evaluateJacobianDerivative(x(iActive), dx(iActive));
                ddx = zeros(10,1);
                ddx(iActive) = J \ (obj.ddX - dJ*dx(iActive));
                ddx(iPassive) = 0;
            end
        end
    end
end

