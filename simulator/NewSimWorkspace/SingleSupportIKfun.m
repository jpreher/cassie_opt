classdef SingleSupportIKfun < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        qcur
        dqcur
        X
        sol
        tol
                
        leftStance
        
        iRotor
        iJoint
    end
    
    methods
        function obj = SingleSupportIKfun()
            obj.qcur = zeros(22,1);
            
            obj.X = zeros(9,1);
            
            obj.leftStance = false;
            
            obj.iRotor = [CassieStateEnum.LeftHipRoll; 
                          CassieStateEnum.LeftHipYaw;
                          CassieStateEnum.LeftHipPitch;
                          CassieStateEnum.LeftKneePitch;
                          CassieStateEnum.LeftToePitch;
                          CassieStateEnum.RightHipRoll;
                          CassieStateEnum.RightHipYaw;
                          CassieStateEnum.RightHipPitch;
                          CassieStateEnum.RightKneePitch;
                          CassieStateEnum.RightToePitch];
            obj.iJoint = [CassieStateEnum.LeftShinPitch;
                          CassieStateEnum.LeftTarsusPitch;
                          CassieStateEnum.RightShinPitch;
                          CassieStateEnum.RightTarsusPitch];
        end
        
        function [F,J] = evaluate(obj,x)
            F = obj.evaluateResidual(x);
            if nargout > 1
                J = obj.evaluateJacobian(x);
            end
        end
        
        function F = evaluateResidual(obj, x)
            q = obj.qcur;
            q(obj.iJoint) = [0; deg2rad(13) - q(CassieStateEnum.LeftKneePitch); 
                             0; deg2rad(13) - q(CassieStateEnum.RightKneePitch)];
            
            if obj.leftStance
                % Is left single support
                posefoot0 = codegen.kinematics.pose_leftFoot(q);
                
                q(obj.iJoint) = [0; deg2rad(13) - x(4); 0; deg2rad(13) - x(8)];
                iActive = obj.iRotor([1:4, 6:10]);
                q(iActive) = x;
                posefoot = codegen.kinematics.pose_leftFoot(q);
                roll  = q(4) + posefoot0(4) - posefoot(4);
                pitch = q(5) + posefoot0(5) - posefoot(5);
                
                ya = codegen.outputs.yaLeftStance(q);
            else
                % Is right single support
                posefoot0 = codegen.kinematics.pose_rightFoot(q);
                
                q(obj.iJoint) = [0; deg2rad(13) - x(4); 0; deg2rad(13) - x(9)];
                iActive = obj.iRotor(1:9);
                q(iActive) = x;
                posefoot = codegen.kinematics.pose_rightFoot(q);
                roll  = q(4) + posefoot0(4) - posefoot(4);
                pitch = q(5) + posefoot0(5) - posefoot(5);
                
                ya = codegen.outputs.yaRightStance(q);
            end
            
            ya(1) = roll;
            ya(2) = pitch;
            
            F = ya - obj.X;
        end
        
        function J = evaluateJacobian(obj, x)
            q = obj.qcur;
            
            if obj.leftStance
                % Is left single support
                %q(obj.iJoint) = [0; deg2rad(13) - x(4); 0; deg2rad(13) - x(8)];
                iActive = obj.iRotor([1:4, 6:10]);
                mActive = [1:4, 6:10];
                q(iActive) = x;
                Jy = codegen.outputs.J_yaLeftStance(q);
            else
                % Is right single support
                %q(obj.iJoint) = [0; deg2rad(13) - x(4); 0; deg2rad(13) - x(9)];
                iActive = obj.iRotor(1:9);
                mActive = 1:9;
                q(iActive) = x;
                Jy = codegen.outputs.J_yaRightStance(q);
            end
            J = Jy(:,mActive);
        end
        
        function dJ = evaluateJacobianDerivative(obj, x, dx)
            q = obj.qcur;
            dq = obj.dqcur;
            
            if obj.leftStance
                % Is left single support
                %q(obj.iJoint) = [0; deg2rad(13) - x(4); 0; deg2rad(13) - x(8)];
                %dq(obj.iJoint) = [0; - dx(4); 0; - dx(8)];
                iActive = obj.iRotor([1:4, 6:10]);
                mActive = [1:4, 6:10];
                q(iActive) = x;
                dq(iActive) = dx;
                Jdoty = codegen.outputs.Jdot_yaLeftStance(q,dq);
            else
                % Is right single support
                %q(obj.iJoint) = [0; deg2rad(13) - x(4); 0; deg2rad(13) - x(9)];
                %dq(obj.iJoint) = [0; - dx(4); 0; - dx(9)];
                iActive = obj.iRotor(1:9);
                mActive = 1:9;
                q(iActive) = x;
                dq(iActive) = dx;
                Jdoty = codegen.outputs.Jdot_yaRightStance(q,dq);
            end
            dJ = Jdoty(:,mActive);
        end
        
        function [] = setDesired(obj, desired, leftStance)
            obj.X = desired;
            obj.leftStance = leftStance;
        end
        
    end
end

