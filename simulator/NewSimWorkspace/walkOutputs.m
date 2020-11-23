classdef walkOutputs < handle
    
    properties
        gam
        ep
        
        y
        dy
        
        ya
        dya
        d2ya
        Dya
        DLfya
        Jya
        
        yd
        dyd
        d2yd
        
        F
        G
        Q
        P
        Pep
        Veta
    end
    
    methods
        
        function obj = walkOutputs(param)
            ny = 9;
            obj.ep = param.ep;
            
            % Compute eta and the Lyapunov function
            obj.F = [zeros(ny), eye(ny);
                     zeros(ny), zeros(ny)];
            obj.G = [zeros(ny); eye(ny)];
            obj.Q = blkdiag(eye(ny), 0.1 .* eye(ny));
            [obj.P,~,~] = care(obj.F,obj.G,obj.Q);
            
            obj.gam = min(eig(obj.Q))/ max(eig(obj.P)) / obj.ep;

            Iep = blkdiag(1 / obj.ep * eye(ny), eye(ny));
            obj.Pep = Iep' * obj.P * Iep;
            
        end
        
        function [psi0, psi1, LFV, LGV] = update(obj, q, dq, param)
            % Compute the outputs
            aMat = param.gait_param.aposition;            
            obj.yd   = bezier(aMat, param.phase.tau);
            obj.dyd  = dbezier(aMat, param.phase.tau) * param.phase.dtau;
            obj.d2yd = d2bezier(aMat, param.phase.tau) * param.phase.dtau^2;
            
            % obj.computeFootPlacement(q, dq, param);
                        
            if strcmp(param.stance_leg, 'Left')
                obj.ya   = frost_expr.outputs.yaLeftStance(q);
                obj.dya  = frost_expr.outputs.dyaLeftStance(q,dq);
                obj.Jya  = frost_expr.outputs.J_yaLeftStance(q);
                obj.Dya      = frost_expr.outputs.Dya_LeftStanceActual(q);
                obj.DLfya    = frost_expr.outputs.DLfya_LeftStanceActual(q,dq);
            elseif strcmp(param.stance_leg, 'Right')
                obj.ya   = frost_expr.outputs.yaRightStance(q);
                obj.dya  = frost_expr.outputs.dyaRightStance(q,dq);
                obj.Jya  = frost_expr.outputs.J_yaRightStance(q);
                obj.Dya      = frost_expr.outputs.Dya_RightStanceActual(q);
                obj.DLfya    = frost_expr.outputs.DLfya_RightStanceActual(q,dq);
            end
            
            obj.y = obj.ya - obj.yd;
            obj.dy = obj.dya - obj.dyd;
            
            eta = [obj.y; obj.dy];
            obj.Veta = eta' * obj.Pep * eta;
            
            % Construct the control constraint
            LFV = eta'*(obj.F'*obj.Pep + obj.Pep*obj.F)*eta;
            LGV = 2*eta'*obj.Pep*obj.G;
            
            psi0 = LFV + obj.gam*obj.Veta;
            psi1 = LGV';
        end
        
        function [] = computeFootPlacement(obj, q, dq, param)
            
        vd = param.vd;
        va = param.velocityLP.filteredValue;
            
        % Update foot placement controller
        sl_b = [0,0,ones(1,4)];
        s_slow = bezier(sl_b, param.phase.tau);
        ds_slow = dbezier(sl_b, param.phase.tau)*param.phase.dtau;
        
        % foot placement in saggital plane
        sw_LA = 7;
        obj.yd(sw_LA) = obj.yd(sw_LA)   + (param.Kp_x*(va(1)-vd(1)) + param.sagittal_offset + param.Kd_x*(va(1) -param.v_prev(1)))*s_slow;
        obj.dyd(sw_LA) = obj.dyd(sw_LA) + (param.Kp_x*(va(2)-vd(2)) + param.sagittal_offset + param.Kd_x*(va(1) -param.v_prev(1)))*ds_slow;
        
        % % foot placement in frontal plane
        %dqy_b_avg_1 = (obj.lateral_velocity_weight*obj.dqy_b_fil+(1-obj.lateral_velocity_weight)*obj.dqy_b_start);
        %lateral_ftpl = (obj.Kfl_p*dqy_b_avg_1 + obj.Kfl_d*(dqy_b_avg_1 - obj.v_final_avgy) + abduction_direction*obj.init_lateral_velocity*median([0,1,obj.dqx_b_fil]))*min(1.5*s,1);
        
        
        end
        
    end
end

