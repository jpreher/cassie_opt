classdef standOutputs < handle
    
    properties
        gam
        ep
        
        ya
        dya
        
        F
        G
        Q
        P
        Pep
    end
    
    methods
        
        function obj = standOutputs()
            ny = 6;
            obj.ep = 1 / 20;
            
            % Compute eta and the Lyapunov function
            obj.F = [zeros(ny),eye(ny);
                     zeros(ny),zeros(ny)];
            obj.G = [zeros(ny);eye(ny)];
            obj.Q = eye(2*ny);
            obj.Q(1,1) = 10;
            obj.Q(2,2) = 10;
            [obj.P,~,~] = care(obj.F,obj.G,obj.Q);
            
            obj.gam = min(eig(obj.Q))/ max(eig(obj.P)) / obj.ep;

            Iep = blkdiag(1 / obj.ep * eye(ny), eye(ny));
            obj.Pep = Iep' * obj.P * Iep;
            
        end
        
        function [A_mat, Lf_mat, psi0, psi1, y] = update(obj, q, dq, dyn, param)
            % Compute the outputs
            if coder.target('MATLAB')
                obj.ya = frost_expr.outputs.ya_stand(q);
                obj.dya   = frost_expr.outputs.dya_stand(q,dq);
                Dya   = frost_expr.outputs.Dya_stand(q);
                DLfya = frost_expr.outputs.DLfya_stand(q,dq);
            else
                % Not implemented
                error('Codegen not supported!');
            end
            
            y = obj.ya - param.yd_qp;
            dy = obj.dya - param.dyd_qp;
            
            eta = [y; dy];
            Veta = eta' * obj.Pep * eta;
            
            % Construct the control constraint
            LfV = eta'*(obj.F'*obj.Pep + obj.Pep*obj.F)*eta;
            LgV = 2*eta'*obj.Pep*obj.G;

            psi0 = LfV + obj.gam*Veta;
            psi1 = LgV';
            
            % Get dynamics
            Bbar = [dyn.Be, dyn.Jc'];
            gfc = [zeros(size(Bbar));
                   dyn.De \ Bbar];
            vfc = [dq; 
                   dyn.De \ (dyn.Fspring - dyn.Cvec - dyn.Gvec)];

            Lf_mat = DLfya*vfc;
            A_mat  = DLfya*gfc; % if doing rd1 need more...
            
        end
        
    end
end

