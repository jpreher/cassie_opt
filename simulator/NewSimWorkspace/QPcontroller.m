classdef QPcontroller < handle
    
    properties
        % Output class
        outputs
        contact_pyramid
        lbu
        ubu
        u_bar_last
        
        step_has_raibert
        final_aposition
        v_avg_allocator
        v_avg_count
    end
    
    methods
        function obj = QPcontroller(outputs)
            % Assign the output
            obj.outputs = outputs;
            
            % Pyramidal friction cone approximation constraints
            % fz >= 0
            % |fx|,|fy| <= mu/sqrt(2) * |fz|
            % -mu * foot_length * fz < wz
            mu_ = 0.8;
            foot_length = 0.14;
            obj.contact_pyramid = [0, 0, -1, 0, 0;
                1, 0, -mu_/sqrt(2), 0, 0;
                -1, 0, -mu_/sqrt(2), 0, 0;
                0, 1, -mu_/sqrt(2), 0, 0;
                0,-1, -mu_/sqrt(2), 0, 0;
                0, 0 -foot_length*mu_, 0, 1;
                0, 0 -foot_length*mu_, 0,-1;
                0, 0 -foot_length*mu_, 1, 0;
                0, 0 -foot_length*mu_,-1, 0];
            
            % Torque constr aints
            torque_bounds = repmat([4.5, 4.5, 12.2, 12.2, 0.9], 1, 2)';
            obj.lbu = -torque_bounds;
            obj.ubu = +torque_bounds;
            
            obj.u_bar_last = [];
            
            % flags
            obj.step_has_raibert = false;
            obj.v_avg_allocator = [0;0];
            obj.v_avg_count = 0;
        end
        
        function [u, F, delta] = update(obj, q, dq, dyn, param)
            % Update the phase variable
            param.phase.update(param.timer);
            if param.phase.tau > 1.0
                param.phase.tau = 1.0;
            end
            
            % Update Raibert foot placement
            %obj.raibert(param);
            
            % Update the outputs
            [psi0, psi1, LGV, LFV] = obj.outputs.update(q, dq, param);
            
            % Generate QP torques
            controller = 'new_proj'; %'trad', 'proj', 'IO', 'new'
            if strcmp(controller, 'trad')
                [u, F, delta] = obj.traditional_clfqp(q, dq, psi0, psi1, dyn, param);
            elseif strcmp(controller, 'proj')
                [u, F, delta] = obj.projection_clfqp(q, dq, psi0, psi1, dyn, param);
            elseif strcmp(controller, 'new')
                [u, F, delta] = obj.new_clfqp(q, dq, LGV, LFV, psi0, psi1, dyn, param);
            elseif strcmp(controller, 'new_proj')
                [u, F, delta] = obj.new_proj_clfqp(q, dq, LGV, LFV, psi0, psi1, dyn, param);
            elseif strcmp(controller, 'IO')
                u = obj.IO_control(q, dq, psi0, psi1, dyn, param);
                delta = 0;
                F = [];
            elseif strcmp(controller, 'new_cost')
                [u, F] = obj.new_clfqp_cost(q, dq, LGV, LFV, psi0, psi1, dyn, param);
                delta = 0;
            end
        end
        
        function [] = new_step(obj)
            obj.step_has_raibert = false;
        end
        
        function [] = raibert(obj, param)
            % At tau == 0.5 update parameters according to Dennis MARLO paper
            if param.phase.tau >= 0.50 && ~obj.step_has_raibert
                obj.step_has_raibert = true;
                
                % Get the average velocity over the current 1/2 step
                v_avg = obj.v_avg_allocator / obj.v_avg_count;
                v_avg(2) = param.avg_vel(2); % Override the lateral to use stepwise
                
                % Switch to current speed parameters
                param.gait_param = param_lib(v_avg(1), param.stance_leg, param.interp_lib);
                
                % Get the new impact desireds
                yF   = bezier(param.gait_param.aposition, 1);
                
                % Update foot placement with Raibert and motion transition
                Kp_raibert = [0.38;  0.42];
                Kd_raibert = [0.15;  0.15];
                
                offset = -Kp_raibert .* (v_avg - param.vd) - Kd_raibert .* (v_avg - param.prev_avg_vel);
                
                if param.k_step > 1
                    % Get the pitch and roll offsets for the swing leg
                    if strcmp(param.stance_leg, 'Right')
                        offset(2) = clamp(offset(2), -1.0, 0);
                        yF(1) = yF(1) - offset(2);
                    elseif strcmp(param.stance_leg, 'Left')
                        offset(2) = clamp(offset(2), 0, 1.0);
                        yF(1) = yF(1) - offset(2);
                    end
                    yF(7) = yF(7) + offset(1);
                    
                    % Compute a leg extension to still strike the ground
                    %yF(5) = yF(5) + sin(norm(offset)) * yF(5);
                    
                    % Transition it in
                    param.gait_param.aposition = computeEndTransition( param.phase.dtau, param.gait_param.aposition, yF );
                else
                    %param.gait_param.aposition = param.gait_param.aposition;
                end
                
                obj.v_avg_allocator = [0;0];
                obj.v_avg_count = 0;
                param.prev_avg_vel = v_avg;
            end
            
            % Update the current parameters
            %param.phase.update(param.timer);
            %s_param = 1 - clamp( (param.phase.tau - 0.5) / (0.70 - 0.5), 0, 1 );
            %param.aMat = s_param * param.gait_param.aposition + (1 - s_param) * obj.final_aposition;
        end
        
        function [u, F, delta] = projection_clfqp(obj, q, dq, psi0, psi1, dyn, param)
            %%% Do new cost method
            %%% Run traditional clf-qp
            %             Dc = dyn.Su * dyn.Q' * dyn.De;
            %             Hc = dyn.Su * dyn.Q' * (dyn.Cvec + dyn.Gvec - dyn.Fspring);
            %             Bc = dyn.Su * dyn.Q' * dyn.Be;
            %
            %             gfc = [zeros(22,10);
            %                   Dc \ Bc];
            %             vfc = [dq;
            %                   Dc \ Hc];
            
            Fv = -dyn.Cvec - dyn.Gvec + dyn.Fspring;
            XiInv = dyn.Jc * (dyn.De \ transpose(dyn.Jc));
            gfc = [zeros(size(dyn.Be));
                dyn.De \ (eye(22) - transpose(dyn.Jc)* (XiInv \ (dyn.Jc / dyn.De))) * dyn.Be];
            vfc = [dq;
                dyn.De \ ((eye(22)-transpose(dyn.Jc) * (XiInv \ (dyn.Jc / dyn.De))) * (Fv) - transpose(dyn.Jc) * (XiInv \ dyn.dJc * dq))];
            
            % Contact conditions
            lambda_0 = XiInv \ (dyn.dJc * dq + dyn.Jc * (dyn.De \ Fv)); %  + Gv
            A0 = - XiInv \ (dyn.dJc * dq + dyn.Jc * inv(dyn.De) * dyn.Be);
            Acontact = obj.contact_pyramid * A0(5:end,:);
            bcontact = obj.contact_pyramid * lambda_0(5:end);
            if param.phase.tau > 0.9
                bcontact = inf * ones(size(bcontact));% bcontact * 1000000;
            end
            
            Lf_mat = obj.outputs.DLfya*vfc;
            A_mat  = obj.outputs.DLfya*gfc;
            
            % The CLF convergence rate must be satisfied
            % psi0 + psi1'*(A_hat*u_hat + Lf) <= delta
            A_clf = [psi1'*A_mat, -1];
            b_clf = -psi1'*(Lf_mat - obj.outputs.d2yd) - psi0;
            
            % Construct the cost
            % 0.5*x'*H*x + f'*x
            H_clf = (A_mat' * A_mat);
            f_clf = 2 * (Lf_mat' * A_mat - obj.outputs.d2yd'*A_mat);
            
            H_slack = param.w_slack;
            f_slack = 0;
            
            % Assemble and solve
            Hmat = blkdiag(H_clf, H_slack);
            fmat = [f_clf, f_slack]';
            
            % Update the u bounds according to stance foot
            if strcmp(param.stance_leg,'Left')
                lb_u = obj.lbu; lb_u(5) = 0;
                ub_u = obj.ubu; ub_u(5) = 0;
            elseif strcmp(param.stance_leg,'Right')
                lb_u = obj.lbu; lb_u(10) = 0;
                ub_u = obj.ubu; ub_u(10) = 0;
            end
            
            lb = [lb_u;
                -inf];
            ub = [ub_u;
                inf];
            
            A = [A_clf;
                [Acontact, zeros(size(Acontact,1),1)]];
            
            lbA = [-inf * ones(1,1);
                -inf * ones(size(bcontact))];
            ubA = [b_clf;
                bcontact];
            
            if isempty(obj.u_bar_last)
                obj.u_bar_last = zeros(size(lb));
            end
            
            aux_data = qpOASES_auxInput('x0', obj.u_bar_last);
            options = qpOASES_options('maxIter', 5000, 'printLevel', 1);
            
            [u_bar, fval, exitflag, numiter] = qpOASES(Hmat, fmat, A, lb, ub, lbA, ubA, options, aux_data);
            
            if exitflag ~= 0
                %warning('The QP did not converge!');
                u = obj.IO_control(q, dq, psi0, psi1, dyn, param);
                delta = 0;
                F = [];
                u_bar = [u; delta];
                %[u_bar, fval, exitflag, numiter] = qpOASES(Hmat, fmat, A, lb, ub, lbA, ubA, options, aux_data);
                obj.u_bar_last = u_bar;
            else
                obj.u_bar_last = u_bar;
            end
            
            u = u_bar(1:10);     % Control
            F = u_bar(11:end-1); % Constraint forces
            delta = u_bar(end);
        end
        
        function [u, F, delta] = traditional_clfqp(obj, q, dq, psi0, psi1, dyn, param)
            %%% Run traditional clf-qp
            Bbar = [dyn.Be, dyn.Jc'];
            gfc = [zeros(size(Bbar));
                dyn.De \ Bbar];
            vfc = [dq;
                dyn.De \ (dyn.Fspring - dyn.Cvec - dyn.Gvec)];
            
            Lf_mat = obj.outputs.DLfya*vfc;
            A_mat  = obj.outputs.DLfya*gfc;
            
            % The CLF convergence rate must be satisfied
            % psi0 + psi1'*(A_hat*u_hat + Lf) <= delta
            A_clf = [psi1'*A_mat, -1];
            b_clf = -psi1'*(Lf_mat- obj.outputs.d2yd) - psi0;
            
            n_vars = size(A_clf,2);
            
            % Holonomic constraints must be satisfied as equality
            % Jcdot*dq + Jc * ( De \ (Bbar*u_bar - Ce*dq - Ge + Fsprings) ) = 0;
            Aeq_holonomic = dyn.Jc * ( dyn.De \ Bbar );
            beq_holonomic = dyn.Jc * ( dyn.De \ (dyn.Cvec + dyn.Gvec - dyn.Fspring) ) - dyn.dJc * dq;
            
            % Friction cone feasibility constraints must be satisfied
            n_cone = size(obj.contact_pyramid,1);
            if size(dyn.Jc,1) > 19
                % We are in double support
                Aineq_contact = zeros(2*n_cone, n_vars);
                bineq_contact = zeros(2*n_cone,1);
                Aineq_contact(: , n_vars-10:n_vars-1) = blkdiag(obj.contact_pyramid, obj.contact_pyramid);
            else
                % We are in single support
                Aineq_contact = zeros(n_cone, n_vars);
                bineq_contact = zeros(n_cone,1);
                Aineq_contact(: , n_vars-5:n_vars-1) = obj.contact_pyramid;
            end
            
            if param.phase.tau > 0.9
                bineq_contact = inf * ones(size(bineq_contact));% bcontact * 1000000;
            end
            
            % Construct the cost
            % 0.5*x'*H*x + f'*x
            alpha = 1;
            ATA = A_mat' * A_mat;
            H_clf = alpha * (ATA) + (1-alpha) * eye(size(ATA,1));
            f_clf = alpha * 2 * (Lf_mat' * A_mat - obj.outputs.d2yd'*A_mat);
            
            H_slack = param.w_slack;
            f_slack = 0;
            
            % Assemble and solve
            Hmat = blkdiag(H_clf, H_slack);
            fmat = [f_clf, f_slack]';
            
            % Update the u bounds according to stance foot
            if strcmp(param.stance_leg,'Left')
                lb_u = obj.lbu; lb_u(5) = 0;
                ub_u = obj.ubu; ub_u(5) = 0;
            elseif strcmp(param.stance_leg,'Right')
                lb_u = obj.lbu; lb_u(10) = 0;
                ub_u = obj.ubu; ub_u(10) = 0;
            end
            
            lb = [lb_u;
                -inf * ones(size(dyn.Jc,1),1);
                -inf];
            ub = [ub_u;
                inf * ones(size(dyn.Jc,1),1);
                inf];
            
            A = [[Aeq_holonomic, zeros(size(Aeq_holonomic,1),1)]
                A_clf;
                Aineq_contact];
            
            lbA = [beq_holonomic;
                -inf * ones(1,1);
                -inf * ones(size(bineq_contact))];
            ubA = [beq_holonomic;
                b_clf;
                bineq_contact];
            
            if isempty(obj.u_bar_last)
                obj.u_bar_last = zeros(size(lb));
            end
            
            aux_data = qpOASES_auxInput('x0', obj.u_bar_last);
            options = qpOASES_options('maxIter', 5000, 'printLevel', 1);
            [u_bar, fval, exitflag, numiter] = qpOASES(Hmat, fmat, A, lb, ub, lbA, ubA, options, aux_data);
            
            %             t1 = obj.outputs.DLfya * (dyn.De \ ( Bbar * u_bar(1:19) + dyn.Fspring - dyn.Cvec - dyn.Gvec))
            %             t2 = obj.outputs.DLfya * (dyn.De \ ( Bbar * u_bar(1:19) )  + obj.outputs.DLfya * dyn.De \ (dyn.Fspring - dyn.Cvec - dyn.Gvec))
            
            if exitflag ~= 0
                %warning('The QP did not converge!');
                [u_bar, fval, exitflag, numiter] = qpOASES(Hmat + eye(size(Hmat)), fmat, [], lb, ub, [], [], options, aux_data);
                if exitflag ~= 0
                    obj.u_bar_last = [];
                else
                    obj.u_bar_last = u_bar;
                end
            else
                obj.u_bar_last = u_bar;
            end
            
            u = u_bar(1:10);     % Control
            F = u_bar(11:end-1); % Constraint forces
            delta = u_bar(end);
        end
        
        function u = IO_control(obj, q, dq, ~, ~, dyn, ~)
            %%% Run traditional IO
            Fv = -dyn.Cvec - dyn.Gvec + dyn.Fspring;
            Fv = Fv * 0.9;
            XiInv = dyn.Jc * (dyn.De \ transpose(dyn.Jc));
            gfc = [zeros(size(dyn.Be));
                dyn.De \ (eye(22) - transpose(dyn.Jc)* (XiInv \ (dyn.Jc / dyn.De))) * dyn.Be];
            vfc = [dq;
                dyn.De \ ((eye(22)-transpose(dyn.Jc) * (XiInv \ (dyn.Jc / dyn.De))) * (Fv) - transpose(dyn.Jc) * (XiInv \ dyn.dJc * dq))];
            
            Lf_mat = obj.outputs.DLfya*vfc;
            A_mat  = obj.outputs.DLfya*gfc;
            
            Kp = 400;
            Kd = 25;
            
            mu = Kp * (obj.outputs.ya - obj.outputs.yd) + Kd * (obj.outputs.dya - obj.outputs.dyd);
            
            u_ff = - A_mat \ (Lf_mat - obj.outputs.d2yd);
            u_fb = -A_mat \ mu;
            u = u_ff + u_fb;
        end
        
        function [u, F, delta] = new_clfqp(obj, q, dq, LFV, LGV, psi0, psi1, dyn, param)
            % Make sure initial guess is not empty
            if isempty(obj.u_bar_last)
                obj.u_bar_last = zeros(42,1);
            end
            
            % Dynamics Equality Constraint
            He = dyn.Cvec + dyn.Gvec;
            Aeq_dyn = [ dyn.De, -dyn.Be, -dyn.Jc', zeros(22,1) ];
            beq_dyn = - He + dyn.Fspring;
            
            % Holonomic Constraints
            A_hols = [dyn.Jc, zeros(9,10), zeros(9,9), zeros(9,1)];
            b_hols = - dyn.dJc * dq;
            
            % Friction Cone Inequality Constr1aint
            bcontact = zeros(9,1);
            % if param.phase.tau > 0.9
            % bcontact(2:7) = inf;% bcontact * 1000000;
            % end
            P = [zeros(9,2), ... % No constraint on the first 4 holonomic wrenches
                obj.contact_pyramid, ...
                zeros(9,2)]; % Constraints on GRF only
            A_friction = [ zeros(9,22), zeros(9,10), P, zeros(9,1) ];
            b_lb_fric = -inf * ones(9,1);
            b_ub_fric = inf * ones(9,1);%bcontact;
            
            % CLF Convergence Constraint
            A_clf = [ LGV*obj.outputs.Dya(:,1:22), zeros(1,10), zeros(1,9), -1 ];
            b_lb_clf = -inf;
            b_ub_clf = +inf;
            b_ub_clf = - obj.outputs.gam*obj.outputs.Veta - LFV - LGV * obj.outputs.DLfya(:,1:22) * dq; %  b_clf;
            
            % Regularization
            A_reg = eye(42);
            b_reg = zeros(42,1);
            
            ddqd = bezier(param.gait_param.ddqd, param.phase.tau);
            %b_reg(1:22) = dyn.JCc \ (- dyn.dJCc * dq) + (eye(22) + dyn.JCc \ dyn.JCc) * ddqd;
            b_reg(1:22) = ddqd;
            
            fd = bezier(param.gait_param.fd, param.phase.tau);
            b_reg(33:39) = fd;
            
            % Output PD control + virtual holonomic
            Kpy = [1600, 800, 400, 1200, 1000, 400, 400, 400, 750]';
            Kdy = [25, 16, 5, 20, 18, 6, 6, 6, 25]';%25;
            
            %B  = zeros(22,9);
            %By = zeros(10,9);
            %By(1:4,1:4) = eye(4);
            %if strcmp(param.stance_leg,'Left')
            %    By(6:10, 5:9) = eye(5);
            %    B = dyn.Be(:, [1:4,6:10]);
            %else
            %    By(5:9, 5:9) = eye(5);
            %    B = dyn.Be(:, 1:9);
            %end
            %P = eye(22) - pinv(dyn.JCc)* dyn.JCc;
            %ddy = - ( pinv(P * B) * P ) * (B * (Kpy.*obj.outputs.y + Kdy.*obj.outputs.dy));
            
            A_y = [obj.outputs.Dya(:,1:22), zeros(9,10), zeros(9,9), zeros(9,1)];
            %b_y = obj.outputs.d2yd - obj.outputs.DLfya(:,1:22) * dq + ddy;
            b_y = obj.outputs.d2yd - obj.outputs.DLfya(:,1:22) * dq;% - Kpy.*obj.outputs.y - Kdy.*obj.outputs.dy;
            
            % Update the u bounds according to stance foot
            % and smooth with previous torque
            if strcmp(param.stance_leg,'Left')
                lb_u = obj.lbu; lb_u(5) = 0;
                ub_u = obj.ubu; ub_u(5) = 0;
            elseif strcmp(param.stance_leg,'Right')
                lb_u = obj.lbu; lb_u(10) = 0;
                ub_u = obj.ubu; ub_u(10) = 0;
            end
            u_prev = obj.u_bar_last(23:32);
            A_u_chat = [zeros(10,22), eye(10), zeros(10,9), zeros(10,1)];
            b_u_chat = u_prev;
            %b_u_chat = pinv(dyn.Su*dyn.Q'*dyn.Be) * dyn.Su * dyn.Q' * (dyn.De*ddqd + He ) - pinv(obj.outputs.Jya) * (Kpy.*obj.outputs.y + Kdy.*obj.outputs.dy);
            
            
            
            
            
            % Build full constraint matrices
            lb = [-inf * ones(22,1); %ddq
                lb_u;                %torque
                -inf * ones(9,1);   %forces
                -inf];               %delta
            ub = [+inf * ones(22,1); %ddq
                ub_u;                %torque
                +inf * ones(9,1);   %forces
                +inf];               %delta
            Acon = [Aeq_dyn;
                A_friction;
                A_clf];
            lbAcon = [beq_dyn;
                b_lb_fric;
                b_lb_clf];
            ubAcon = [beq_dyn;
                b_ub_fric;
                b_ub_clf];
            
            % Construct the cost
            % 0.5*x'*H*x + f'*x
            % Assemble and solve
            w_ach_hol = 10;
            w_rigidsp_hol = 5;
            w_contact_hol = 10;
            w_hol = [w_ach_hol * ones(2,1);
                w_contact_hol * ones(5,1);
                w_rigidsp_hol * ones(2,1)];
            w_u_chatter = 1;
            w_reg = [1e-2 * ones(22,1); % ddq
                1e-1 * ones(10,1); % u
                1e-4 * ones(2,1); % lambda_ach
                1e-4 * ones(2,1); % fx fy
                1e-4 * ones(1,1); % fz
                1e-4 * ones(2,1); % muy, muz
                1e-5 * ones(2,1); % lambda_rigid
                param.w_slack * ones(1,1)]; % delta
            w_y = 1;
            %             w = [w_hol; w_u_chatter.*ones(10,1); w_y.*ones(9,1); w_reg];
            
            %             w_ach_hol = 10;
            %             w_rigidsp_hol = 5;
            %             w_contact_hol = 10;
            %             w_hol = [w_ach_hol * ones(2,1);
            %                      w_rigidsp_hol * ones(2,1);
            %                      w_contact_hol * ones(5,1)];
            %             w_u_chatter = 1e-1;
            %             w_reg = [1e-5 * ones(22,1); % ddq
            %                      1e-1 * ones(10,1); % u
            %                      1e-5 * ones(2,1); % lambda_ach
            %                      1e-5 * ones(2,1); % lambda_rigid
            %                      1e-3 * ones(2,1); % fx fy
            %                      1e-5 * ones(1,1); % fz
            %                      1e-3 * ones(2,1); % muy, muz
            %                      param.w_slack * ones(1,1)]; % delta
            %             w_y = 1;
            
            
            A = [w_hol .* A_hols;
                w_u_chatter .* A_u_chat;
                w_y .* A_y;
                w_reg .* A_reg];
            b = [w_hol .* b_hols;
                w_u_chatter .* b_u_chat;
                w_y .* b_y;
                w_reg .* b_reg];
            
            G = A'*A;
            g = -A'*b + 2.5 .* [(LGV*obj.outputs.Dya(:,1:22))'; zeros(20,1)];
            
            % Run QP
            aux_data = qpOASES_auxInput('x0', obj.u_bar_last);
            options = qpOASES_options('maxIter', 100, 'printLevel', 1, ...
                'enableRamping', false, 'enableFarBounds', true, 'enableFlippingBounds', false, ...
                'enableRegularisation', false, 'enableNZCTests', false, 'enableDriftCorrection', false, ...
                'enableEqualities', true, 'terminationTolerance', 1e9*eps, ...
                'numRegularisationSteps', 1, 'numRefinementSteps', 0);
            [u_bar, fval, exitflag, numiter] = qpOASES(G, g, Acon, lb, ub, lbAcon, ubAcon, options, aux_data);
            
            if exitflag ~= 0
                % If we did not converge, use IO controller
                u = obj.IO_control(q, dq, psi0, psi1, dyn, param);
                delta = 0;
                u_bar = [zeros(22,1); u; zeros(9,1); delta];
            end
            
            % Finalize
            obj.u_bar_last = u_bar;
            ddq = u_bar(1:22);
            u = u_bar(23:32);     % Control
            F = u_bar(33:42);
            delta = u_bar(end);
        end
        
        function [u, F, delta] = new_proj_clfqp(obj, q, dq, LFV, LGV, psi0, psi1, dyn, param)
            % Make sure initial guess is not empty
            if isempty(obj.u_bar_last)
                obj.u_bar_last = zeros(33,1);
            end
            
            % Get dynamics terms
            Paghili = eye(22) - pinv(dyn.JCc)*dyn.JCc;
            
            %He = (dyn.Cvec + dyn.Gvec - dyn.Fspring);
            He = (dyn.Cvec + dyn.Gvec);
            
            Mc = dyn.De + Paghili*dyn.De - (Paghili*dyn.De)';
            C = -pinv(dyn.JCc)*dyn.dJCc;
            Cc = dyn.De * C;
            Aeq_dyn = [ Mc, -Paghili*dyn.Be, zeros(size(Mc',1),1) ];
            beq_dyn = - Paghili * He + Cc*dq;
            
            % Friction Cone Inequality Constraint
            % F = pinv(dyn.Jc') * ( (eye(22) - Paghili) * dyn.De*ddq + He - dyn.Be * u )
            P = [zeros(9,4), ... % No constraint on the first 4 holonomic wrenches
                zeros(9,2), ...
                obj.contact_pyramid]; % Constraints on GRF only
            JcTinv = pinv(dyn.JCc');
            A_friction = [ P*JcTinv*(eye(22)-Paghili)*dyn.De, -P*JcTinv*dyn.Be , zeros(9,1) ];
            b_lb_fric = -inf * ones(9,1);
            b_ub_fric = -P*JcTinv*He;
            
            % CLF Convergence Constraint
            A_clf = [ LGV*obj.outputs.Dya(:,1:22), zeros(1,10), -1 ];
            b_lb_clf = -inf;
            b_ub_clf = inf;
            %b_ub_clf = - obj.outputs.gam*obj.outputs.Veta - LFV - LGV * obj.outputs.DLfya(:,23:44) * dq;
            
            % Update the u bounds according to stance foot
            % and smooth with previous torque
            if strcmp(param.stance_leg,'Left')
                lb_u = obj.lbu; lb_u(5) = 0;
                ub_u = obj.ubu; ub_u(5) = 0;
            elseif strcmp(param.stance_leg,'Right')
                lb_u = obj.lbu; lb_u(10) = 0;
                ub_u = obj.ubu; ub_u(10) = 0;
            end
            u_prev = obj.u_bar_last(23:32);
            A_u_chat = [zeros(10,22), eye(10), zeros(10,1)];
            b_u_chat = u_prev;
            
            % Output PD control + virtual holonomic
            Kpy = 500;%500;
            Kdy = 10;%25;
            A_y = [obj.outputs.Dya(:,1:22), zeros(9,10), zeros(9,1)];
            b_y = obj.outputs.d2yd - obj.outputs.DLfya(:,23:44) * dq - Kpy*obj.outputs.y - Kdy*obj.outputs.dy;
            
            % Regularization
            A_reg = eye(33);
            b_reg = zeros(33,1);
            
            ddqd = bezier(param.gait_param.ddqd, param.phase.tau);
            b_reg(1:22) = ddqd;
            
            fd = bezier(param.gait_param.fd, param.phase.tau);
            
            % Build full constraint matrices
            lb = [-inf * ones(22,1); %ddq
                lb_u;              %torque
                -inf];             %delta
            ub = [+inf * ones(22,1); %ddq
                ub_u;              %torque
                +inf];             %delta
            Acon = [Aeq_dyn;
                A_friction;
                A_clf];
            lbAcon = [beq_dyn;
                b_lb_fric;
                b_lb_clf];
            ubAcon = [beq_dyn;
                b_ub_fric;
                b_ub_clf];
            
            % Construct the cost
            % 0.5*x'*H*x + f'*x
            % Assemble and solve
            w_u_chatter = 0.01;
            w_reg = [0.001 * ones(22,1); % ddq
                0.1 * ones(10,1); % u
                1 * ones(1,1)]; % delta
            w_y = 1;
            
            A = [w_u_chatter .* A_u_chat;
                w_reg .* A_reg;
                w_y .* A_y];
            b = [w_u_chatter .* b_u_chat;
                w_reg .* b_reg;
                w_y .* b_y];
            
            G = A'*A;
            g = -A'*b;% + 2.5 .* [(LGV*obj.outputs.Dya(:,1:22))'; zeros(11,1)];
            
            % Run QP
            aux_data = qpOASES_auxInput('x0', obj.u_bar_last);
            options = qpOASES_options('maxIter', 500, 'printLevel', 1);
            [u_bar, fval, exitflag, numiter] = qpOASES(G, g, Acon, lb, ub, lbAcon, ubAcon, options, aux_data);
            
            if exitflag ~= 0
                % If we did not converge, use IO controller
                u = obj.IO_control(q, dq, psi0, psi1, dyn, param);
                delta = 0;
                u_bar = [zeros(22,1); u; delta];
            end
            
            % Finalize
            obj.u_bar_last = u_bar;
            ddq = u_bar(1:22);
            u = u_bar(23:32);     % Control
            %F(dyn.E,1) = dyn.R \ dyn.Sc * (QDe * ddq + QHe - QBe * u)
            F = pinv(dyn.JCc') * ( (eye(22) - Paghili) * dyn.De*ddq + He - dyn.Be * u );
            
            Fcon = F(5:end);
            delta = u_bar(end);
        end
        
        %         function [u, F, delta] = new_proj_clfqp(obj, q, dq, LFV, LGV, psi0, psi1, dyn, param)
        %             % Make sure initial guess is not empty
        %             if isempty(obj.u_bar_last)
        %                 obj.u_bar_last = zeros(33,1);
        %             end
        %
        %             % Get dynamics terms
        %             Paghili = eye(22) - pinv(dyn.JCc)*dyn.JCc;
        %
        %             %He = (dyn.Cvec + dyn.Gvec - dyn.Fspring);
        %             He = (dyn.Cvec + dyn.Gvec);
        %
        %             Mc = dyn.De + Paghili*dyn.De - (Paghili*dyn.De)';
        %             C = -pinv(dyn.JCc)*dyn.dJCc;
        %             Cc = dyn.De * C;
        %             Aeq_dyn = [ Mc, -Paghili*dyn.Be, zeros(size(Mc',1),1) ];
        %             beq_dyn = - Paghili * He + Cc*dq;
        %
        %             % Friction Cone Inequality Constraint
        %             % F = pinv(dyn.Jc') * ( (eye(22) - Paghili) * dyn.De*ddq + He - dyn.Be * u )
        %             P = [zeros(9,4), ... % No constraint on the first 4 holonomic wrenches
        %                  zeros(9,2), ...
        %                  obj.contact_pyramid]; % Constraints on GRF only
        %             JcTinv = pinv(dyn.JCc');
        %             A_friction = [ P*JcTinv*(eye(22)-Paghili)*dyn.De, -P*JcTinv*dyn.Be , zeros(9,1) ];
        %             b_lb_fric = -inf * ones(9,1);
        %             b_ub_fric = -P*JcTinv*He;
        %
        %             % CLF Convergence Constraint
        %             A_clf = [ LGV*obj.outputs.Dya(:,1:22), zeros(1,10), -1 ];
        %             b_lb_clf = -inf;
        %             b_ub_clf = inf;
        %             %b_ub_clf = - obj.outputs.gam*obj.outputs.Veta - LFV - LGV * obj.outputs.DLfya(:,23:44) * dq;
        %
        %             % Update the u bounds according to stance foot
        %             % and smooth with previous torque
        %             if strcmp(param.stance_leg,'Left')
        %                 lb_u = obj.lbu; lb_u(5) = 0;
        %                 ub_u = obj.ubu; ub_u(5) = 0;
        %             elseif strcmp(param.stance_leg,'Right')
        %                 lb_u = obj.lbu; lb_u(10) = 0;
        %                 ub_u = obj.ubu; ub_u(10) = 0;
        %             end
        %             u_prev = obj.u_bar_last(23:32);
        %             A_u_chat = [zeros(10,22), eye(10), zeros(10,1)];
        %             b_u_chat = u_prev;
        %
        %             % Output PD control + virtual holonomic
        %             Kpy = 500;%500;
        %             Kdy = 10;%25;
        %             A_y = [obj.outputs.Dya(:,1:22), zeros(9,10), zeros(9,1)];
        %             b_y = obj.outputs.d2yd - obj.outputs.DLfya(:,23:44) * dq - Kpy*obj.outputs.y - Kdy*obj.outputs.dy;
        %
        %             % Regularization
        %             A_reg = eye(33);
        %             b_reg = zeros(33,1);
        %
        %             ddqd = bezier(param.gait_param.ddqd, param.phase.tau);
        %             b_reg(1:22) = ddqd;
        %
        %             fd = bezier(param.gait_param.fd, param.phase.tau);
        %
        %             % Build full constraint matrices
        %             lb = [-inf * ones(22,1); %ddq
        %                 lb_u;              %torque
        %                 -inf];             %delta
        %             ub = [+inf * ones(22,1); %ddq
        %                 ub_u;              %torque
        %                 +inf];             %delta
        %             Acon = [Aeq_dyn;
        %                 A_friction;
        %                 A_clf];
        %             lbAcon = [beq_dyn;
        %                 b_lb_fric;
        %                 b_lb_clf];
        %             ubAcon = [beq_dyn;
        %                 b_ub_fric;
        %                 b_ub_clf];
        %
        %             % Construct the cost
        %             % 0.5*x'*H*x + f'*x
        %             % Assemble and solve
        %             w_u_chatter = 0.01;
        %             w_reg = [0.001 * ones(22,1); % ddq
        %                 0.1 * ones(10,1); % u
        %                 1 * ones(1,1)]; % delta
        %             w_y = 1;
        %
        %             A = [w_u_chatter .* A_u_chat;
        %                 w_reg .* A_reg;
        %                 w_y .* A_y];
        %             b = [w_u_chatter .* b_u_chat;
        %                 w_reg .* b_reg;
        %                 w_y .* b_y];
        %
        %             G = A'*A;
        %             g = -A'*b;% + 2.5 .* [(LGV*obj.outputs.Dya(:,1:22))'; zeros(11,1)];
        %
        %             % Run QP
        %             aux_data = qpOASES_auxInput('x0', obj.u_bar_last);
        %             options = qpOASES_options('maxIter', 500, 'printLevel', 1);
        %             [u_bar, fval, exitflag, numiter] = qpOASES(G, g, Acon, lb, ub, lbAcon, ubAcon, options, aux_data);
        %
        %             if exitflag ~= 0
        %                 % If we did not converge, use IO controller
        %                 u = obj.IO_control(q, dq, psi0, psi1, dyn, param);
        %                 delta = 0;
        %                 u_bar = [zeros(22,1); u; delta];
        %             end
        %
        %             % Finalize
        %             obj.u_bar_last = u_bar;
        %             ddq = u_bar(1:22);
        %             u = u_bar(23:32);     % Control
        %             %F(dyn.E,1) = dyn.R \ dyn.Sc * (QDe * ddq + QHe - QBe * u)
        %             F = pinv(dyn.JCc') * ( (eye(22) - Paghili) * dyn.De*ddq + He - dyn.Be * u );
        %
        %             Fcon = F(5:end);
        %             delta = u_bar(end);
        %         end
        
        
        function [u, F] = new_clfqp_cost(obj, q, dq, LFV, LGV, psi0, psi1, dyn, param)
            % Make sure initial guess is not empty
            if isempty(obj.u_bar_last)
                obj.u_bar_last = zeros(41,1);
            end
            
            % Dynamics Equality Constraint
            He = dyn.Cvec + dyn.Gvec;
            Aeq_dyn = [ dyn.De, -dyn.Be, -dyn.Jc'];
            %beq_dyn = - He + dyn.Fspring;
            beq_dyn = - He; % + dyn.Fspring;
            
            % Holonomic Constraints
            A_hols = [dyn.Jc, zeros(9,10), zeros(9,9)];
            b_hols = - dyn.dJc * dq;
            
            % Friction Cone Inequality Constraint
            bcontact = zeros(7,1);
            % if param.phase.tau > 0.9
            % bcontact(2:7) = inf;% bcontact * 1000000;
            % end
            P = [zeros(7,4), ... % No constraint on the first 4 holonomic wrenches
                obj.contact_pyramid]; % Constraints on GRF only
            A_friction = [ zeros(7,22), zeros(7,10), P];
            b_lb_fric = -inf * ones(7,1);
            b_ub_fric = bcontact;
            
            % Update the u bounds according to stance foot
            % and smooth with previous torque
            if strcmp(param.stance_leg,'Left')
                lb_u = obj.lbu; lb_u(5) = 0;
                ub_u = obj.ubu; ub_u(5) = 0;
            elseif strcmp(param.stance_leg,'Right')
                lb_u = obj.lbu; lb_u(10) = 0;
                ub_u = obj.ubu; ub_u(10) = 0;
            end
            u_prev = obj.u_bar_last(23:32);
            A_u_chat = [zeros(10,22), eye(10), zeros(10,9)];
            b_u_chat = u_prev;
            
            % Output PD control + virtual holonomic
            Kpy = 500;%500;
            Kdy = 50;%25;
            A_y = [obj.outputs.Dya(:,1:22), zeros(9,10), zeros(9,9)];
            b_y = obj.outputs.d2yd - obj.outputs.DLfya(:,1:22) * dq - Kpy*obj.outputs.y - Kdy*obj.outputs.dy;
            
            % Regularization
            A_reg = eye(41);
            b_reg = zeros(41,1);
            
            % Build full constraint matrices
            lb = [-inf * ones(22,1); %ddq
                lb_u;              %torque
                -inf * ones(9,1)]; %forces
            ub = [+inf * ones(22,1); %ddq
                ub_u;              %torque
                +inf * ones(9,1)]; %forces
            Acon = [Aeq_dyn;
                A_friction];
            lbAcon = [beq_dyn;
                b_lb_fric];
            ubAcon = [beq_dyn;
                b_ub_fric];
            
            % Construct the cost
            % 0.5*x'*H*x + f'*x
            % Assemble and solve
            w_ach_hol = 10;
            w_rigidsp_hol = 5;
            w_contact_hol = 10;
            w_hol = [w_ach_hol * ones(2,1);
                w_rigidsp_hol * ones(2,1);
                w_contact_hol * ones(5,1)];
            w_u_chatter = 1e-1;
            w_reg = [1e-5 * ones(22,1); % ddq
                1e-1 * ones(10,1); % u
                1e-5 * ones(2,1); % lambda_ach
                1e-5 * ones(2,1); % lambda_rigid
                1e-4 * ones(2,1); % fx fy
                1e-5 * ones(1,1); % fz
                1e-4 * ones(2,1)]; % muy, muz
            w_y = 1e-1;
            
            A = [...
                w_hol .* A_hols;
                w_u_chatter .* A_u_chat;
                w_y .* A_y;
                w_reg .* A_reg];
            b = [...
                w_hol .* b_hols;
                w_u_chatter .* b_u_chat;
                w_y .* b_y;
                w_reg .* b_reg];
            
            G = A'*A;
            g = -A'*b;% +  0.1 .* [(LGV*obj.outputs.Dya(:,1:22))'; zeros(19,1)];
            
            % Run QP
            aux_data = qpOASES_auxInput('x0', obj.u_bar_last);
            options = qpOASES_options('maxIter', 50, 'printLevel', 1);
            [u_bar, fval, exitflag, numiter] = qpOASES(G, g, Acon, lb, ub, lbAcon, ubAcon, options, aux_data);
            
            if exitflag ~= 0
                % If we did not converge, use IO controller
                u = obj.IO_control(q, dq, psi0, psi1, dyn, param);
                delta = 0;
                u_bar = [zeros(22,1); u; zeros(9,1)];
            end
            
            % Finalize
            obj.u_bar_last = u_bar;
            ddq = u_bar(1:22);
            u = u_bar(23:32);     % Control
            F = u_bar(33:41);
            Fcon = F(5:end);
        end
        
        
    end
end

