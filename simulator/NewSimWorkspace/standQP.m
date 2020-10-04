function u = standQP(q, dq, dyn, param)

gam = 0.1;
ep = 1e-3;
ny = 6;

% Compute the outputs
if coder.target('MATLAB')
    ya = frost_expr.outputs.ya_stand(q);
    dya = frost_expr.outputs.dya_stand(q,dq);
    Dya = frost_expr.outputs.Dya_stand(q);
    DLfya = frost_expr.outputs.DLfya_stand(q,dq);
else
    % Not implemented
    error('Codegen not supported!');
end


% Compute eta and the Lyapunov function
eta = [ya - param.yd_qp;
       dya - param.dyd_qp];
F = [zeros(ny),eye(ny);
     zeros(ny),zeros(ny)];
G = [zeros(ny);eye(ny)];
Q = eye(2*ny);
[P,~,~] = care(F,G,Q);

Iep = blkdiag(ep * eye(ny), eye(ny));
Pep = Iep' * P * Iep;
Veta = eta' * Pep * eta; 


% Generate QP constraints
traditional_qp = true;
if traditional_qp
    %%% Run traditional clf-qp
    Bbar = [dyn.Be, dyn.Jc'];
    
    % Holonomic constraints must be satisfied
    % Jcdot*dq + Jc * ( De \ (Bbar*u_bar - Ce*dq - Ge + Fsprings) ) = 0;
    Aeq_holonomic = dyn.Jc * ( dyn.De \ Bbar );
    beq_holonomic = dyn.Jc * ( dyn.De \ (dyn.Ce * dq + dyn.Ge - dyn.Fspring) ) - dyn.dJc * dq;
    
    % Torque constraints must be satisfied
    A_torque    = [eye(10), zeros(10, size(dyn.Jc,1));
                   eye(10), zeros(10, size(dyn.Jc,1))];
    b_torque = [repmat([4.5, 4.5, 12.2, 12.2, 0.9], 1, 2)';
               -repmat([4.5, 4.5, 12.2, 12.2, 0.9], 1, 2)'];
   
    % Create friction cone constraints
           
           
    % If time based
    DLfy = DLfya;
    ddy = zeros(size(DLfy,1),1); % ddyd
    % Otherwise...
    % DLfy(y_indices,:) = y_a{i}{end} - y_d{i}{end};
        
    % Get dynamics
    gfc = [zeros(size(Bbar));
           dyn.De \ Bbar];
    vfc = [dq; 
           dyn.De \ (dyn.Fspring - dyn.Ce*dq - dyn.Ge)];
    
    Lf_mat = DLfya*vfc;
    A_mat  = DLfya*gfc; % if doing rd1 need more...
    
    % Construct the control constraint
    LfV = eta'*(F'*Pep + Pep*F)*eta;
    LgV = 2*eta'*Pep*G;
    
    psi0 = LfV + gam*Veta;
    psi1 = LgV';
    
    % psi0 + psi1'*(A_hat*u_hat + Lf) <= delta
    A_clf = [psi1'*A_mat, -1];
    b_clf = -psi1'*Lf_mat - psi0;
    
    % Construct the cost
    % 0.5*x'*H*x + f'*x
    H_clf = 2 * (A_mat' * A_mat); % 2* cancels with quadprog 1/2
    f_clf = 2 * Lf_mat' * A_mat - ddy'*A_mat;
    
    H_slack = 1;
    f_slack = param.w_slack;
    
    % Assemble and solve
    Hmat = blkdiag(H_clf, H_slack);
    fmat = [f_clf, f_slack];
    
    A = [A_clf];
    b = [b_clf];
    
    Aeq = [Aeq_holonomic, zeros(size(Aeq_holonomic,1),1)];
    beq = [beq_holonomic];
        
    options = optimoptions('quadprog','Algorithm','interior-point-convex','display','iter');
    u_bar   = quadprog(Hmat,fmat,A,b,Aeq,beq,[],[],[],options);
    
    u = u_bar(1:10);  % Control
    F = u_bar(11:end); % Constraint forces
    
    u'
else
    %%% Do new cost method
    
    u = zeros(10,1);
    
end



end