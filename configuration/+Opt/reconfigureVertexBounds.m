function [  ] = reconfigureVertexBounds( nlp )

domain = nlp.Plant;
x = domain.States.x;
dx = domain.States.dx;

% Update dynamics equation
bound = struct();

% Get all domain constraints
nlpConstr = fields(nlp.ConstrTable);

for n = 1:nlp.NumNode
    % Update integration
    bound.lb = -1e-6;
    bound.ub =  1e-6;
    int = nlp.ConstrTable.intX(n);
    if ~isempty(int.Name)
        int.updateProp(bound);
    end
    
    dint = nlp.ConstrTable.intXdot(n);
    if ~isempty(dint.Name)
        dint.updateProp(bound);
    end    
    
    % Update Dynamics
    bound.lb = -5e-4;
    bound.ub =  5e-4;
    nlp.ConstrTable.dynamics_equation(n).updateProp(bound);

    % Update holonomic constraints
    iHol = startsWith(nlpConstr, 'h_');
    hols = nlpConstr(iHol);
    for i = 1:sum(iHol)
        hName = hols{i};
        dhName = ['d', hName];
        ddhName = ['d', dhName];

        % Position
        bound.lb = -1e-5;
        bound.ub =  1e-5;
        h = nlp.ConstrTable.(hName);
        if ~isempty(h(n).Name)
            h(n).updateProp(bound);
        end

        % Velocity
        bound.lb = -1e-4;
        bound.ub =  1e-4;
        dh = nlp.ConstrTable.(dhName);
        if ~isempty(dh(n).Name)
            dh(n).updateProp(bound);
        end

        % Acceleration
        bound.lb = -1e-4;
        bound.ub =  1e-4;
        ddh = nlp.ConstrTable.(ddhName);
        if ~isempty(ddh(n).Name)
            ddh(n).updateProp(bound);
        end
    end

    % Update virtual constraints
    if isfield(domain.VirtualConstraints, 'velocity')          
        hName = 'velocity_output_dynamics';

        % Output Dynamics
        bound.lb = -1e-3;
        bound.ub =  1e-3;
        h = nlp.ConstrTable.(hName);
        if ~isempty(h(n).Name)
            h(n).updateProp(bound);
        end
    end

    if isfield(domain.VirtualConstraints, 'position')
        hName = ['y_position_', domain.Name];
        dhName = ['d1', hName];
        ddhName = 'position_output_dynamics';

        % Position
        bound.lb = -1e-3;
        bound.ub =  1e-3;
        h = nlp.ConstrTable.(hName);
        if ~isempty(h(n).Name)
            h(n).updateProp(bound);
        end

        % Velocity
        dh = nlp.ConstrTable.(dhName);
        if ~isempty(dh(n).Name)
            dh(n).updateProp(bound);
        end

        % Acceleration
        bound.lb = -1e-3;
        bound.ub =  1e-3;
        ddh = nlp.ConstrTable.(ddhName);
        if ~isempty(ddh(n).Name)
            ddh(n).updateProp(bound);
        end
    end
       
end

end

