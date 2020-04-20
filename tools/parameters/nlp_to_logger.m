function [ logger, edges ] = nlp_to_logger( nlp, sol )
[tspan, states, inputs, params] = exportSolution(nlp, sol);

ind = 1;
edg = 1;

for i = 1:length(nlp.Phase)
    if mod(i,2) == 1
        plant = nlp.Phase(i).Plant;
        
        logger(ind) = SimLogger(nlp.Phase(i).Plant);
        logger(ind).static.params = params{i};

        for k = 1:length(tspan{i})
            calc = struct();
            calc.t = tspan{i}(k);
            calc.states.x = states{i}.x(:,k);
            calc.states.dx = states{i}.dx(:,k);
            calc.states.ddx = states{i}.ddx(:,k);
            
            calc.inputs.Control.u = inputs{i}.u(:,k);
            
            calc.inputs = struct();
            
            input_names = fieldnames(plant.Inputs.Control);
            if ~isempty(input_names)        
                for j=1:length(input_names)
                    name = input_names{j};            
                    input = inputs{i}.(name);
                    calc.inputs.Control.(name) = input(:,k);
                end
            end

            input_names = fieldnames(plant.Inputs.ConstraintWrench);
            if ~isempty(input_names)        
                for j=1:length(input_names)
                    name = input_names{j};            
                    input = inputs{i}.(name);
                    if size(input,2) == 1
                        calc.inputs.ConstraintWrench.(name) = input(:);
                    else
                        calc.inputs.ConstraintWrench.(name) = input(:,k);
                    end
                end
            end


            input_names = fieldnames(plant.Inputs.External);

            if ~isempty(input_names)        
                for j=1:length(input_names)
                    name = input_names{j};      
                    input = inputs{i}.(name);
                    calc.inputs.External.(name) = input(:,k);
                end
            end

            
            logger(ind).calc = calc;

            logger(ind).updateLog();
        end
        ind = ind+1;
    else
        plant = nlp.Phase(i).Plant;
        
        % We are on an edge
        edges(edg) = SimLogger(nlp.Phase(i).Plant);
        
        calc = struct();
        calc.t = tspan{i};
        calc.states.x = states{i}.x;
        calc.states.xn = states{i}.xn;
        calc.states.dx = states{i}.dx;
        calc.states.dxn = states{i}.dxn;

        calc.inputs = struct();


        input_names = fieldnames(plant.Inputs.ConstraintWrench);
        if ~isempty(input_names)        
            for j=1:length(input_names)
                name = input_names{j};            
                input = inputs{i}.(name);
                if size(input,2) == 1
                        calc.inputs.ConstraintWrench.(name) = input(:);
                else
                    calc.inputs.ConstraintWrench.(name) = input(:,k);
                end
            end
        end


        input_names = fieldnames(plant.Inputs.External);

        if ~isempty(input_names)        
            for j=1:length(input_names)
                name = input_names{j};      
                input = inputs{i}.(name);
                calc.inputs.External.(name) = input;
            end
        end
        edges(edg).calc = calc;
        edges(edg).updateLog();
        
        edg = edg + 1;
    end
    
end

    % When there is no edges assigned
    if ~exist('edges','var') 
        edges = [];
    end

end

