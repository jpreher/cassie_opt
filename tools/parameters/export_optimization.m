function [logger, edges, params, export_path] = export_optimization( nlp, sol, behavior, do_save, label_append )
if nargin < 5
    label_append = '';
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Export the optimization result
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[ logger, edges ] = nlp_to_logger( nlp, sol );

for i = 1:numel(logger)
    params{i,1} = logger(i).static.params;
    
    if ~isempty(params{i})    
        params{i,1}.name = logger(i).plant.Name;
        params{i,1}.x0 = [logger(i).flow.states.x(:,1); logger(i).flow.states.dx(:,1)];

        % Force zero to be true zero on polynomials
        if isfield(params{i}, 'aposition')
            params{i,1}.pvelocity = params{i,1}.pposition;
        end
    end
end

if behavior.isSymmetric
        params = remap_symmetric_param(params, behavior, logger);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Save info and parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
export_path = [];
if do_save
    %if ispc
        baseDir = './';
    %else
    %    baseDir = ros_resolve_local_url('package://cassie_opt/matlab/');
    %end
    %
    gaitName = string(datetime('now', 'Format', 'yyyy-MM-dd''T''HH-mm'));
    export_path = char(strcat(baseDir, 'params/', nlp.Name, '/', gaitName, '_', label_append, '/'));
    if ~exist(export_path,'dir')
        mkdir(char(export_path));
    end
    
    k=1;
    for i = 1:numel(params)
        if ~isempty(fields(params{i}))
            export_params{k} = params{i};
            k = k+1;
        end
    end
    
    param_name = strcat('params_',gaitName);
    export_name = strcat(export_path, param_name, '_', label_append, '.yaml');
    yaml_write_file(export_name, export_params);
    
    % Export peripheral data
    if ~exist(strcat(export_path, 'info'), 'dir')
        mkdir(char(strcat(export_path, 'info')));
    end
    
    constr_out = strcat(export_path, 'info/constr.txt');
    varibs_out = strcat(export_path, 'info/vars.txt');
    costs_out  = strcat(export_path, 'info/cost.txt');

    %%% Output the folder 'info'
    nlp.checkConstraints(sol, 1e-3, constr_out);
    nlp.checkVariables(sol, 1e-3, varibs_out);
    nlp.checkCosts (sol, costs_out);
end

end

