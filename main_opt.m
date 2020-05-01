%% Script: main_opt
%
% Description: This script runs the main optimization routine for a loaded
%   behavior. The "startup_cassie.m" script must be run first. You can
%   specify whether to recompile the behavior expressions, model, and cost
%   functions separately through a true/false flag. You can also load
%   initial guesses from simulation through commenting out the
%   corresponding lines related to the simulation "logger". Users
%   can specify whether to use IPOPT or SNOPT by simply commenting out the
%   corresponding lines. SNOPT requires an additional license. Finally, the
%   "do_save" option at the bottom of the script can be set to "true" if
%   you would like to save the walking gait parameters as a YAML file.
%
% Author: Jenna Reher (v1, jreher@caltech)
% _________________________________________________________________________

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Load the behavior specific NLP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
vd = [0; 0];
nlp = feval(strcat(behavior.name, '.Constraints.setupOpt'), behavior, vd);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Compile and export optimization functions
%%%% (uncomment the following lines when run it for the first time.)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
do_export_optModel    = 0;
do_export_optBehavior = 0;
do_export_optCost     = 0;

t1 = tic;
customExportOptimization(behavior, nlp, do_export_optModel, do_export_optBehavior, do_export_optCost);
fprintf('Compilation took %f minutes.\n', toc(t1)/60);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Create a library of walking gaits
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gait Library
vd_x = -0.8 : 0.1 : 1.0;
vd_y = -0.5 : 0.1 : 0.5;

% Iterate over list
for i = 1:length(vd_x)
    % Make sure that the optimization always uses the closest y speed in
    % the initial guess 
    % (i.e. (-0.5,0.4)->(-0.4,0.4) rather than (-0.5,0.4)->(-0.4,-0.4)
    flipy = false;
    if mod(i,2) == 0
        flipy = true;
    end
    for j = 1:length(vd_y)
        % Get the new desired velocity
        if flipy
            vd = [vd_x(i); vd_y(length(vd_y)-j+1)];
        else
            vd = [vd_x(i); vd_y(j)];
        end
        show_this = strcat("vd: ", num2str(vd(1)), ", ", num2str(vd(2)));
        display(show_this)
        nlp = feval(strcat(behavior.name, '.Constraints.setupOpt'), behavior, vd);
        
        if exist('logger','var') == 1
            loadNodeInitialGuess(nlp.Phase(1), logger(1));
            if length(logger) > 1
                loadNodeInitialGuess(nlp.Phase(3), logger(2));
                loadEdgeInitialGuess(nlp.Phase(2), logger(1), logger(2));
                loadEdgeInitialGuess(nlp.Phase(4), logger(2), logger(1));
            else
                loadEdgeInitialGuess(nlp.Phase(2), logger(1), logger(1));
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%% Link the NLP problem to a NLP solver
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Log all activities in cmd
        if exist('log.yaml','file')
            delete('log.yaml');
        end
        diary log.txt;
        
        % Run IPOPT
        options = struct();
        options.max_iter = 1000;
        solver = IpoptApplication(nlp, options);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%% Run the optimization
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        t1 = tic;
        [sol, info] = optimize(solver);
        fprintf('Elapsed time is %f minutes.\n', toc(t1)/60);
        
        if info.status < 0
            while info.status < 0
                t1 = tic;
                [sol, info] = optimize(solver, sol);
                fprintf('Elapsed time is %f minutes.\n', toc(t1)/60);
            end
        end
        diary off;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%% Check and export the optimization result
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % save the velocity name
        label_append = strcat("v_", num2str(vd(1)), "_", num2str(vd(2)));
        
        do_save = true;
        [logger, edges, params, save_path] = export_optimization( nlp, sol, behavior, do_save, label_append );
        
        % Save the result to a file
        if do_save
            data = struct();
            data.logger = logger;
            data.edges = edges;
            data.params = params;
            data.sol = sol;
            data.nlp = nlp;
            
            %%% log optimization results
            save(strcat(save_path, 'optData.mat'), 'data');
            movefile('log.txt', save_path);
        end
        
        %%% Load previous result for next loop of gait library
        %loadNodeInitialGuess(nlp.Phase(1), logger(1));
        %loadEdgeInitialGuess(nlp.Phase(2), logger(1), logger(1));
    end
end