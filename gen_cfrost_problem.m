%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Setting up paths
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
cur = fileparts(mfilename('fullpath'));
addpath(genpath('tools'));
addpath(genpath('configuration'));
addpath(genpath('modules/C-Frost'));
addpath('modules/cassie_description/MATLAB');
warning('off', 'MATLAB:MKDIR:DirectoryExists');
frost_addpath;
export_path = fullfile(cur, 'export/');
mkdir(export_path);
addpath(export_path);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Load the behavior specific NLP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
omit_coriolis   = false;
is_symmetric    = false;

% behaviorName = 'stepping_planar';
behaviorName = 'stepping';
% behaviorName = 'walking';

behavior = loadBehavior(behaviorName, is_symmetric, ...
    omit_coriolis, omit_coriolis, false, false);
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
%%%% Creating the solver problem
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
solver = IpoptApplication(nlp);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Create c ipopt problem
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
COMPILE = true;

c_code_path   = strcat('c_code/', behavior.name);
src_path      = strcat(c_code_path, '/src');
src_gen_path  = strcat(c_code_path, '/src/gen');
include_dir   = strcat(c_code_path, '/include');
res_path      = strcat(c_code_path, '/res');
root_path_str = pwd;

mkdir(c_code_path);
mkdir(src_path);
mkdir(src_gen_path);
mkdir(include_dir);
mkdir(res_path);

[funcs] = frost_c.getAllFuncs(solver);
if COMPILE
    frost_c.createFunctionListHeader(funcs, src_path, include_dir);
    frost_c.createIncludeHeader(funcs, include_dir);
    save('function_list.mat', 'funcs');
    
    % frost_c.createConstraints(nlp,[],[],src_gen_path, include_dir, {'dynamics_equation'}); % Constraints WITHOUT dynamics
    frost_c.createConstraints(nlp,[],[],src_gen_path, include_dir); % All constraints INCLUDING dynamics
    frost_c.createObjectives(nlp,[],[],src_gen_path, include_dir);
end
frost_c.createDataFile(solver, funcs, res_path, 'data');

% Copy over a template CMakeLists
fid = fopen('tools/custom_compiling/CMakeLists_cfrost.txt');
F = fread(fid, '*char')';
fclose(fid);
F=strrep(F, 'ROOT_PATH', root_path_str);
fid = fopen(fullfile(c_code_path, 'CMakeLists.txt'), 'w');
fwrite(fid, F);
fclose(fid);
