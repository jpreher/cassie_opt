%% Function: customCompileVertex
%
% Description:
%   Compile expressions with customizible options
%
% Author: Jenna Reher, jreher@caltech.edu
% ______________________________________
function [ ] = customCompileVertex( vertex, export_path )

varargin = {};

% Create export directory if it does not exst
if ~exist(export_path,'dir')
    mkdir(export_path);
    addpath(export_path);
end

% export the holonomic constraints    
h_constrs = fieldnames(vertex.HolonomicConstraints);
if ~isempty(h_constrs)
    for i=1:length(h_constrs)
        constr = h_constrs{i};
        export(vertex.HolonomicConstraints.(constr),export_path,varargin{:});
    end
end

% export the unilateral constraints       
u_constrs = fieldnames(vertex.UnilateralConstraints);
if ~isempty(u_constrs)
    for i=1:length(u_constrs)
        constr = u_constrs{i};
        export(vertex.UnilateralConstraints.(constr),export_path,varargin{:});
    end

end

% export the virtual constraints       
v_constrs = fieldnames(vertex.VirtualConstraints);
if ~isempty(v_constrs)
    for i=1:length(v_constrs)
        constr = v_constrs{i};
        export(vertex.VirtualConstraints.(constr),export_path,varargin{:});
    end

end

% export the event functions
funcs = fieldnames(vertex.EventFuncs);
if ~isempty(funcs)
    for i=1:length(funcs)
        fun = funcs{i};
        export(vertex.EventFuncs.(fun),export_path,varargin{:});
    end        
end

% call superclass method
input_categories = {'Control','ConstraintWrench','External'};
for k=1:3
    category = input_categories{k};
    % export the input map
    gmap_funcs = fieldnames(vertex.Gmap.(category));
    if ~isempty(gmap_funcs)
        for i=1:length(gmap_funcs)
            fun = gmap_funcs{i};
            if ~isempty(vertex.Gmap.(category).(fun))
                export(vertex.Gmap.(category).(fun),export_path,varargin{:});
            end
        end
    end


    % export the input vector fields
    gvec_funcs = fieldnames(vertex.Gvec.(category));
    if ~isempty(gvec_funcs)
        for i=1:length(gvec_funcs)
            fun = gvec_funcs{i};
            if ~isempty(vertex.Gvec.(category).(fun))
                export(vertex.Gvec.(category).(fun),export_path,varargin{:});
            end
        end
    end
end

end