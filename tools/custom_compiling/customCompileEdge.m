%% Function: customCompileEdge
%
% Description:
%   Compile expressions with customizible options
%
% Author: Jenna Reher, jreher@caltech.edu
% ______________________________________
function [ ] = customCompileEdge( edge, export_path )

varargin = {};

% export the impact constraints    
i_constrs = fieldnames(edge.ImpactConstraints);
if ~isempty(i_constrs)
    for i=1:length(i_constrs)
        constr = i_constrs{i};
        export(edge.ImpactConstraints.(constr),export_path,varargin{:});

    end

end

% call superclass method
input_categories = {'Control','ConstraintWrench','External'};
for k=1:3
    category = input_categories{k};
    % export the input map
    gmap_funcs = fieldnames(edge.Gmap.(category));
    if ~isempty(gmap_funcs)
        for i=1:length(gmap_funcs)
            fun = gmap_funcs{i};
            if ~isempty(edge.Gmap.(category).(fun))
                export(edge.Gmap.(category).(fun),export_path,varargin{:});
            end
        end
    end


    % export the input vector fields
    gvec_funcs = fieldnames(edge.Gvec.(category));
    if ~isempty(gvec_funcs)
        for i=1:length(gvec_funcs)
            fun = gvec_funcs{i};
            if ~isempty(edge.Gvec.(category).(fun))
                export(edge.Gvec.(category).(fun),export_path,varargin{:});
            end
        end
    end
end

end

