%% Function: compileDynamicsWithoutCoriolis
% 
% Description:
%   To compile dynamical functions partially, 
%   Not compiling M, C & G vecotrs
%   Only, springs, holonomic constraints, ...
%
% Author: Wenlong Ma, wma@caltech.edu
%         Jenna Reher, jreher@caltech.edu
% ______________________________________

function [] = compileDynamicsWithoutCoriolis(nlp, export_path)

nDomain = floor(numel(nlp.Phase) / 2);

for ph_index = 1:2:nDomain

    ph_Constr = nlp.Phase(ph_index).ConstrTable;
    
    for i = 1 : length(ph_Constr.dynamics_equation(1).SummandFunctions)

        w = ph_Constr.dynamics_equation(1).SummandFunctions(i).Name;
        if ~startsWith(w, 'Mmat') && ~startsWith(w, 'Ce') && ~startsWith(w, 'Ge')
                export(ph_Constr.dynamics_equation(1).SummandFunctions(i).SymFun, export_path);
                
                % If derivative level above 0 do jacobian
                if nlp.Options.DerivativeLevel > 0
                    exportJacobian(ph_Constr.dynamics_equation(1).SummandFunctions(i).SymFun, export_path);
                end
                % If higher (max 2) then do hessian
                if nlp.Options.DerivativeLevel > 1
                    exportHessian(ph_Constr.dynamics_equation(1).SummandFunctions(i).SymFun, export_path);
                end            
        end
        
    end

end

end