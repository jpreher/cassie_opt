function left_impact_constraints(nlp, src, tar, bounds, varargin)

    plant = nlp.Plant;
        
    % first call the class method (calling impact model since it no longer
    % applies if we have a custom function)
    plant.rigidImpactConstraint(nlp, src, tar, bounds, varargin{:});
    
    % Make parameters symmetric
    Constraints.edgeOutputSymmetry(bounds, nlp, src, tar)
    
end