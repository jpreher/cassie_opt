function right_lift_constraints(nlp, src, tar, bounds, varargin)

    plant = nlp.Plant;
    
    % first call the class method (calling impact model since it no longer
    % applies if we have a custom function)
    plant.rigidImpactConstraint(nlp, src, tar, bounds, varargin{:});
    
    % Add average velocity constraint over [DS, SS]
    Constraints.walkingSpeedMultipleDomain(bounds, nlp, src, tar);
    
end