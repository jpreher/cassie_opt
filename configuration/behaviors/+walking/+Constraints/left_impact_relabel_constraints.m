function left_impact_relabel_constraints(nlp, src, tar, bounds, varargin)
    % NOT time-continuous
    removeConstraint(nlp,'tContDomain');
    
    %% Replace the joint position stitching for HZD condition
    plant = nlp.Plant;
    
    % First call the class method
    plant.rigidImpactConstraint(nlp, src, tar, bounds, varargin{:});
    
    % The relabeling of joint coordiante is no longer valid
    removeConstraint(nlp,'xDiscreteMapLeftImpactRelabel');
    
    R = plant.R;
    
    % The configuration only depends on the relabeling matrix
    x = plant.States.x;
    xn = plant.States.xn;
    x_diff = R*x-xn;
    
    x_map = SymFunction(['xDiscreteMap' plant.Name], x_diff(3:end), {x,xn});
    addNodeConstraint(nlp, x_map, {'x','xn'}, 'first', 0, 0, 'Linear');
        
end