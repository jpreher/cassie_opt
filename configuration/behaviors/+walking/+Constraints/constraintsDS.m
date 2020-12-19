function constraintsDS(nlp, bounds, varargin)

% Virtual constraints
domain = nlp.Plant;
% domain.VirtualConstraints.velocity.imposeNLPConstraint(nlp, bounds.velocity.ep, 0);
domain.VirtualConstraints.position.imposeNLPConstraint(nlp, [bounds.position.kp, bounds.position.kd], [1, 1]);

% All other constraints
Constraints.tauDS(nlp, true, false);
% Constraints.walkingSpeed(bounds, nlp);

end