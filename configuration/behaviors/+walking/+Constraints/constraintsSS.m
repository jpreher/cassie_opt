function constraintsSS(nlp, bounds, varargin)

% Virtual constraints
domain = nlp.Plant;
%domain.VirtualConstraints.velocity.imposeNLPConstraint(nlp, bounds.velocity.ep, 0);
domain.VirtualConstraints.position.imposeNLPConstraint(nlp, [bounds.position.kp, bounds.position.kd], [1, 1]);

% All other constraints
Constraints.tauSS(nlp, true, false);
Constraints.nsfYaw(bounds,nlp);
Constraints.nsfPitch(bounds,nlp);
Constraints.impactVelocity(bounds, nlp);
Constraints.stepClearance(bounds,nlp);
Constraints.stepWidth(bounds,nlp);
Constraints.swingFootSpeed(bounds, nlp, 'Right');

end