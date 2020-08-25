function constraintsSS(nlp, bounds, varargin)

% Virtual constraints
domain = nlp.Plant;
%domain.VirtualConstraints.velocity.imposeNLPConstraint(nlp, bounds.velocity.ep, 0);
domain.VirtualConstraints.position.imposeNLPConstraint(nlp, [bounds.position.kp, bounds.position.kd], [1, 1]);

% All other constraints
if isempty(strfind(domain.Name, 'Right'))
    stanceFoot = 'Left';
else
    stanceFoot = 'Right';
end
Constraints.tauSS(nlp, true, false);
Constraints.nsfYaw(bounds,nlp, stanceFoot);
Constraints.nsfPitch(bounds,nlp, stanceFoot);
Constraints.impactVelocity(bounds, nlp, stanceFoot);
Constraints.stepClearance(bounds,nlp, stanceFoot);
Constraints.stepWidth(bounds,nlp, stanceFoot);
Constraints.swingFootSpeed(bounds, nlp, stanceFoot);
Constraints.walkingSpeed(bounds, nlp);




end