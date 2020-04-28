function nsfPitch(bounds, nlp, stanceFoot)

domain = nlp.Plant;
x = domain.States.x;

if strcmp(stanceFoot, 'Left')
    nsf_orient = domain.getCartesianPosition(domain.ContactPoints.RightToe) - domain.getCartesianPosition(domain.ContactPoints.RightHeel);
else
    nsf_orient = domain.getCartesianPosition(domain.ContactPoints.LeftToe) - domain.getCartesianPosition(domain.ContactPoints.LeftHeel);
end

pitch_constr_fun = SymFunction(['nsf_pitch_',domain.Name], nsf_orient(3), x);
addNodeConstraint(nlp, pitch_constr_fun, {'x'}, 'all', 0, 0, 'Nonlinear');

end

