function nsfYaw(bounds, nlp, stanceFoot)

domain = nlp.Plant;
x = domain.States.x;

if strcmp(stanceFoot, 'Left')
    nsf_orient = domain.getCartesianPosition(domain.ContactPoints.RightToe) - domain.getCartesianPosition(domain.ContactPoints.RightHeel);
else
    nsf_orient = domain.getCartesianPosition(domain.ContactPoints.LeftToe) - domain.getCartesianPosition(domain.ContactPoints.LeftHeel);
end

yaw_constr_fun = SymFunction(['nsf_yaw_',domain.Name], nsf_orient(2), x);
addNodeConstraint(nlp, yaw_constr_fun, {'x'}, 'all', 0, 0, 'Nonlinear');


end

