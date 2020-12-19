function stepClearance(bounds, nlp, stanceFoot)

domain = nlp.Plant;
x = domain.States.x;
dx = domain.States.dx;

nsf_height = SymFunction(['nsf_height_', domain.Name], nlp.Plant.EventFuncs.nsf.ConstrExpr, {x});
addNodeConstraint(nlp, nsf_height, {'x'}, 'all',                  0,    0.18, 'Nonlinear');
addNodeConstraint(nlp, nsf_height, {'x'}, floor(nlp.NumNode/4),   0.04, 0.18, 'Nonlinear');
addNodeConstraint(nlp, nsf_height, {'x'}, ceil(nlp.NumNode/2),    0.12, 0.18,'Nonlinear');
addNodeConstraint(nlp, nsf_height, {'x'}, floor(3*nlp.NumNode/4), 0.04, 0.18, 'Nonlinear');

end

