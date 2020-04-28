function edgeOutputSymmetry(bounds, nlp, src, tar)
assert(src.Plant.VirtualConstraints.position.PolyDegree == ...
    tar.Plant.VirtualConstraints.position.PolyDegree, ...
    'The polynomial degree for target and source domains MUST be equal!');

plant = nlp.Plant;
domain = src.Plant;
x = domain.States.x;
dx = domain.States.dx;
v_target = bounds.params.vd;

if abs(v_target(2)) < 1e-3
    lb = 0;
    ub = 0;
else
    lb = -inf;
    ub = +inf;
end

degree = domain.VirtualConstraints.position.PolyDegree;
labels = domain.VirtualConstraints.position.OutputLabel;
ny = src.Plant.VirtualConstraints.position.Dimension;

yawOutput = strfind(labels, 'Yaw');
rollOutput = strfind(labels, 'Roll');
posYOutput = strfind(labels, 'PosY');

yawIndices  = find(~cellfun(@isempty,yawOutput));
rollIndices = find(~cellfun(@isempty,rollOutput));
YposIndices = find(~cellfun(@isempty,posYOutput));

ap_s = SymVariable('aps', [ny, degree+1]);
ap_t = SymVariable('apt', [ny, degree+1]);
ap_t_fix = SymVariable('apt', [ny, degree+1]);

ap_t_fix(yawIndices,:) = -ap_t_fix(yawIndices,:);
ap_t_fix(rollIndices,:) = -ap_t_fix(rollIndices,:);

aDiff = SymExpression(zeros(ny, degree+1));
for j = 1:ny
    aDiff(j,:) = ap_s(j,:) - ap_t_fix(j,:);
end
aDiff = flatten(aDiff(:))';
na = length(aDiff);
aDiff_eq = SymFunction(['apositionEquality_', plant.Name], aDiff, {SymVariable(flatten(ap_s(:))'), ...
    SymVariable(flatten(ap_t(:))')});

ap_cstr = NlpFunction('Name',['apositionEquality_', plant.Name],...
    'Dimension', na,...
    'lb', lb,...
    'ub', ub,...
    'Type', 'Linear',...
    'SymFun', aDiff_eq,...
    'DepVariables',[src.OptVarTable.aposition(end); tar.OptVarTable.aposition(1)]);
nlp.addConstraint(['apositionEquality_', plant.Name], 'first', ap_cstr); 


end

