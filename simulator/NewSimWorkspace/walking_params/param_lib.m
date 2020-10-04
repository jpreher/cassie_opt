function params = param_lib(vd, leg, interp_lib)

for i = 1:length(interp_lib.aposition)
    params.aposition(i,1) = feval(interp_lib.aposition{i},vd);
end

for i = 1:length(interp_lib.pposition)
    params.pposition(i,1) = feval(interp_lib.pposition{i},vd);
end

for i = 1:length(interp_lib.x0)
    params.x0(i,1) = feval(interp_lib.x0{i},vd);
end

% Export in correct leg
params.aposition = reshape(params.aposition, 9,7);
if strcmp(leg, 'Left')
    params.aposition(1,:) = -params.aposition(1,:);
    params.aposition(2,:) = -params.aposition(2,:);
    params.aposition(3,:) = -params.aposition(3,:);
    params.aposition(4,:) = -params.aposition(4,:);
    
    params.x0 = []; % invalid
end % Otherwise we are fine
    
end