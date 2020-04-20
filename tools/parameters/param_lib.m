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
params.pposition = reshape(params.pposition, 9,8);
if strcmp(leg, 'Left')
    params.pposition(1,:) = -params.pposition(1,:);
    params.pposition(1,:) = -params.pposition(1,:);
    
    params.x0 = []; % invalid
end % Otherwise we are fine
    
end