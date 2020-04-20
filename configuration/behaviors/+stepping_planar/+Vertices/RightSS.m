function domain = RightSS(model)
% Flat-footed walking, Right Stance Domain 
% Contact: Right Foot (line with friction)
% Construct the right stance domain of the Cassie
%
% model: the right body model of Cassie robot
    
    %% first make a copy of the robot model
    %| @note Do not directly assign with the model variable, since it is a
    %handle object.
    domain = copy(model);
    domain.setName('RightSS');
    
    %% Set the domain
    phaseType = 'StateBased';
    domain = Domain.SingleSupportPlanar(domain, 'Right', phaseType);
    
    %% Fix time based params
    domain.PreProcess = @Edges.NewStepPreProcess;

end