function domain = RightDS(model)
% Flat-footed walking, Right Stance Domain 
% Contact: Right Foot (line with friction)
% Construct the right stance domain of the Cassie
%
% model: the right body model of Cassie robot
    
    %% first make a copy of the robot model
    %| @note Do not directly assign with the model variable, since it is a
    %handle object.
    domain = copy(model);
    domain.setName('RightDS');
    
    %% Set the domain
    phaseType = 'TimeBased';
    domain = Domain.DoubleSupport(domain, 'Right', phaseType);
    
    %% Fix time based params
    domain.PreProcess = @Edges.NewStepPreProcess;

end