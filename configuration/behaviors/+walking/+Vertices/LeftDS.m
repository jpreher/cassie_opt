function domain = LeftDS(model)
 % Left Stance Domain 
 %
 % Contact: Left Foot (line with friction)
 %
 % model: the Left body model of Cassie robot
    
    %% first make a copy of the robot model
    %| @note Do not directly assign with the model variable, since it is a
    %handle object.
    domain = copy(model);
    domain.setName('LeftDS');
    
    %% Set the domain
    phaseType = 'TimeBased';
    domain = Domain.DoubleSupport(domain, 'Left', phaseType);

    %% Fix time based params
    domain.PreProcess = @Edges.NewStepPreProcess;
    
end
    