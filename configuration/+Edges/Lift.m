%% Function: LeftLift
%
% Description: Apply a trivival (identity mapping) domain transition when:
%      nonstanceFoot leaves ground, while stanceFoot remains on ground.
%      NO stance leg changed
%
% Author: Jenna Reher, jreher@caltech.edu
%         Wenlong Ma, wma@caltech.edu
% ________________________________________

function guard = Lift(tar)

    if startsWith(tar.Name,'Left')
        guardName = 'RightLift';
    elseif startsWith(tar.Name,'Right')
        guardName = 'LeftLift';
    else
        error('Something is wrong, foo');
    end
    guard = RigidImpact(guardName, tar, 'groundForce');
    
    %% Set the impact constraint
%     guard.addImpactConstraint(struct2array(tar.HolonomicConstraints));
    
    %% The guard should not be a rigid impact...
    %     constr = fields(guard.ImpactConstraints);r
    %     guard.removeImpactConstraint(constr);

    %R = guard.R;
    %js = {'LeftShinPitch','LeftAchillesSpring','RightShinPitch','RightAchillesSpring'};
    %for j = 1:length(js)
    %    ind = tar.getJointIndices(js{j});
    %    R(ind,ind) = 0;
    %end
    
    % The guard is identity
    guard.configure();

end