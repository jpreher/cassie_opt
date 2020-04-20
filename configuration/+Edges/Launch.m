%% Function: RightLaunch
%
% Description: Apply domain transition when:
%   Launching from Stance-Phase to Flight-Phase, NO stance leg changed
%
% Author: Wenlong Ma, wma@caltech.edu
%         Jake Reher, jreher@caltech.edu
% ______________________________________

function guard = Launch(tar)

    guardName = [tar.Name(1:end-2), 'Launch'];
    guard = RigidImpact(guardName, tar, 'groundForce');
    
    %% Set the impact constraint
    if ~isempty(fields(tar.HolonomicConstraints))
        guard.addImpactConstraint(struct2array(tar.HolonomicConstraints));
    end
    
    %% Only for the case when the launch does not have rigid impact.
    %  instead, a trivival (identity mapping) domain transition:
    constr = fields(guard.ImpactConstraints);
    guard.removeImpactConstraint(constr);
    guard.configure();
    % Otherwise, comment this section.

end