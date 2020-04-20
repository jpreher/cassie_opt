%% Function: Impact
%
% Description: Applies a rigid impact to the nonstance leg.
%              used in optimization as the final guard - thus the relabelling matrix.
%
% Author: Jenna Reher, (v1, jreher@caltech.edu)
%         Wenlong Ma, (v2, wma@caltech.edu)
% _________________________________________________________________________

function guard = Impact(tar, impactFootName, isRelabel)
    
    if nargin<3
        isRelabel = false;
    end
    
    if isRelabel
        guard = RigidImpact([impactFootName,'ImpactRelabel'],tar,'nsf');
    else
        guard = RigidImpact([impactFootName,'Impact'], tar,'nsf');
    end

    %% Re-define the relabel matrix for optimization
    if isRelabel
        jointName = {tar.Joints.Name};
        
        % Indices of right leg joints
        qRLegIndices = find(strncmpi(jointName,'Right',5));
        
        % Indices of left leg joints
        qLLegIndices = find(strncmpi(jointName,'Left',4));
        
        swappingIndices = cumsum(ones(tar.numState,1));
        swappingIndices(qRLegIndices) = qLLegIndices;
        swappingIndices(qLLegIndices) = qRLegIndices;
        
        % find roll joints of both legs
        rollJoints = strfind(jointName,'Roll');
        rollJointIndices = find(~cellfun(@isempty,rollJoints));
        
        % find yaw joints of both legs
        yawJoints = strfind(jointName, 'Yaw');
        yawJointIndices = find(~cellfun(@isempty,yawJoints));
        
        swappingSign = ones(tar.numState,1);
        swappingSign(rollJointIndices) = -1*ones(numel(rollJointIndices),1);
        swappingSign(yawJointIndices)  = -1*ones(numel(yawJointIndices),1);

        if strcmp(tar.Joints(6).Name, 'BaseRotZ')
            qbIndices = 1:6;
            swappingIndices(qbIndices) = qbIndices;
            swappingSign(qbIndices(2)) = -1; % BasePosY
            swappingSign(qbIndices(4)) = -1; % base-roll: BaseRotX
            swappingSign(qbIndices(6)) = -1; % base-yaw : BaseRotZ
        end
        % No base swapping needed if 2D

        relabel = diag(swappingSign);
        R = relabel(swappingIndices,:);
        guard.R = R;
    end
    
    %% Set the impact constraint
    guard.addImpactConstraint(struct2array(tar.HolonomicConstraints));
    
end