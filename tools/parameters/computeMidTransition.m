function [ newParameters ] = computeMidTransition( tauMid, dtau, a0, yMid, dyMid, yF )

% First compute the end transition
newParameters = a0;

% Then blend the middle
nParams  = length(a0(1,:));
nOutputs = length(a0(:,1));
reserveParams = [1:2, 5:8];
updateParams = [3:4];

tau_0 = tauMid;
dtau_0 = dtau;
ya0 = yMid;
Lfya0 = dyMid;
Y = [ya0, Lfya0];

for i = 1:nOutputs
    index = i;
    a_temp = zeros(length(updateParams),nParams);
    a_temp(:,reserveParams) = ones(length(updateParams),1) * newParameters(index,reserveParams);
    a_temp(:,updateParams) = eye(length(updateParams));
    
    yd0   = bezier(a_temp,tau_0);
    Lfyd0 = dbezier(a_temp,tau_0)*dtau_0;
    
    a_rem = [yd0'; Lfyd0']\Y(index,:)';
    
    newParameters(index,updateParams) = a_rem;
end

end

