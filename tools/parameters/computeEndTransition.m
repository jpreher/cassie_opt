function [ newParameters ] = computeEndTransition( dtau, a0, yEndDesired )
% See Westervelt p195 for a description on updating the first two
% parameter columns based on the post impact state.

nParams  = length(a0(1,:));
nOutputs = length(a0(:,1));
reserveParams = 1:(length(a0(1,:))-2);
updateParams = (length(a0(1,:))-1):(length(a0(1,:)));

tau_F = 1;
dtau_F = dtau;
yaF = yEndDesired;
LfyaF = dbezier(a0,tau_F)*dtau_F;
Y = [yaF, LfyaF];

newParameters = a0;

for i = 1:nOutputs
    index = i;
    a_temp   = zeros(2,nParams);
    a_rest   = zeros(1,nParams);

    a_rest(:,reserveParams) = newParameters(index,reserveParams);
    a_temp(:,updateParams) = eye(2);

    ydFr = bezier(a_rest,tau_F);
    LfydFr = dbezier(a_rest,tau_F)*dtau_F;

    ydF = bezier(a_temp,tau_F);
    LfydF = dbezier(a_temp,tau_F)*dtau_F;

    a_bias = [ydFr;LfydFr];
    Phi_rem = [ydF'; LfydF'];
    
    if isinf(inv(Phi_rem))
        continue
    end

    a_rem = Phi_rem\(Y(index,:)'-a_bias);
    newParameters(index,updateParams) = a_rem;
end

% Start and end ranges
%p0 = phaseRange(1);
%pf = phaseRange(2);

% Bezier polynomial order
%M = size(a0,2) - 1;

% Allocate the new parameters
%newParameters = a0;

% Update the first parameters based on position
%newParameters(:,1) = yActual;

% Update the second parameters based on velocity
% newParameters(:,2) = ( (pf - p0) / (M * dtau) ) * (dyActual) + newParameters(:,1);

%newParameters(:,2) = ( (pf - p0) / (M) ) * (dyActual) + newParameters(:,1);

end

