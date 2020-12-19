function [ newParameters ] = computeEndBezier( tau_0, dtau, a0, y0, dy0, yEndDesired )
% See Westervelt p195 for a description on updating the first two
% parameter columns based on the post impact state.

nParams  = length(a0(1,:));
nOutputs = length(a0(:,1));
reserveParams = 1:2;
updateParams = 3:7;

tau_F = 1;
dtau_F = dtau;

dtau_0 = dtau;
ya0 = y0;
Lfya0 = dy0;

yaF = yEndDesired;
LfyaF = dbezier(a0,tau_F)*dtau_F;

Y = [ya0, Lfya0, yaF, LfyaF];

newParameters = a0;

for i = 1:nOutputs
    index = i;
    a_temp   = zeros(length(updateParams),nParams);
    a_rest   = zeros(1,nParams);

    a_rest(:,reserveParams) = newParameters(index,reserveParams);
    a_temp(:,updateParams) = eye(length(updateParams));
    
    yd0r = bezier(a_rest,tau_0);
    Lfyd0r = dbezier(a_rest,tau_0)*dtau_0;

    yd0   = bezier(a_temp,tau_0);
    Lfyd0 = dbezier(a_temp,tau_0)*dtau_0;

    ydFr = bezier(a_rest,tau_F);
    LfydFr = dbezier(a_rest,tau_F)*dtau_F;

    ydF = bezier(a_temp,tau_F);
    LfydF = dbezier(a_temp,tau_F)*dtau_F;

    a_bias = [yd0r;Lfyd0r;ydFr;LfydFr];
    Phi_rem = [yd0';Lfyd0';ydF'; LfydF'];
    
    if isinf(pinv(Phi_rem))
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

