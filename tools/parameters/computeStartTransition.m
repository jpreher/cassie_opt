function [ newParameters ] = computeStartTransition( dtau, phaseRange, a0, yActual, dyActual, updateIndices )
% See Westervelt p195 for a description on updating the first two
% parameter columns based on the post impact state.

if nargin < 6
    updateIndices = 1: length(yActual);
end

nParams  = length(a0(1,:));
nOutputs = length(updateIndices);
reserveParams = 3:(length(a0(1,:)));
updateParams = 1:2;

tau_0 = 0;
dtau_0 = dtau;
ya0 = yActual;
Lfya0 = dyActual;
Y = [ya0, Lfya0];

newParameters = a0;

for i = 1:nOutputs
    index = updateIndices(i);
    a_temp   = zeros(2,nParams);
    a_rest   = zeros(1,nParams);

    a_rest(:,reserveParams) = newParameters(index,reserveParams);
    a_temp(:,updateParams) = eye(2);

    yd0r = bezier(a_rest,tau_0);
    Lfyd0r = dbezier(a_rest,tau_0)*dtau_0;

    yd0   = bezier(a_temp,tau_0);
    Lfyd0 = dbezier(a_temp,tau_0)*dtau_0;

    a_bias = [yd0r;Lfyd0r];
    Phi_rem = [yd0'; Lfyd0'];
    
    if isinf(inv(Phi_rem))
        continue
    end

    a_rem = Phi_rem\(Y(index,:)'-a_bias);
    newParameters(index,updateParams) = a_rem;
end


end


% Legacy
% % Start and end ranges
% p0 = phaseRange(1);
% pf = phaseRange(2);
% 
% % Bezier polynomial order
% M = size(a0,2) - 1;
% 
% % Allocate the new parameters
% newParameters = a0;
% 
% % Update the first parameters based on position
% newParameters(:,1) = yActual;
% 
% % Update the second parameters based on velocity
% newParameters(:,2) = ( (pf - p0) / (M) ) * (dyActual) + newParameters(:,1);




% Old
% newParameters(:,2) = ( (pf - p0) / (M * dtau) ) * (dyActual) + newParameters(:,1);


