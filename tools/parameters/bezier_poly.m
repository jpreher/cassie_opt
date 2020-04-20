%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% BEZIER EVALUATOR
% Calculate set of points [s] on a Bezier curve at s in [0,1] for some
%   parameter set
% Author: Jake Reher
% Lab: AMBER Lab
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ b ] = bezier_poly(s, alpha)
M = length(alpha)-1;
b = zeros(length(s),1);

for i=1:length(s)
    x = s(i);
    for j=1:length(alpha)
        k = j-1;
        b(i) = b(i) + alpha(j) * (factorial(M) / (factorial(k)*factorial(M-k))) ...
            * x^(k) * (1-x)^(M-k);
    end
end

end