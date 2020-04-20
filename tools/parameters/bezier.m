%% Function Name: bezier
%
% Description: A function for computing n Bezier polynomial curves of order
%   m-1. Heavily uses nchoosek(N,K) (see MATLAB documentation)
%   
% Inputs:
%   coeff: Matrix of n polynomials with order m-1
%   s: Point of the curve is evaluated at P(s)
%
% Outputs:
%   fcn: The Bezier polynomials evaluated at s. 
%
% Author: Jake Reher, jreher@caltech.edu
%
% Date: August 31, 2017
% ________________________________________

function fcn = bezier(coeff,s) 
  
[n,m] = size(coeff);
[~,y] = size(s);

m=m-1; %Bezier polynomials have m terms for m-1 order

fcn = zeros(n,y);
for k = 0:1:m
    fcn = fcn + coeff(:,k+1).*singleterm_bezier(m,k,s);
end

return

% Function for facilitating the evaluation of each polynomial separately.
function val = singleterm_bezier(m,k,s)
  
if (k == 0)
    val = nchoosek(m,k).*(1-s).^(m-k);
elseif (m == k)
    val = nchoosek(m,k).*s.^(k);
else
    val = nchoosek(m,k).*s.^(k).*(1-s).^(m-k);
end

return