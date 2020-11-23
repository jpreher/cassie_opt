%% Function Name: Clamp
%
% Description: 
%   
% Inputs:
%   value: 
%   lb: 
%   ub: 
%
% Outputs:
%   value: The final value between ub and lb
%
% Author: Jake Reher, jreher@caltech.edu
%
% Date: August 31, 2017
% ________________________________________

function value = Clamp(value, lb, ub)

sz = size(value);

assert(sz(2) == 1, 'Must  be a vector input!');

for i = 1:length(value)
    if value(i) < lb(i)
        value(i) = lb(i);
    elseif value(i) > ub(i)
        value(i) = ub(i);
    end
end

end

