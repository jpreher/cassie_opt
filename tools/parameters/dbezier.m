%% Function Name: dbezier
%
% Description: A function for computing the first derivative of a set of
%   Bezier polynomials.
%   
% Inputs:
%   coeff: Matrix of n polynomials with order m-1
%   s: Point of the curve is evaluated at P(s)
%
% Outputs:
%   fcn: The first derivative of the Bezier polynomials at s. 
%
% Author: Jake Reher, jreher@caltech.edu
%
% Date: August 31, 2017
% ________________________________________

function fcn = dbezier(coeff,s)

    dcoeff = differentiateBezierCoefficients(coeff);
    fcn = bezier(dcoeff,s);
    
end