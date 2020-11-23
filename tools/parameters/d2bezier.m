%% Function Name: d2bezier
%
% Description: A function for computing the second derivative of a set of
%   Bezier polynomials.
%   
% Inputs:
%   coeff: Matrix of n polynomials with order m-1
%   s: Point of the curve is evaluated at P(s)
%
% Outputs:
%   fcn: The second derivative of the Bezier polynomial at s. 
%
% Author: Jenna Reher, jreher@caltech.edu
%
% Date: August 31, 2017
% ________________________________________

function fcn = d2bezier(coeff,s)

	dcoeff  = differentiateBezierCoefficients(coeff);
	d2coeff = differentiateBezierCoefficients(dcoeff); 
	
	fcn = bezier(d2coeff, s);
    
end