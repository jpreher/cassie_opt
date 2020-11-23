%% Function Name: differentiateBezierCoefficients
%
% Description: Differentiates the coefficients for a single Bezier
%   polynomial.
%   
% Inputs:
%   coeff: Vector of coefficients for one Bezier polynomial.
%
% Outputs:
%   dcoeff: The derivative of coeff.  
%
% Author: Jake Reher, jreher@caltech.edu
%
% Date: August 31, 2017
% ________________________________________

function dcoeff = differentiateBezierCoefficients(coeff)

	M = size(coeff,2)-1;
	A = zeros(M,M+1);
	
	for i=0:1:M-1
		A(i+1,i+1) = -(M-i)*nchoosek(M,i)/nchoosek(M-1,i);
		A(i+1,i+2) = (i+1)*nchoosek(M,i+1)/nchoosek(M-1,i);
	end
	
	A(M,M+1) = M*nchoosek(M,M);
	dcoeff = coeff*A';
    
end