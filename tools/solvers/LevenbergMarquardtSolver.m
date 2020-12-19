classdef LevenbergMarquardtSolver < handle
    
    % This class solves the general problem 
    % minimize 1/2 || r(x) || ^2
    %    x
    % subject to lb <= x <= ub
    % 
    % Reference: METHODS FOR NON-LINEAR LEAST SQUARES PROBLEMS 
    %            K. Madsen, H.B. Nielsen, O. Tingleff - 2004
    
    properties (GetAccess = public, SetAccess = public)
        % Iteration limits
        maxIter    
        
        % Optimality Tolerances
        xTolerance
        fTolerance
        
        % Levenberg-Marquardt Parameters 
        initialDamping
        minDamping
        maxDamping
        
        lb
        ub
    end
    
    properties (GetAccess = public, SetAccess = private)
        % Handle to problem formulation
        userFun
        
        % State information
        solution
        
        
        % Variable values
        iteration
        
        % Output status
        status
    end
    
    methods
        function obj = LevenbergMarquardtSolver(userFun, lowerBound, upperBound)
            obj.userFun = userFun;
            
            obj.solution = 0.*lowerBound;
                       
            for i = 1:numel(lowerBound)
                assert(lowerBound(i) < upperBound(i), 'Bounds are not consistent!');
                if ~isinf(lowerBound(i)) || isinf(upperBound(i))
                    obj.solution(i,1) = (lowerBound(i) + upperBound(i)) / 2;
                end
            end
            obj.lb = lowerBound;
            obj.ub = upperBound;
            
            % Specify constant values
            obj.maxIter = 150;
            obj.xTolerance = 1e-8;
            obj.fTolerance = 1e-8;
            obj.iteration = 0;
            obj.status = LevenbergMarquardtStatus.NOT_INTIALIZED;
            obj.initialDamping = 1e-3;
            obj.maxDamping = 1e7;
            obj.minDamping = 1e-7;
        end
        
        function [] = solve(obj, xGuess)
            % Set the initial guess if not specified
            if nargin < 2
                xGuess = 0.*obj.solution;
            end
            y = obj.boundedToUnconstrained(xGuess);
                        
            % Specify starting values for search
            obj.iteration = 0;
            obj.status = LevenbergMarquardtStatus.SOLVING;
            tau = obj.initialDamping;
            
            % First evaluation
            [f, J] = obj.userFun.evaluate(xGuess);
            F = (1/2) * (f' * f);
            A = J' * J;
            g = J' * f;
            mu = tau * max(diag(A));
            nu = 2;
            I = eye(length(f));
            x_old = xGuess;
            exitFlag = 0;
            
            % Main optimization routine
            for i = 1 : obj.maxIter 
                % Initialize iteration
                obj.iteration = obj.iteration + 1;
                
                % Compute step direction
                delta = - J' * ((J*J' + mu * I) \ f);
                
                % Compute gain
                x_new = obj.unconstrainedToBounded(y + delta);
                [f_new, J_new] = obj.userFun.evaluate(x_new);
                F_new = (1/2) * (f_new' * f_new);
                
                denominator = (1/2) * delta' * (mu*delta - g);
                gain = (F - F_new) / denominator;
                
                % Update the variables
                if ( F_new < F )
                    x_old = obj.unconstrainedToBounded(y);
                    y = obj.boundedToUnconstrained(x_new);
                    f = f_new;
                    F = F_new;
                    J = J_new;
                    g = J' * g;
                    mu = mu * max(1/3, 1 - (2*gain - 1)^2);
                    nu = 2;
                    valid = true;
                else
                    % Step not accepted
                    mu = min(mu*nu, obj.maxDamping);
                    nu = 2*nu;
                    valid = false;
                end
                    
                % Check termination criternion
                if abs(denominator) < eps
                    exitFlag = -1;
                else
                    if ( valid )
                        if norm( x_new - x_old ) < obj.xTolerance
                            exitFlag = 2;
                        elseif F < obj.fTolerance
                            exitFlag = 3;
                        else
                            exitFlag = -1;
                        end
                    else
                        exitFlag = -1;
                    end
                end
                if exitFlag > 0
                    break;
                end
            end
            
            obj.status = LevenbergMarquardtStatus(exitFlag);
            obj.solution = obj.unconstrainedToBounded(y);
        end
        
        function [] = setBounds(obj, lb, ub)
            obj.lb = lb;
            obj.ub = ub;
        end
        
        function [x, status] = getSolution(obj)
            x = obj.solution;
            status = obj.status;
        end
        
    end % public methods
    
    methods (Access = private)  
        function [x, dx] = unconstrainedToBounded(obj, y)
            %  Transforms variable from [-inf to inf] to [lb,ub]
            x = y;
            if nargout > 1
                dx = 0.*y;
            end
            
            for i = 1:length(y)
                if isfinite(obj.lb(i)) && isfinite(obj.ub(i)) % lower and upper bound
                    d = obj.ub(i) - obj.lb(i);
                    tmp = 2 * y(i) ./ d;
                    x(i) = (obj.lb(i)+obj.ub(i))/2 + d/2 .* sin(tmp);
                    if nargout > 1
                        dx(i) = cos(2*y(i)/(obj.ub(i)-obj.lb(i)));
                    end                    
                elseif isfinite(obj.lb(i)) && ~isfinite(obj.ub(i)) %  lower bound
                    x(i)= obj.lb(i) - 1 + sqrt(y(i).^2+1);
                    if nargout > 1
                        dx(i)= y(i)/sqrt(y(i).^2+1);
                    end
                elseif ~isfinite(obj.lb(i)) && isfinite(obj.ub(i)) %  upper bound
                    x(i)= obj.ub(i) + 1 - sqrt(y(i).^2+1);
                    if nargout > 1
                        dx(i)= -y(i)/sqrt(y(i).^2+1);
                    end
                else % no bound, should not get here!
                    x(i) = y(i);
                    if nargout > 1
                        dx(i) = 1;
                    end
                end
            end
        end
        
        function y = boundedToUnconstrained(obj, x)
            y = x;
            for i = 1:length(x)
                if isfinite(obj.lb(i)) && isfinite(obj.ub(i)) % lower and upper bound
                    d = obj.ub(i)-obj.lb(i);
                    y(i) = d/2 .* real(asin((2*x(i)-obj.ub(i)-obj.lb(i))./d));
                elseif isfinite(obj.lb(i)) && ~isfinite(obj.ub(i)) %  lower bound
                    y(i) = real(sqrt((obj.lb(i) -1 - x(i)).^2 - 1));
                elseif ~isfinite(obj.lb(i)) && isfinite(obj.ub(i)) %  upper bound
                    y(i) = real(sqrt((obj.ub(i) +1 - x(i)).^2 - 1));
                else % no bound, should not get here!
                    y(i) = x(i);
                end
            end
        end
        
    end % private methods
    
end
