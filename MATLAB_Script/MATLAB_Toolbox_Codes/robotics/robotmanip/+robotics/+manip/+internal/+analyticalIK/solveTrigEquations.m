function theta = solveTrigEquations(a,b,c)
%solveTrigEquations Solve equations of the form a*cos(theta) + b*sin(theta) = c for theta
%   This function solves the common trigonometric equality by equating the
%   solution to cos(phi)sin(theta) + sin(phi)cos(theta) = sin(phi + theta).
%   The function returns two possible solutions for theta.
 
%   Copyright 2020 The MathWorks, Inc.
 
theta = nan(1,2);

% Handle the trivial case
if isEqualWithinTolerance(a,0) && isEqualWithinTolerance(b,0) && isEqualWithinTolerance(c,0)
    theta(1) = 0;
    return;
elseif isEqualWithinTolerance(a,0) && isEqualWithinTolerance(b,0) && ~isEqualWithinTolerance(c,0)
    return;
end

% As long as a or b are nonzero, a set of general solutions may be found
d = sqrt(a^2 + b^2);
cPrime = c/d;
if cPrime < 1 || isEqualWithinTolerance(cPrime,1)
    % Throw out the imaginary solutions, which occur when cPrime > 1
    phi1 = atan2(a,b);
    phi2 = atan2(-a,-b);
    theta(1) = real(asin(complex(cPrime))) - phi1;
    theta(2) = -real(asin(complex(cPrime))) - phi2;
end
 
end
