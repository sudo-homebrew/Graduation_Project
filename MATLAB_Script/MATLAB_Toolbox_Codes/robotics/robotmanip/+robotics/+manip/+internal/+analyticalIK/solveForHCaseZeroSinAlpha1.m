function [hSolns, hasFiniteNumSol, hasPiSoln] = solveForHCaseZeroSinAlpha1(a3, alpha1, alpha2, alpha3, d2, d3, d4, z3)
%solveForHCaseZeroSinAlpha1 Solve for h when sin(alpha1) is zero
%   To solve for theta3, it is necessary to reparameterize a trigonometric
%   equation in terms of a new parameter h = tan(that3/2) using the
%   Weierstrass equation. This function solves equation 3.26 in the Pieper
%   inverse kinematics solution for h in the case where DH parameter alpha1
%   is defined such that sin(alpha1) = 0. In that case, equation 3.26
%   becomes:
%      z3 = F4
%   Here F1 to F4 are functions of theta3 (and the constant DH parameters),
%   and z3 is a function of P, a known position input from the IK problem,
%   and the DH parameter d1:
%      z3 = P(3) - d1
%   Equation 3.26 may then may be reparameterized in h, producing a
%   quadratic polynomial in h. This function solves that polynomial for the
%   values of h given z3 and the DH parameters of the associated serial
%   manipulator.
 
%   Copyright 2020 The MathWorks, Inc.

% These solutions solve the equation above, z3 = F4, which becomes a
% quadratic in h = tan(theta3/2). As the polynomial is quadratic, the
% solution has a format that matches that of the solutions to the quadratic
% equation A*h^2 + B*h + C = 0.

% Compute the terms of the quadratic equation
A = z3 - cos(alpha1)*(d2 + d3*cos(alpha2) + d4*cos(alpha2)*cos(alpha3) + d4*sin(alpha2)*sin(alpha3));
B = -2*a3*cos(alpha1)*sin(alpha2);
C = z3 - cos(alpha1)*(d2 + d3*cos(alpha2) + d4*cos(alpha2)*cos(alpha3)) + d4*cos(alpha1)*sin(alpha2)*sin(alpha3);

% There are three possible solution cases
if isEqualWithinTolerance([A B C], [0 0 0])
    % The trivial case happens when the rotation of theta3 has no impact on
    % the end effector position (only on the orientation) because the next
    % joint lies on the axis of rotation. Since this equation is derived
    % from the position solution, any real-valued orientation solution
    % would work. Default to zero.
    hSolns = 0;
    hasFiniteNumSol = false;
elseif isEqualWithinTolerance(A, 0)
    % When the first term is zero, the equation is linear
    hSolns = -C/B;
    hasFiniteNumSol = true;
else
    % The equation is quadratic
    if B^2 - 4*A*C < 0
        % This solution will be complex
        h1 = (-B - sqrt(complex(B^2 - 4*A*C)))/(2*A);
        h2 = (-B + sqrt(complex(B^2 - 4*A*C)))/(2*A);
    else
        % This solution will be real
        h1 = real((-B - sqrt(B^2 - 4*A*C))/(2*A));
        h2 = real((-B + sqrt(B^2 - 4*A*C))/(2*A));
    end
    hSolns = [h1 h2];
    hasFiniteNumSol = true;
end

% Check if there is a solution at theta3 = pi, for which h is undefined, by
% checking if z3 = F4 (eq 3.26) is satisfied for that solution.
if hasFiniteNumSol
    localTheta = pi;
    fTerms = computef13SupportingEquations(a3, alpha3, d3, d4, localTheta);
        
    % The a1 and a2 terms are required to use the function that computes F,
    % but they only affect the first and third terms, which are unused
    % here. Therefore, dummy values are used.
    dummyA1 = 0;
    dummyA2 = 0;
    FTerms = computeF14SupportingEquations(dummyA1, dummyA2, alpha1, alpha2, fTerms(1), fTerms(2), fTerms(3), d2);
    
    hasPiSoln = isEqualWithinTolerance(FTerms(4), z3);
else
    hasPiSoln = true;
end
 
end