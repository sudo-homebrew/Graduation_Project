function [hSolns, hasFiniteNumSol, hasPiSoln] = solveForHCaseZeroA1(R3, a2, a3, alpha2, alpha3, d2, d3, d4)
%solveForHCaseZeroA1 Solve for h when a1 is zero
%   To solve for theta3, it is necessary to reparameterize a trigonometric
%   equation in terms of a new parameter h = tan(theta3/2) using the
%   Weierstrass equation. This function solves equation 3.25 in the Pieper
%   inverse kinematics solution for h in the case where DH parameter a1 is
%   zero. In that case, equation 3.25 becomes:
%      R3 = F3
%   Here F1 to F4 are functions of theta3 (and the constant DH parameters),
%   and R3 is a function of P, a known position input from the IK problem,
%   and the DH parameter d1:
%      R3 = P(1)^2 + P(2)^2 + (P(3) – d1)^2 
%   Equation 3.25 may then may be reparameterized in h, producing a
%   quadratic polynomial in h. This function solves that polynomial for the
%   values of h given R3 and the DH parameters of the associated serial
%   manipulator.
 
%   Copyright 2020 The MathWorks, Inc.
 
% These solutions solve the equation above, R3 = F3, which becomes a
% quadratic in h = tan(theta3/2). As the polynomial is quadratic, the
% solution has a format that matches that of the solutions to the quadratic
% equation A*h^2 + B*h + C = 0.

% Compute the terms of the quadratic equation
A = a2^2 - 2*a2*a3 - R3 + a3^2 + d2^2 + d3^2 + d4^2 + 2*d2*d3*cos(alpha2) + 2*d3*d4*cos(alpha3) + 2*d2*d4*cos(alpha2)*cos(alpha3) + 2*d2*d4*sin(alpha2)*sin(alpha3);
B = 4*a3*d2*sin(alpha2) + 4*a2*d4*sin(alpha3);
C = 2*a2*a3 - R3 + a2^2 + a3^2 + d2^2 + d3^2 + d4^2 + 2*d2*d3*cos(alpha2) + 2*d3*d4*cos(alpha3) + 2*d2*d4*cos(alpha2)*cos(alpha3) - 2*d2*d4*sin(alpha2)*sin(alpha3);

% There are three possible solution cases
if isEqualWithinTolerance([A B C], [0 0 0])
    % The trivial case happens whenever any value of theta3 solves the
    % quadratic derived from equation 3.25. In that case, any theta3 may
    % solve the problem, though it may be further constrained by equation
    % 3.26. Physically, this happens when theta3 has no impact on the
    % position solution, or when its solution is intertwined with theta2.
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
% checking if R3 = F3 (eq 3.25) is satisfied for that solution.
if hasFiniteNumSol
    localA1 = 0; 
    localTheta = pi;
    fTerms = computef13SupportingEquations(a3, alpha3, d3, d4, localTheta);
        
    % alpha1 is required to use the standard function that computes F, but
    % it only affects the fourth term, F4, which is unused here. Therefore,
    % a dummy value is used.
    dummyAlpha1 = 0;
    FTerms = computeF14SupportingEquations(localA1, a2, dummyAlpha1, alpha2, fTerms(1), fTerms(2), fTerms(3), d2);
    
    hasPiSoln = isEqualWithinTolerance(FTerms(3), R3);
else
    hasPiSoln = true;
end
 
end