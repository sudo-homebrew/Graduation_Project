function polySolns = solveQuarticPolynomial(polyCoeffs)
%solveQuarticPolynomial Solve 4th order polynomial
%   This function accepts a vector [A B C D E] of coefficients and solves
%   the equation of the form Ax^4 + Bx^3 + Cx^2 + Dx + E = 0 using an
%   analytical formulation.
%
%   Reference:
%       Weisstein, Eric W. "Quartic Equation." From MathWorld--A
%       Wolfram Web Resource. https://mathworld.wolfram.com/QuarticEquation.html
 
%   Copyright 2020 The MathWorks, Inc.
 
% Analytical methods are not robust to division by zero, so filter out
% cases that are actually linear, quadratic, or cubic
isCoeffZero = [...
    isEqualWithinTolerance(polyCoeffs(1),0) ...
    isEqualWithinTolerance(polyCoeffs(2),0) ...
    isEqualWithinTolerance(polyCoeffs(3),0) ...
    isEqualWithinTolerance(polyCoeffs(4),0) ...
    isEqualWithinTolerance(polyCoeffs(5),0) ...
    ];
    
if all(isCoeffZero)
    % All coefficients are zero. This is a trivial solution; output zero
    polySolns = 0;
elseif all(isCoeffZero([1 2 3]))
    % The first three coefficients are zero; this problem is linear
    polySolns = -polyCoeffs(5)/polyCoeffs(4);
elseif all(isCoeffZero([1 2]))
    % The first two coefficients are zero; this problem is quadratic
    polySolns = [...
        (-polyCoeffs(4) + sqrt(complex(polyCoeffs(4)^2 - 4*polyCoeffs(3)*polyCoeffs(5))))/(2*polyCoeffs(3)); ...
        (-polyCoeffs(4) - sqrt(complex(polyCoeffs(4)^2 - 4*polyCoeffs(3)*polyCoeffs(5))))/(2*polyCoeffs(3))];
elseif isCoeffZero(1)
    % The first coefficient are zero; this problem is cubic. Solve this
    % using the cubic solver subroutine, which accepts inputs in standard
    % polynomial form.
    polySolns = solveCubicPolynomial(polyCoeffs(3)/polyCoeffs(2), polyCoeffs(4)/polyCoeffs(2), polyCoeffs(5)/polyCoeffs(2));
else
    % This problem is quartic
    
    % Rewrite in standard polynomial form. Be sure to use different
    % variables than are used elsewhere in code, as this may otherwise
    % create global variables that corrupt data in other parts of the
    % generated solution.
    polyA0 = polyCoeffs(5)/polyCoeffs(1);
    polyA1 = polyCoeffs(4)/polyCoeffs(1);
    polyA2 = polyCoeffs(3)/polyCoeffs(1);
    polyA3 = polyCoeffs(2)/polyCoeffs(1);
 
    % Compute a real solution to the resolvent cubic polynomial
    cubicRoots = solveCubicPolynomial(-polyA2, polyA1*polyA3 - 4*polyA0, 4*polyA2*polyA0- polyA1^2 - polyA3^2*polyA0);
    
    % Select a real-valued root  
    resCubicRealRoot = cubicRoots(1);
    for i = 1:3
        if imag(cubicRoots(i)) == 0
            resCubicRealRoot = cubicRoots(i);
            continue;
        end
    end
 
    % To minimize code generation issues, declare contents of the square
    % roots to be complex to avoid unexpected complex terms
    
    % Compute supporting elements
    R = sqrt(complex(1/4*polyA3^2 - polyA2 + resCubicRealRoot));
 
    if isEqualWithinTolerance(R, 0)
        D = sqrt(complex(3/4*polyA3^2 - 2*polyA2 + 2*sqrt(resCubicRealRoot^2 - 4*polyA0)));
        E = sqrt(complex(3/4*polyA3^2 - 2*polyA2 - 2*sqrt(resCubicRealRoot^2 - 4*polyA0)));
    else
        D = sqrt(complex(3/4*polyA3^2 - R^2 - 2*polyA2 + 1/4*(4*polyA3*polyA2 - 8*polyA1 - polyA3^3)/R));
        E = sqrt(complex(3/4*polyA3^2 - R^2 - 2*polyA2 - 1/4*(4*polyA3*polyA2 - 8*polyA1 - polyA3^3)/R));
    end
 
    % Assemble the four solutions
    polySolns = [...
        -1/4*polyA3 + 1/2*R + 1/2*D; ...
        -1/4*polyA3 + 1/2*R - 1/2*D; ...
        -1/4*polyA3 - 1/2*R + 1/2*E; ...
        -1/4*polyA3 - 1/2*R - 1/2*E];
end
     
 
    function cubicRoots = solveCubicPolynomial(b2, b1, b0)
        %solveCubicPolynomial Solve for a real-valued root to a cubic polynomial
        %   This function solves for a real root of the cubic polynomial in
        %   the form x^3 + b2*x^2 + b1*x + b0 = 0. This type of polynomial
        %   has three roots, two of which may be complex.
        
        % Use Cardano's formula
        cubicQ = (3*b1 - b2^2)/9;
        cubicR = (9*b2*b1 - 27*b0 - 2*b2^3)/54;
        cubicD = cubicQ^3 + cubicR^2;
        
        % Make sure to call square roots with sqrt(complex()) to ensure
        % code generation support when numbers are negative
        if cubicD < 0
            cubicS = (cubicR + sqrt(complex(cubicD)))^(1/3);
            cubicT = (cubicR - sqrt(complex(cubicD)))^(1/3);
        else
            % When D is greater than zero, use nthroot, which ensures the
            % real-valued root is returned
            cubicS = nthroot((cubicR + sqrt(cubicD)),3);
            cubicT = nthroot((cubicR - sqrt(cubicD)),3);
        end
        
        cubicRoots = [...
            -1/3*b2 + (cubicS + cubicT); ...
            -1/3*b2 - 1/2*(cubicS + cubicT) + 1/2*1i*sqrt(3)*(cubicS - cubicT); ...
            -1/3*b2 - 1/2*(cubicS + cubicT) - 1/2*1i*sqrt(3)*(cubicS - cubicT); ...
            ];
    end
end