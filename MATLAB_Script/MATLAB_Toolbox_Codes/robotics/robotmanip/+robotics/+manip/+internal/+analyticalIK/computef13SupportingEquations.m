function f = computef13SupportingEquations(a3,alpha3,s3,s4,theta3)
%computef13SupportingEquations Compute f1 to f3 supporting equations
%   This function computes f1 to f3, which are functions of theta3. For a
%   given robot with three consecutive revolute axes, the position of a
%   joint a distance s4 along the joint 3 axis can be described as:
%      P = T1*T2*T3*[0; 0; s4; 1], 
%   where Ti represent transformation matrices associated with links. Then
%   this equation may be rewritten as P = T1*T2*f. This function computes
%   the values of f that satisfy the rewritten equation.
 
%   Copyright 2020 The MathWorks, Inc.
 
% Initialize output
f = zeros(3,1);
 
% Compute component terms
t2 = sin(alpha3);
t3 = cos(theta3);
t4 = sin(theta3);
 
% Assemble outputs. Note that there is technically a fourth output, f(4) =
% 1, but its value is unused, so it is not computed or returned.
f(1) = a3.*t3+s4.*t2.*t4;
f(2) = a3.*t4-s4.*t2.*t3;
f(3) = s3 + s4.*cos(alpha3);
 
end
 
