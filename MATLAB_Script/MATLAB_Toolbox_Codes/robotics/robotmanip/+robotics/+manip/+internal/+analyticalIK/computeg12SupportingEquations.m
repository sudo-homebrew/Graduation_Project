function g = computeg12SupportingEquations(a1,a2,alpha1,alpha2,f1,f2,f3,s2,theta2)
%computeg12SupportingEquations Compute g1 and g2 supporting equations
%   This function computes g1 and g2, which are functions of theta2 and
%   theta3.
 
%   Copyright 2020 The MathWorks, Inc.

% Initialize output
g = zeros(1,2);
 
% Compute component terms
t2 = cos(alpha1);
t3 = cos(alpha2);
t4 = sin(alpha2);
t5 = cos(theta2);
t6 = sin(theta2);
t7 = a2+f1;
t8 = f2.*t3;
t9 = f3.*t4;
t10 = -t9;
t11 = t8+t10;
 
% Assemble outputs
g(1) = a1+t5.*t7-t6.*t11;
g(2) = sin(alpha1).*(s2+f2.*t4+f3.*t3)-t2.*t6.*t7-t2.*t5.*t11;
 
end
 
