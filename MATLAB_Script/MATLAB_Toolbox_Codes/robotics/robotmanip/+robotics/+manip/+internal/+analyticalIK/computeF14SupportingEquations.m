function F = computeF14SupportingEquations(a1,a2,alpha1,alpha2,f1,f2,f3,s2)
%computeF14SupportingEquations Compute intermediate variables F1 to F4
%   This function computes F1 to F4, which are intermediate variables in
%   Pieper's derivation that are functions of the theta3 joint position
%   (and constant parameters). The function accepts several DH parameters,
%   as well as the intermediate variables f1 to f3, which are functions of
%   theta3, and outputs the four F1 to F4 intermediate variables.
 
%   Copyright 2020 The MathWorks, Inc.
 
% Initialize output
F = zeros(1,4);
 
F(1) = a2+f1;
 
t2 = cos(alpha2);
t3 = sin(alpha2);
F(2) = -f2.*t2+f3.*t3;
 
t4 = f3.*t2;
t5 = f2.*t3;
F(3) = s2.*(t4+t5).*2.0+a2.*f1.*2.0+a1.^2+a2.^2+f1.^2+f2.^2+f3.^2+s2.^2;
 
F(4) = cos(alpha1).*(s2+t4+t5);
 
end