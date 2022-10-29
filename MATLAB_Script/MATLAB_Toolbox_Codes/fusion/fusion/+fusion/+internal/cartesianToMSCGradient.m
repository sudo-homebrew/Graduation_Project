function grad = cartesianToMSCGradient(cart)
% cartesianToMSCGradient - Jacobian of cartesianToMSC conversion.
%
% This function returns the jacobian of the function zmsc = g(xcart)
% evaluated at the cartesian state.  
%
% This is an internal function and may be removed in future.
%

% Copyright 2018 The MathWorks Inc.

%#codegen

% No validation is performed in this function.

% Obtain position and velocity information
X = cart(1);
Y = cart(3);
Z = cart(5);
Vx = cart(2);
Vy = cart(4);
Vz = cart(6);

% The orientation of the cartesian state is assumed [x y z vx vy vz] for
% calculation. This is transformed into [x vx y vy z vz] later. 
dinvRbyX =  [-X/(X^2 + Y^2 + Z^2)^(3/2)
        -Y/(X^2 + Y^2 + Z^2)^(3/2)
        -Z/(X^2 + Y^2 + Z^2)^(3/2)
                                    0
                                    0
                                    0];
                                
dAzbyX =  [-Y/(X^2 + Y^2)
          X/(X^2 + Y^2)
                         0
                         0
                         0
                         0];

dElbyX = [(X*Z)/((X^2 + Y^2)^(1/2)*(X^2 + Y^2 + Z^2))
        (Y*Z)/((X^2 + Y^2)^(1/2)*(X^2 + Y^2 + Z^2))
          -(X^2 + Y^2)^(1/2)/(X^2 + Y^2 + Z^2)
                                                  0
                                                  0
                                                  0];
                                                   
dDrInvRbyX =  [-(Vx*X^2 + 2*Vy*X*Y + 2*Vz*X*Z - Vx*Y^2 - Vx*Z^2)/(X^2 + Y^2 + Z^2)^2
        -(- Vy*X^2 + 2*Vx*X*Y + Vy*Y^2 + 2*Vz*Y*Z - Vy*Z^2)/(X^2 + Y^2 + Z^2)^2
        -(- Vz*X^2 + 2*Vx*X*Z - Vz*Y^2 + 2*Vy*Y*Z + Vz*Z^2)/(X^2 + Y^2 + Z^2)^2
                                                                  X/(X^2 + Y^2 + Z^2)
                                                                  Y/(X^2 + Y^2 + Z^2)
                                                                  Z/(X^2 + Y^2 + Z^2)];
                                                              
dOmegabyX = [Vy/((Z^2/(X^2 + Y^2) + 1)^(1/2)*(X^2 + Y^2)) - (2*X*(Vy*X - Vx*Y))/((Z^2/(X^2 + Y^2) + 1)^(1/2)*(X^2 + Y^2)^2) + (X*Z^2*(Vy*X - Vx*Y))/((Z^2/(X^2 + Y^2) + 1)^(3/2)*(X^2 + Y^2)^3)
        (Y*Z^2*(Vy*X - Vx*Y))/((Z^2/(X^2 + Y^2) + 1)^(3/2)*(X^2 + Y^2)^3) - (2*Y*(Vy*X - Vx*Y))/((Z^2/(X^2 + Y^2) + 1)^(1/2)*(X^2 + Y^2)^2) - Vx/((Z^2/(X^2 + Y^2) + 1)^(1/2)*(X^2 + Y^2))
                                         -(Z*(Vy*X - Vx*Y))/((Z^2/(X^2 + Y^2) + 1)^(3/2)*(X^2 + Y^2)^2)
                                                             -Y/((Z^2/(X^2 + Y^2) + 1)^(1/2)*(X^2 + Y^2))
                                                              X/((Z^2/(X^2 + Y^2) + 1)^(1/2)*(X^2 + Y^2))
                                                                                                                0];
dElDotbyX = [- (2*X*Vz - Z*Vx)/((X^2 + Y^2)^(1/2)*(X^2 + Y^2 + Z^2)) - (2*X*(Z*(X*Vx + Y*Vy) - Vz*(X^2 + Y^2)))/((X^2 + Y^2)^(1/2)*(X^2 + Y^2 + Z^2)^2) - (X*(Z*(X*Vx + Y*Vy) - Vz*(X^2 + Y^2)))/((X^2 + Y^2)^(3/2)*(X^2 + Y^2 + Z^2))
 - (2*Y*Vz - Z*Vy)/((X^2 + Y^2)^(1/2)*(X^2 + Y^2 + Z^2)) - (2*Y*(Z*(X*Vx + Y*Vy) - Vz*(X^2 + Y^2)))/((X^2 + Y^2)^(1/2)*(X^2 + Y^2 + Z^2)^2) - (Y*(Z*(X*Vx + Y*Vy) - Vz*(X^2 + Y^2)))/((X^2 + Y^2)^(3/2)*(X^2 + Y^2 + Z^2))
                                                                                               (X*Vx + Y*Vy)/((X^2 + Y^2)^(1/2)*(X^2 + Y^2 + Z^2)) - (2*Z*(Z*(X*Vx + Y*Vy) - Vz*(X^2 + Y^2)))/((X^2 + Y^2)^(1/2)*(X^2 + Y^2 + Z^2)^2)
                                                                                                                                                                                                     (X*Z)/((X^2 + Y^2)^(1/2)*(X^2 + Y^2 + Z^2))
                                                                                                                                                                                                     (Y*Z)/((X^2 + Y^2)^(1/2)*(X^2 + Y^2 + Z^2))
                                                                                                                                                                                        -(X^2 + Y^2)^(1/2)/(X^2 + Y^2 + Z^2)];
% Transform jacobian to follow [x;vx;y;vy;z;vz] state.                                
idx  = [1 4 2 5 3 6];
dinvRbyX = dinvRbyX(idx);
dAzbyX = dAzbyX(idx);
dElbyX = dElbyX(idx);
dDrInvRbyX = dDrInvRbyX(idx);
dOmegabyX = dOmegabyX(idx);
dElDotbyX = dElDotbyX(idx);

grad = [dAzbyX dOmegabyX dElbyX dElDotbyX dinvRbyX dDrInvRbyX]';
% convert el to -el and elDot to -el
p = eye(6,'like',X);
p(3,3) = -p(3,3);
p(4,4) = -p(4,4);
grad = p*grad;
