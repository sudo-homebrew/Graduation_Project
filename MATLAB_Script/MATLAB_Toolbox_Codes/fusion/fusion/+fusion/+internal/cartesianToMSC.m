function msc = cartesianToMSC(x)
% cartesianToMSC converts a cartesian state to modified spherical coordinate
% state.
% mscState = cartesianToMSC(cartState)
% cartState is defined as [x;vx;y;vy;z;vz]. 
% mscState is defined as [az omega el elrate inv-range range-rate/range];

% This is an internal function and may be modified in a future release.

% Copyright 2018 The MathWorks Inc.

%#codegen

X = x(1,:);
Y = x(3,:);
Z = x(5,:);
Vx = x(2,:);
Vy = x(4,:);
Vz = x(6,:);

invR = 1./(X.^2 + Y.^2 + Z.^2).^(1/2);
az = atan2(Y,X);
el = -atan2(-Z,(X.^2 + Y.^2).^(1/2));
dRinvR = (X.*Vx + Y.*Vy + Z.*Vz)./(X.^2 + Y.^2 + Z.^2);
omega = (X.*Vy - Y.*Vx)./(X.^2 + Y.^2).*cos(atan2(-Z,sqrt(X.^2 + Y.^2)));
elDot = -(Z.*(X.*Vx + Y.*Vy) - Vz.*(X.^2 + Y.^2))./((X.^2 + Y.^2).^(1/2).*(X.^2 + Y.^2 + Z.^2));
msc = [az;omega;el;elDot;invR;dRinvR];

end