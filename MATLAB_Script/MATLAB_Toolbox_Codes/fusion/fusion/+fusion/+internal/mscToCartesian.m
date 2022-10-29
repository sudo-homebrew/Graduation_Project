function cartState = mscToCartesian(mscState)
% mscToCartesian - Convert state in MSC frame to cartesian frame.
% cartState = mscToCartesian(mscState);
% This function transforms a cartesian state to modified spherical
% coordinate state. 
% cartState = [x;vx;y;vy;z;vz];
%
% mscState = [az w el elDot 1/r rDot/r]

% This is an internal function and may be removed in a future release.

% Copyright 2018 The MathWorks Inc.

%#codegen

invR = mscState(5,:);
az = mscState(1,:);
el = -mscState(3,:);
dRinvR = mscState(6,:);
omega = mscState(2,:);
elDot = -mscState(4,:);


x = cos(az).*cos(el)./invR;
y = sin(az).*cos(el)./invR;
z = (-sin(el)./invR);
vx = (dRinvR.*cos(az).*cos(el) - omega.*sin(az) - elDot.*cos(az).*sin(el))./invR;
vy = (dRinvR.*sin(az).*cos(el) + omega.*cos(az) - elDot.*sin(az).*sin(el))./invR;
vz = ((-dRinvR.*sin(el) - elDot.*cos(el))./invR);

cartState = [x;vx;y;vy;z;vz];


