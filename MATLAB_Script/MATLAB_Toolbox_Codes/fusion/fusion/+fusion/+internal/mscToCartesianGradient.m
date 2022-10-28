function gradient = mscToCartesianGradient(state)
% mscToCartesianGradient - Jacobian of mscToCartesian conversion.
%
% jac = mscToCartesian(mscState);
% where jac(i,j) = dcart(i)/dmsc(j) where:
% cart = [x;vx;y;vy;z;vz]
% msc =  [az;w;el;elDot;1/r;rDot/r];

% This is an internal function and may be removed in a future release.

% Copyright 2018 The MathWorks Inc.

%#codegen

invR = state(5);
az = state(1);
el = -state(3);
dRinvR = state(6);
omega = state(2);
elDot = -state(4);

cosAzcosEl = cos(az)*cos(el);
sinAzcosEl = sin(az)*cos(el);
cosAzsinEl = cos(az)*sin(el);
sinAzsinEl = sin(az)*sin(el);

% The internal jacobian calculations are defined in this order
% [1/r az el rDot/r omega elRate];
idx = [2 5 3 6 1 4];

% Derivative of X wrt Zmsc
dXbyZmsc = [-(cosAzcosEl)/invR^2,...
        -(sinAzcosEl)/invR,...
        -(cosAzsinEl)/invR,...
        0 0 0];
dXbyZmsc = dXbyZmsc(idx);

% Derivative of Y wrt Zmsc
dYbyZmsc = [-(sinAzcosEl)/invR^2,...
         (cosAzcosEl)/invR,...
        -(sinAzsinEl)/invR,...
        0 0 0];
dYbyZmsc = dYbyZmsc(idx);

% Derivative of Z wrt Zmsc
dZbyZmsc = [sin(el)/invR^2, 0 -cos(el)/invR 0 0 0];
dZbyZmsc = dZbyZmsc(idx);

% Derivative of Vx wrt Zmsc
dVxbyZmsc = [(-dRinvR*cosAzcosEl + omega*sin(az) + elDot*cosAzsinEl)/invR^2,...
        -(dRinvR*sinAzcosEl + omega*cos(az) - elDot*sinAzsinEl)/invR,...
        -(dRinvR*cosAzsinEl + elDot*cosAzcosEl)/invR,...
         (cosAzcosEl)/invR,...
         -sin(az)/invR,...
         -(cosAzsinEl)/invR];
dVxbyZmsc = dVxbyZmsc(idx);


% Derivative of Vy wrt Zmsc     
dVybyZmsc = [(-dRinvR*sinAzcosEl - omega*cos(az) + elDot*sinAzsinEl)/invR^2,...
        (dRinvR*cosAzcosEl - omega*sin(az) - elDot*cosAzsinEl)/invR,...
        -(dRinvR*sinAzsinEl + elDot*sinAzcosEl)/invR,...
        sinAzcosEl/invR,...
        cos(az)/invR,...
        -sinAzsinEl/invR];
dVybyZmsc = dVybyZmsc(idx);
    
% Derivative of Vz wrt Zmsc
dVzbyZmsc = [(dRinvR*sin(el) + elDot*cos(el))/invR^2,...
        0,...
        (-dRinvR*cos(el) + elDot*sin(el))/invR,...
        -sin(el)/invR,...
        0,...
        -cos(el)/invR];
dVzbyZmsc = dVzbyZmsc(idx);    

% Use [x;vx;y;vy;z;vz] as cartesian state order.
gradient = [dXbyZmsc;dVxbyZmsc;dYbyZmsc;dVybyZmsc;dZbyZmsc;dVzbyZmsc];
p = eye(6,'like',el);
p(3,3) = -p(3,3);
p(4,4) = -p(4,4);
gradient = gradient*p;
end