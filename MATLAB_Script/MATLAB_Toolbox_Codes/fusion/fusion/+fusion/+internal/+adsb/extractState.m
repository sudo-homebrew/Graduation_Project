function state = extractState( message, format)
%extractState Extract the state from an ADS-B message
% state = extractState(message) parses the
% message struct, specified as an ads-b message structure, to
% extract the ECEF state. state is a 6-element column vector defined
% as [X; Vx; Y; Vy; Z; Vz], where [X, Y, Z] are the ECEF
% position coordinates and [Vx, Vy, Vz] are the ECEF velocity
% coordinates
%
% This is an internal function and may be removed or modified in a future
% release.

%   Copyright 2020 The MathWorks, Inc.

%#codegen

lat = message.Latitude;
long = message.Longitude;
alt = message.Altitude;
Vn = message.Vnorth;
Ve = message.Veast;
Vd = - message.ClimbRate;


ecefPos = fusion.internal.frames.lla2ecef([lat long alt]);
ecefVel = fusion.internal.frames.ned2ecefv([Vn Ve Vd], lat, long);

% Form state vector [x, Vx, y, Vy, z, Vz]
state = [ecefPos(1); ecefVel(1);...
    ecefPos(2); ecefVel(2);...
    ecefPos(3); ecefVel(3)];

if nargin > 2 && strcmp(format,'Position')
    state = ecefPos(:);
end

end