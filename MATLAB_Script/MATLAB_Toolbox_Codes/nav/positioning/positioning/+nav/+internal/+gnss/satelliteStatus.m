function [isSatVisible, satAz, satEl] = satelliteStatus(receiverLLA, maskAngle, satelliteECEF)
%SATELLITESTATUS Calculate satellite azimuth, elevation, and visibility
%
%   Input arguments:
%       RECEIVERLLA - Receiver position in geodetic coordinates. Latitude
%                     and longitude in degrees, altitude in meters.
%       MASKANGLE - Masking angle of receiver in degrees.
%       SATELLITEECEF - Satellite position in the ECEF frame in meters.
%
%   Output arguments:
%       ISSATVISIBLE - Logical array of satellite visibility. TRUE means
%                      the satellite is visible at the current receiver
%                      location.
%       SATAZ - Azimuth of satellites in degrees.
%       SATEL - Elevation of satellites in degrees.

%   Copyright 2020 The MathWorks, Inc.

%#codegen


numSats = size(satelliteECEF, 1);
numAx = size(satelliteECEF, 2);

receiverECEF = fusion.internal.frames.lla2ecef(receiverLLA);

cosLat = cosd(receiverLLA(:,1));
sinLat = sind(receiverLLA(:,1));
cosLon = cosd(receiverLLA(:,2));
sinLon = sind(receiverLLA(:,2));
rotECEF2NED = [-sinLat .* cosLon, -sinLat .* sinLon,  cosLat;...
                         -sinLon,            cosLon,       0;...
               -cosLat .* cosLon, -cosLat .* sinLon, -sinLat];
          
lineOfSightECEF = satelliteECEF - repmat(receiverECEF, numSats, 1);
range = vecnorm(lineOfSightECEF, 2, 2);
unitLineOfSightECEF = lineOfSightECEF ./ repmat(range, 1, numAx);
unitLineOfSightNED = (rotECEF2NED * unitLineOfSightECEF.').';

satAz = atan2d(unitLineOfSightNED(:,2), unitLineOfSightNED(:,1));
satAz(satAz < 0) = 360 + satAz(satAz < 0);
satEl = -asind(unitLineOfSightNED(:,3));

isSatVisible = (satEl >= maskAngle);

end