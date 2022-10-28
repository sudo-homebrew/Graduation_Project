%RECEIVERPOSITION Estimate GNSS receiver position and velocity
%   lla = RECEIVERPOSITION(p, satPos) returns the receiver position, lla,
%   estimated from the pseudoranges, p, in meters, and the satellite
%   positions, satPos, in meters in the Earth-Centered-Earth-Fixed (ECEF)
%   coordinate system. The output position, lla, is specified in geodetic
%   coordinates in (latitude-longitude-altitude) in (degrees, degrees,
%   meters) respectively.
%
%   [lla, gnssVel] = RECEIVERPOSITION(..., pdot, satVel) returns the
%   receiver velocity, gnssVel, estimated from the pseudorange rates, in
%   meters per second, and the satellite velocities, satVel, in meters per
%   second in the ECEF coordinate system. The output velocity, gnssVel, is
%   specified in the North-East-Down (NED) coordinate system.
%
%   [lla, gnssVel, hdop, vdop] = RECEIVERPOSITION(...) returns horizontal
%   dilution of precision, hdop, and the vertical dilution of precision,
%   vdop, associated with the position estimate.
%
%   Example:
%       recPos = [42 -71 50];
%       recVel = [1 2 3];
%       t = datetime('now');
%       % Obtain satellite positions and velocities at the current time.
%       [gpsSatPos, gpsSatVel] = gnssconstellation(t);
%       [az, el, vis] = lookangles(recPos, gpsSatPos);
%       [p, pdot] = pseudoranges(recPos, gpsSatPos, recVel, gpsSatVel);
%       [lla, gnssVel] = receiverposition(p(vis), gpsSatPos(vis,:), ...
%           pdot(vis), gpsSatVel(vis,:));
%
%   See also gnssconstellation, pseudoranges, lookangles, gnssSensor.

 
%   Copyright 2020 The MathWorks, Inc.

