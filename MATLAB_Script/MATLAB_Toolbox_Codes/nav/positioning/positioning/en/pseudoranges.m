%PSEUDORANGES Pseudoranges between GNSS receiver and satellites
%   p = PSEUDORANGES(recPos, satPos) returns the pseudoranges, in meters,
%   between the receiver at position recPos and the satellites at positions
%   satPos. The receiver position is specified in geodetic coordinates
%   (latitude-longitude-altitude) in (degrees, degrees, meters). The
%   satellite positions are specified as an S-by-3 matrix in meters in the
%   Earth-Centered-Earth-Fixed (ECEF) coordinate system. S is the number of
%   satellites.
%
%   [p, pdot] = PSEUDORANGES(___, recVel, satVel) returns the pseudorange
%   rates, pdot, in meters per second, between the receiver and satellites.
%   The receiver velocity, recVel, is specified in meters per second in the
%   North-East-Down (NED) coordinate system. The satellite velocities,
%   satVel, are specified as an S-by-3 matrix in meters per second in the
%   ECEF coordinate system. S is the number of satellites.
%
%   [p, pdot] = PSEUDORANGES(___, 'RangeAccuracy', rangeStd, ...
%   'RangeRateAccuracy', rangeRateStd) returns the pseudoranges and
%   pseudorange rates with random noises specified by rangeStd and
%   rangeRateStd, in meters and meters per second, respectively. The
%   default value of rangeStd and rangeRateStd are 1 and 0.02,
%   respectively.
%
%   Example:
%       recPos = [42 -71 50];
%       recVel = [1 2 3];
%       t = datetime('now');
%       [gpsSatPos, gpsSatVel] = gnssconstellation(t);
%       [p, pdot] = pseudoranges(recPos, gpsSatPos, recVel, gpsSatVel);
%
%   See also gnssconstellation, lookangles, receiverposition, gnssSensor.

 
%   Copyright 2020 The MathWorks, Inc.

