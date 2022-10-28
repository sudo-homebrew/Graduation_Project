%LOOKANGLES Satellite look angles from receiver and satellite positions
%   [az, el, vis] = LOOKANGLES(recPos, satPos) returns the look angles,
%   azimuth az and elevation el in degrees, of the satellites using the
%   satellite positions satPos in the Earth-Centered-Earth-Fixed (ECEF)
%   coordinate system in meters and the receiver position recPos in
%   geodetic coordinates in (latitude-longitude-altitude) in
%   (degrees, degrees, meters). The output vis is a logical array specifying
%   the visibility of each satellite. The visibility is determined using
%   the default receiver mask angle of 10 degrees.
%
%   [az, el, vis] = LOOKANGLES(..., maskAngle) returns the look angles and
%   visibilities of satellites with a mask angle maskAngle.
%
%   Example:
%       recPos = [42 -71 50];
%       t = datetime('now');
%       gpsSatPos = gnssconstellation(t);
%       maskAngle = 5;
%       [az, el, vis] = lookangles(recPos, gpsSatPos, maskAngle);
%       fprintf('%d satellites visible at %s.\n', nnz(vis), t);
%
%   See also gnssconstellation, pseudoranges, receiverposition, gnssSensor.

 
%   Copyright 2020 The MathWorks, Inc.

