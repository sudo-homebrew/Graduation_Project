%GNSSCONSTELLATION Satellite locations at specified time
%   [satPos,satVel] = GNSSCONSTELLATION(t) returns the satellite positions
%   and velocities at the datetime t. Positions and velocities are
%   specified in the Earth-Centered-Earth-Fixed (ECEF) coordinate system in
%   meters and meters per second, respectively. If the TimeZone for the
%   datetime is not specified, it is assumed to be UTC.
%
%   [satPos,satVel,satID] = GNSSCONSTELLATION(t,RINEXData=navData) 
%   returns the satellite positions, velocities, and IDs specified by the
%   RINEX Navigation Message data navData. 
%   
%   The satellite positions are calculated using the initial parameters 
%   (Table 30-I) and position calculations (Table 30-II) in the 
%   "IS-GPS-200K Interface Specification".
%
%   Example:
%       % Get the current satellite positions and velocities.
%       t = datetime("now","TimeZone","Local");
%       [satPos,satVel] = gnssconstellation(t);
%
%        % Get satellite positions, velocities, and IDs from GPS navigation
%        % message data from a RINEX file.
%        filename = "GODS00USA_R_20211750000_01D_GN.rnx"; 
%        data = rinexread(filename);
%        gpsData = data.GPS;
%        t = gpsData.Time(1);
%        [satPos,satVel,satID] = gnssconstellation(t,RINEXData=gpsData);
%
%   References:
%   [1] International GNSS Service (IGS), Daily 30-Second GPS Broadcast
%       Ephemeris Data, NASA Crustal Dynamics Data Information System
%       (CDDIS), Greenbelt, MD, USA, Jun. 24, 2021. Accessed on: Jun. 25,
%       2021. [Online]. Available:
%       https://dx.doi.org/10.5067/GNSS/gnss_daily_n_001.
%
%   See also lookangles, pseudoranges, receiverposition, gnssSensor.

 
%   Copyright 2020-2021 The MathWorks, Inc.

