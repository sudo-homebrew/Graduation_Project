function [satPos, satVel, satIDs] = gnssconstellation(t, varargin)
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

%#codegen

coder.extrinsic('matlabshared.internal.gnss.GPSTime.getGPSTime');

validateattributes(t, {'datetime'}, {'scalar', 'finite'});

% Input argument t must be a constant during code generation.
[gpsWeek, tow] = coder.const(@matlabshared.internal.gnss.GPSTime.getGPSTime, t);

[~, ~, mu, OmegaEDot] = fusion.internal.frames.wgs84ModelParams;

if (nargin > 1)
    narginchk(3, 3);
    validatestring(varargin{1}, "RINEXData", "gnssconstellation", "RINEXData", 2);
    
    % Validate the timetable input.
    orbitParams = varargin{2};
    validateattributes(orbitParams, {'timetable'}, {});
    requiredVarNames = {'Toe', 'sqrtA', 'Delta_n', 'M0', ...
        'Eccentricity', 'omega', 'i0', 'IDOT', ...
        'Cis', 'Cic', 'Crs', 'Crc', 'Cus', 'Cuc', ...
        'OMEGA_DOT', 'OMEGA0', 'SatelliteID'};
    varNames = orbitParams.Properties.VariableNames;
    for ii = 1:numel(requiredVarNames)
        reqVarName = requiredVarNames{ii};
        coder.internal.errorIf(~any(matches(varNames, reqVarName)), ...
            "nav_positioning:gnssconstellation:MissingOrbitParameter", ...
            reqVarName);
    end
    % Check that one of the expected satellite system weeks exist.
    isSatSys = @(satSys) any(matches(varNames, satSys + "Week"));
    if isSatSys("GPS")
        weekNum = orbitParams.GPSWeek;
    elseif isSatSys("GAL")
        weekNum = orbitParams.GALWeek;
    else
        error(message("nav_positioning:gnssconstellation:MissingWeek", "GPSWeek", "GALWeek")); 
    end
    
    toe = orbitParams.Toe;
    ARef = orbitParams.sqrtA.^2;
    deltan0 = orbitParams.Delta_n;
    M0 = orbitParams.M0;
    e0 = orbitParams.Eccentricity;
    omega0 = orbitParams.omega;
    i0 = orbitParams.i0;
    iDot = orbitParams.IDOT;
    Cis = orbitParams.Cis;
    Cic = orbitParams.Cic;
    Crs = orbitParams.Crs;
    Crc = orbitParams.Crc;
    Cus = orbitParams.Cus;
    Cuc = orbitParams.Cuc;
    OmegaRefDot = orbitParams.OMEGA_DOT;
    Omega0 = orbitParams.OMEGA0;
    satIDs = orbitParams.SatelliteID;
else
    [e0, i0, OmegaRefDot, ARef, Omega0, omega0, M0] ...
        = matlabshared.internal.gnss.OrbitalParameters.nominal;
    weekNum = 0;
    toe = 0;
    deltan0 = 0;
    iDot = 0;
    Cis = 0;
    Cic = 0;
    Crs = 0;
    Crc = 0;
    Cus = 0;
    Cuc = 0;
end

[satPos, satVel] = matlabshared.internal.gnss.orbitParametersToECEF(...
    gpsWeek, tow, weekNum, toe, ...
    ARef, 0, 0, ...
    mu, deltan0, 0, ...
    M0, e0, omega0, ...
    i0, iDot, ...
    Cis, Cic, Crs, Crc, Cus, Cuc, ...
    OmegaEDot, OmegaRefDot, Omega0, 0);
end
