classdef gnssSensor < matlab.System & fusion.internal.UnitDisplayer
%GNSSSENSOR Simulate GNSS position, velocity, and satellites
%   GNSS = GNSSSENSOR returns a System object, GNSS, that computes global 
%   navigation satellite system receiver readings based on local position 
%   and velocity input. The default reference position in geodetic 
%   coordinates is latitude: 0 degrees N, longitude: 0 degrees E, altitude:
%   0 m. 
%
%   GNSS = GNSSSENSOR("ReferenceFrame", RF) returns a GNSSSENSOR System
%   object that computes a global navigation satellite system receiver 
%   reading relative to the reference frame RF. Specify the reference frame
%   as 'NED' (North-East-Down) or 'ENU' (East-North-Up). The default value 
%   is 'NED'. 
% 
%   GNSS = GNSSSENSOR('Name', Value, ...) returns a GNSSSENSOR System 
%   object with each specified property name set to the specified value. 
%   You can specify additional name-value pair arguments in any order as 
%   (Name1,Value1,...,NameN, ValueN). 
%
%   To simulate GNSS receiver readings: 
%   1) Create the GNSSSENSOR object and set its properties. 
%   2) Call the object with arguments, as if it were a function. 
% 
%   [LLA, GNSSVEL, STATUS] = GNSS(POS, VEL) computes GNSS receiver readings
%   from position, POS, and velocity, VEL, inputs.
%    
%   Input arguments: 
% 
%       POS        Position of the GNSS receiver in the local navigation 
%                  coordinate system, specified as a real finite N-by-3  
%                  array in meters. N is the number of samples in the  
%                  current frame. 
% 
%       VEL        Velocity of the GNSS receiver in the local navigation 
%                  coordinate system, specified as a real finite N-by-3  
%                  array in meters per second. N is the number of samples  
%                  in the current frame. 
% 
%   Output arguments:  
% 
%       LLA        Position of the GNSS receiver in the geodetic latitude,
%                  longitude, and altitude coordinate system, returned as a
%                  real finite N-by-3 array. Latitude and longitude are in
%                  degrees with North and East being positive. Altitude is
%                  in meters. N is the number of samples in the current
%                  frame.
% 
%       GNSSVEL    Velocity of the GNSS receiver in the local navigation  
%                  coordinate system, returned as a real finite N-by-3
%                  array in meters per second. N is the number of samples
%                  in the current frame.
% 
%       STATUS    N-by-1 struct array containing additional information  
%                 about the receiver. The struct has the following fields: 
%                     SatelliteAzimuth   - Azimuth of visible satellites 
%                                          (deg) 
%                     SatelliteElevation - Elevation of visible satellites 
%                                          (deg) 
%                     HDOP               - Horizontal dilution of precision
%                     VDOP               - Vertical dilution of precision 
% 
%   Either single or double datatypes are supported for the inputs to  
%   GNSSSENSOR. Outputs have the same datatype as the input. 
% 
%   GNSSSENSOR methods: 
% 
%   step        - Simulate GNSS receiver readings 
%   release     - Allow property value and input characteristics to change,
%                 and release GNSSSENSOR resources
%   clone       - Create GNSSSENSOR object with same property values 
%   isLocked    - Display locked status (logical) 
%   reset       - Reset the states of the GNSSSENSOR 
%
%   GNSSSENSOR properties: 
% 
%   SampleRate            - Sampling rate of receiver (Hz) 
%   InitialTime           - Initial time of receiver clock  
%   ReferenceLocation     - Origin of local navigation reference frame 
%   MaskAngle             - Minimum elevation angle of satellites in view 
%                           (deg) 
%   RangeAccuracy         - Measurement noise in pseudoranges (m)
%   RangeRateAccuracy     - Measurement noise in pseudorange rates (m/s)
%   RandomStream          - Source of random number stream 
%   Seed                  - Initial seed of mt19937ar random number 
%
%   EXAMPLE: Generate GNSS position measurements from stationary input. 
% 
%       Fs = 1; 
%       numSamples = 1000; 
%       t = 0:1/Fs:(numSamples-1)/Fs; 
%       % LLA position for Natick, MA 
%       refLocNatick = [42.2825 -71.343 53.0352]; 
%  
%       gnss = gnssSensor('SampleRate', Fs, ... 
%           'ReferenceLocation', refLocNatick); 
%  
%       pos = zeros(numSamples, 3); 
%       vel = zeros(numSamples, 3); 
%  
%       lla = gnss(pos, vel); 
%
%   See also gpsSensor, imuSensor. 

%   Copyright 2020-2021 The MathWorks, Inc.

%#codegen

    properties (Nontunable)
        %SampleRate Sampling rate of receiver (Hz)
        %   Specify the sampling rate of the GNSS receiver as a positive
        %   scalar. The default value is 1.
        SampleRate(1,1) {mustBeNumeric, mustBePositive, mustBeFinite} = 1;
    end
    
    properties (Nontunable, Dependent)
        %InitialTime Initial time of receiver
        %   Specify the initial time of the GNSS receiver as a scalar
        %   datetime object. If the time zone on the datetime object is not
        %   specified, it is assumed to be UTC. The default value is
        %   datetime("now", "TimeZone", "UTC").
        InitialTime(1,1) datetime;
    end
    
    properties (Nontunable)
        %ReferenceLocation Reference location
        %   Specify the origin of the local coordinate system as a
        %   3-element vector in geodetic coordinates (latitude, longitude,
        %   and altitude). Altitude is the height above the reference
        %   ellipsoid model, WGS84. The reference location is in [degrees
        %   degrees meters]. The default value is [0 0 0].
        ReferenceLocation(1,3) {mustBeNumeric, mustBeReal, ...
            mustBeFinite} = [0, 0, 0];
    end
    
    properties
        %MaskAngle Elevation mask angle (deg)
        %   Specify the mask angle of the GNSS receiver as a scalar between
        %   0 and 90 in degrees. Satellites in view but below the mask
        %   angle are not used in the receiver positioning estimate. This
        %   property is tunable. The default value is 10.
        MaskAngle(1,1) {mustBeNumeric, mustBeReal, ...
            mustBeGreaterThanOrEqual(MaskAngle, 0), ...
            mustBeLessThanOrEqual(MaskAngle, 90)} = 10;
        %RangeAccuracy Measurement noise in pseudoranges (m)
        %   Specify the standard deviation of the noise in the pseudorange
        %   measurements as a real scalar in meters. This property is
        %   tunable. The default values is 1.
        RangeAccuracy(1,1) {mustBeNumeric, mustBeReal, ...
            mustBeNonnegative, mustBeFinite} = 1;
        %RangeRateAccuracy Measurement noise in pseudorange rates (m/s)
        %   Specify the standard deviation of the noise in the pseudorange
        %   rate measurements as a real scalar in meters per second. This
        %   property is tunable. The default values is 0.02.
        RangeRateAccuracy(1,1) {mustBeNumeric, mustBeReal, ...
            mustBeNonnegative, mustBeFinite} = 0.02;
    end
    
    properties (Nontunable)
        %RandomStream Random number source
        %   Specify the source of the random number stream as one of the
        %   following:
        %
        %   'Global stream' - Random numbers are generated using the
        %   current global random number stream.
        %   'mt19937ar with seed' - Random numbers are generated using the
        %   mt19937ar algorithm with the seed specified by the Seed
        %   property.
        %
        %   The default value is 'Global stream'.
        RandomStream = 'Global stream';
        %Seed Initial seed
        %   Specify the initial seed of an mt19937ar random number
        %   generator algorithm as a real, nonnegative integer scalar. This
        %   property applies when you set the RandomStream property to
        %   'mt19937ar with seed'. The default value is 67.
        Seed(1,1) uint32 {mustBeReal} = uint32(67);
    end
    
    properties (Nontunable, Hidden)
        ReferenceFrame = fusion.internal.frames.ReferenceFrame.getDefault;
    end
    
    properties (Constant, Hidden)
        SampleRateUnits = 'Hz';
        ReferenceLocationUnits = '[deg deg m]';
        MaskAngleUnits = 'deg';
        RangeAccuracyUnits = 'm';
        RangeRateAccuracyUnits = 'm/s';
        RandomStreamSet = matlab.system.StringSet({...
            'Global stream', ...
            'mt19937ar with seed'});
        ReferenceFrameSet = matlab.system.StringSet( ...
            fusion.internal.frames.ReferenceFrame.getOptions);
    end
    
    properties (Access = protected)
        % Used to store input for type casting.
        pInputPrototype;
        % Initial receiver position estimate in meters in ECEF coordinate
        % frame.
        pInitPosECEF = [0, 0, 0];
        % Initial receiver velocity estimate in meters per second in ECEF
        % coordinate frame.
        pInitVelECEF = [0, 0, 0];
        % Current simulation time in seconds.
        pCurrTime;
        
        % Used to keep InitialTime display consistent in MATLAB.
        pTimeZone = '';
        pFormat = 'dd-MMM-uuuu HH:mm:ss';
    end
    properties (Access = protected, Nontunable)
        % Date and time value from InitialTime property stored as a GPS
        % week number and a time of week (TOW) in seconds.
        pGPSWeek;
        pTimeOfWeek;
    end
    
    properties (Nontunable, Access = private)
        % Cached reference frame.
        pRefFrame;
    end
    
    properties (Access = private)
        % Random stream object (used in 'mt19937ar with seed' mode).
        pStream;
        % Random number generator state.
        pStreamState;
    end
    
    methods
        function obj = gnssSensor(varargin)
            coder.extrinsic('matlabshared.internal.gnss.GPSTime.getGPSTime');
            
            setProperties(obj, nargin, varargin{:});
            
            setInitialTime = true;
            for i = 1:2:numel(varargin)-1
                if strcmp(varargin{i}, 'InitialTime')
                    setInitialTime = false;
                    break;
                end
            end
            if setInitialTime
                if isempty(coder.target)
                    % In MATLAB, set the initial time to the local date and
                    % time.
                    obj.InitialTime = datetime('now', 'TimeZone', 'UTC');
                else
                    % In code generation, if the initial time is not set,
                    % only set the underlying values (GPS week and time of
                    % week) to a constant value.
                    [obj.pGPSWeek, obj.pTimeOfWeek] = coder.const( ...
                        @matlabshared.internal.gnss.GPSTime.getGPSTime);
                end
            end
        end
        
        function val = get.InitialTime(obj)
            coder.extrinsic('matlabshared.internal.gnss.GPSTime.getLocalTime');
            
            if isempty(coder.target)
                % Get datetime object from underlying values.
                val = matlabshared.internal.gnss.GPSTime.getLocalTime(obj.pGPSWeek, ...
                obj.pTimeOfWeek, obj.pTimeZone);
                val.TimeZone = obj.pTimeZone;
                val.Format = obj.pFormat;
            else
                % Get equivalent numeric datetime without time zone or
                % format.
                val = coder.const(@matlabshared.internal.gnss.GPSTime.getLocalTime, ...
                    obj.pGPSWeek, obj.pTimeOfWeek);
            end
        end
        function set.InitialTime(obj, val)
            coder.extrinsic('matlabshared.internal.gnss.GPSTime.getGPSTime');
            
            if isempty(coder.target)
                if isempty(val.TimeZone)
                    val.TimeZone = 'UTC';
                end
                % Save underlying values of datetime object.
                [obj.pGPSWeek, obj.pTimeOfWeek] ...
                    = matlabshared.internal.gnss.GPSTime.getGPSTime(val);
                obj.pTimeZone = val.TimeZone;
                obj.pFormat = val.Format;
                if ~contains(val.Format, ["x", "z"], 'IgnoreCase', true)
                        obj.pFormat = [val.Format, ' z'];
                end
            else
                % Save underlying numeric values of datetime object, while
                % ensuring it can be reduced to a constant value.
                nonConstDateTimeInCG = ~coder.internal.isConst(val);
                coder.internal.errorIf(nonConstDateTimeInCG, ...
                    'Coder:builtins:NonTunablePropertyNotConst', ...
                    'InitialTime');
                [obj.pGPSWeek, obj.pTimeOfWeek] = coder.const( ...
                    @matlabshared.internal.gnss.GPSTime.getGPSTime, val);
            end
        end
        
        function set.ReferenceLocation(obj, val)
            obj.ReferenceLocation = val;
            validateattributes(obj.ReferenceLocation(1), {'numeric'}, ...
                {'>=',-90,'<=',90}, ...
                '', ...
                'Latitude');
            validateattributes(obj.ReferenceLocation(2), {'numeric'}, ...
                {'>=',-180,'<=',180}, ...
                '', ...
                'Longitude');
        end
    end
    
    methods (Access = protected)
        displayScalarObject(obj);
        groups = getPropertyGroups(obj);
        flag = isInactivePropertyImpl(obj, prop);
        loadObjectImpl(obj, s, wasLocked);
        resetImpl(obj);
        s = saveObjectImpl(obj);
        setupImpl(obj, pos, vel);
        [lla, gpsVel, status] = stepImpl(obj, pos, vel);
        validateInputsImpl(obj, pos, vel);
    end
    
    methods (Static, Hidden)
        function flag = isAllowedInSystemBlock
            flag = false;
        end
    end
end
