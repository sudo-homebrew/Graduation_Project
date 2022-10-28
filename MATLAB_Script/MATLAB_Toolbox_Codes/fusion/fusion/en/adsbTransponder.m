classdef adsbTransponder< matlab.System
%ADSBTRANSPONDER Automatic Dependent Surveillance - Broadcast transponder model
%  adsbTx = adsbTransponder(ICAO) creates an Automatic Dependent
%  Surveillance - Broadcast (ADS-B) transponder with a unique International
%  Civil Aviation Organization address that generates ADS-B messages. ICAO
%  is specified as a 6-element character vector or a string of length 6.
%
%  adsbTx = adsbTransponder(ICAO, 'Name', Value) creates an adsbTransponder
%  object by specifying its properties as name-value pair arguments.
%  Unspecified properties have default values. See the list of
%  properties below.
%
%  adsbTransponder properties:
%
%   ICAO             - Unique ICAO address
%   Category         - Category of the transponder platform
%   Callsign         - Callsign of the transponder platform
%   UpdateRate       - Transponder update rate (Hz)
%   GPS              - GPS sensor feeding the transponder
%
%  Step method syntax:
%
%  messages = step(adsbTx, position, velocity) generates ADS-B messages
%  from the vectors position and velocity. The position is
%  specified as [latitude, longitude, altitude]. Latitude and longitude
%  are in degrees with north and east being positive, and altitude is the
%  height above the WGS84 ellipsoid in meters. The velocity is specified
%  as cartesian coordinates along the NED axes at the input position.
%  Velocity is in meters per seconds.
%  The messages output is an array of ADS-B message structs as defined
%  <a href="matlab:help fusion.internal.interfaces.DataStructures/adsbMessageStruct">here</a>.
%
%  adsbTransponder methods:
%     clone             - Creates a copy of the adsbTransponder
%     isLocked          - Locked status (logical)
%     release           - Allows property value and input characteristics changes
%     <a href="matlab:help matlab.System/reset   ">reset</a>             - Resets states of the adsbTransponder
%     step              - Generates ADS-B messages
%
%   %Example
%   adsbTx = adsbTransponder('ABC123', 'GPS', gpsSensor('PositionInputFormat','Geodetic','HorizontalPositionAccuracy',100));
%   truePos = [42.753 31.896 10000]; % deg deg m
%   trueVel = [250 0 0]; % m/s
%   % Inspect ADS-B message output
%   adsbMessage = adsbTx(truePos, trueVel)
%
%   See also: adsbReceiver, adsbCategory, gpsSensor

     
    %   Copyright 2020 The MathWorks, Inc.

    methods
        function out=adsbTransponder
        end

        function out=getNumInputsImpl(~) %#ok<STOUT>
        end

        function out=gpsCovariance(~) %#ok<STOUT>
            % position
        end

        function out=loadObjectImpl(~) %#ok<STOUT>
        end

        function out=releaseImpl(~) %#ok<STOUT>
        end

        function out=resetImpl(~) %#ok<STOUT>
            % Reset GPS object
        end

        function out=saveObjectImpl(~) %#ok<STOUT>
        end

        function out=setupImpl(~) %#ok<STOUT>
            %Verify GPS is set to 'Geodetic' before locking
        end

        function out=stepImpl(~) %#ok<STOUT>
            % Input validation
        end

    end
    properties
        % Callsign Callsign of the tranponder platform Specify as as an
        % N-element character vector or a string of length N, where N is
        % less or equal than 8. When Callsign has less than 8 characters,
        % empty characters are used to reach 8. 
        % The default value is '        '.
        Callsign;

        % Category Category of the transponder platform
        % Specify the Category as an <a href="matlab:help adsbCategory">adsbCategory</a>.
        % The default value is adsbCategory(0) = No_Category_Information.
        Category;

        % GPS GPS Sensor
        % Specify as a gpsSensor object with 'PositionInputFormat' set to
        % 'Geodetic'. The 'SampleRate' of the gpsSensor object is
        % synchronized with the 'UpdateRate' of the adsbTransponder.
        % The default value is gpsSensor('PositionInputFormat','Geodetic').
        GPS;

        % ICAO Unique ICAO address
        % Specify the transponder 24 bits address as a 6-element character
        % vector or a string of length 6.
        % This property must be specified. This property has no default
        % value.
        ICAO;

        % UpdateRate Transponder update rate (hz)
        % Specify the update rate as a double.
        % The default value is 1.
        UpdateRate;

        % Internal transponder clock (s)
        pTime;

    end
end
