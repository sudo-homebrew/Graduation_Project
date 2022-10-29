classdef adsbTransponder < matlab.System
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
    %     reset             - Resets states of the adsbTransponder
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
    
    % Reference:
    % ICAO, Doc. "9871: Technical Provisions for Mode S Services and Extended Squitter, AN/464." (2008).
    
    %   Copyright 2020 The MathWorks, Inc.
    
    %#codegen
    
    properties (Nontunable)
        % ICAO Unique ICAO address
        % Specify the transponder 24 bits address as a 6-element character
        % vector or a string of length 6.
        % This property must be specified. This property has no default
        % value.
        ICAO
        
        % Category Category of the transponder platform
        % Specify the Category as an <a href="matlab:help adsbCategory">adsbCategory</a>.
        % The default value is adsbCategory(0) = No_Category_Information.
        Category (1,1) adsbCategory = adsbCategory(0)
        
        % Callsign Callsign of the tranponder platform Specify as as an
        % N-element character vector or a string of length N, where N is
        % less or equal than 8. When Callsign has less than 8 characters,
        % empty characters are used to reach 8. 
        % The default value is '        '.
        Callsign = '        '
    end
    
    properties(Nontunable, Dependent)
        % UpdateRate Transponder update rate (hz)
        % Specify the update rate as a double.
        % The default value is 1.
        UpdateRate (1,1) double {mustBeNonsparse, mustBeFinite, mustBePositive}
    end
    
    properties
        % GPS GPS Sensor
        % Specify as a gpsSensor object with 'PositionInputFormat' set to
        % 'Geodetic'. The 'SampleRate' of the gpsSensor object is
        % synchronized with the 'UpdateRate' of the adsbTransponder.
        % The default value is gpsSensor('PositionInputFormat','Geodetic').
        GPS (1,1) gpsSensor
    end
    
    properties (Access = protected)
        % Internal transponder clock (s)
        pTime = 0
    end
    
    
    methods
        function set.ICAO(obj,val)
            if isValidICAO(val)
                obj.ICAO = val;
            else
                errID= 'fusion:adsb:InvalidString';
                throwAsCaller(MException(errID,message(errID,'ICAO', 6)))
            end
        end
        
        function set.Callsign(obj, val)
            validateCallsign(val);
            charval = char(val);
            nchars = numel(charval);
            obj.Callsign = [charval, repmat(' ',1,8-nchars)];
        end

        function obj = adsbTransponder( varargin)
            coder.extrinsic('adsbTransponder.isLoading');
            
            % Assign default GPS
            obj.GPS = gpsSensor('PositionInputFormat','Geodetic');
            
            % Logic to handle icao input
            if nargin
                firstArg = varargin{1};
                if isValidICAO(firstArg) % valid icao specified
                    propArgs = {'ICAO', firstArg, varargin{2:end}};
                elseif any(strcmp(firstArg,properties(obj))) % valid PV name
                    propArgs = varargin;
                else
                    error(message('fusion:adsb:InvalidFirstArg'));
                end
            else
                propArgs = varargin;
            end
            
            setProperties(obj,numel(propArgs),propArgs{:});
            
            % ICAO must be specified at construction
            cond = ~coder.const(adsbTransponder.isLoading) && ...
                ~coder.internal.is_defined(obj.ICAO);
            coder.internal.errorIf(cond, 'fusion:adsb:MustSpecifyICAO', 'ICAO');
        end
        
        function val = get.UpdateRate(obj)
            val = obj.GPS.SampleRate;
        end
        
        function set.UpdateRate(obj, val)
            obj.GPS.SampleRate = val;
        end
    end
    
    %% System Object Impls
    methods (Access = protected)
        
        function resetImpl(obj)
            % Reset GPS object
            reset(obj.GPS);
            obj.pTime = 0;
        end
        
        function releaseImpl(obj)
            release(obj.GPS);
        end
        
        function setupImpl(obj,varargin)
            %Verify GPS is set to 'Geodetic' before locking
            if strcmp(obj.GPS.PositionInputFormat,'Local')
                coder.internal.error('fusion:adsb:GPSPositionInputFormat');
            end
        end
        
        function message = stepImpl(obj, pos, vel)
            
            % Input validation
            validateattributes(pos, {'numeric'},{'nonempty','real','vector','numel',3,'nonnan','finite'},...
                mfilename,'position');
            validateattributes(vel, {'numeric'},{'nonempty','real','vector','numel',3,'nonnan','finite'},...
                mfilename,'velocity');
            
            % Internally always convert to rows
            position = pos(:)';
            velocity = vel(:)';
            
            % Get GPS measurements
            [lla, velned, ~, heading] = step(obj.GPS,position,velocity);
            [covpos, covvel] = gpsCovariance(obj);
            
            % Get message template
            message = fusion.internal.interfaces.DataStructures.adsbMessageStruct();
            
            % Setup message with Transponder information
            message.ICAO = obj.ICAO;
            message.Callsign = obj.Callsign;
            message.Category = obj.Category;
            message.Time = obj.pTime;
            
            % Add ADS-B Position data
            message.Latitude = lla(1);
            message.Longitude = lla(2);
            message.Altitude = lla(3);
            
            % Add ADS-B Velocity data
            message.Veast = velned(2);
            message.Vnorth = velned(1);
            message.ClimbRate = velned(3);
            message.Heading = heading;
            
            % Compute and add accuracy categories
            epu = fusion.internal.adsb.cov2radius(covpos(1:2,1:2));
            evu = fusion.internal.adsb.cov2radius(covvel(1:2,1:2));
            alterr = sqrt(covpos(3,3)); % 1 sigma error
            [NACp, NACv, GVA] = fusion.internal.adsb.uncertainty2nac(epu, evu, alterr);
            message.NACPosition = NACp(1);
            message.NACVelocity = NACv(1);
            message.GeometricVerticalAccuracy = GVA(1);
            
            % Update Internal Clock
            obj.pTime = obj.pTime + 1/obj.UpdateRate;
        end
        
        function s = saveObjectImpl(obj)
            s = saveObjectImpl@matlab.System(obj);
            s.GPS = matlab.System.saveObject(obj.GPS);
            if isLocked(obj)
                s.pTime = obj.pTime;
            end
        end
        
        function loadObjectImpl(obj,s,wasLocked)
            obj.GPS = matlab.System.loadObject(s.GPS);
            s = rmfield(s,'GPS');
            if wasLocked
                obj.pTime = s.pTime;
            end
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end
        
    end
    
    methods(Access = protected)
        
        function [covpos, covvel] = gpsCovariance(obj)
            % position
            horzstd = obj.GPS.HorizontalPositionAccuracy;
            vertstd = obj.GPS.VerticalPositionAccuracy;
            covpos = diag([horzstd horzstd vertstd].^2);
            %velocity
            covvel =  obj.GPS.VelocityAccuracy.^2 * eye(3);
        end
    end
    
    methods(Static, Hidden)
        function flag = isAllowedInSystemBlock
            flag = false;
        end

        function flag = isLoading()
            % Returns true if the object is currently being loaded
            
            flag = false;
            st = dbstack;
            for m = numel(st):-1:1
                flag = strcmp(st(m).name,'System.loadobj');
                if flag
                    break
                end
            end
        end
    end
    
    methods(Access = protected)
        function num = getNumInputsImpl(~)
            num = 2;
        end
    end
end

function cond = isValidICAO(icao)
cond =(ischar(icao) || isstring(icao)) && numel(char(icao)) == 6;
end

function validateCallsign(cs)
cl = class(cs);
switch cl
    case 'string'
        valid = isscalar(cs) && strlength(cs) <= 8;
    case 'char'
        valid = isvector(cs) && numel(cs) <= 8;
    otherwise
        valid = false;
end
coder.internal.errorIf(~valid, 'fusion:adsb:InvalidString', 'Callsign',8);
end
