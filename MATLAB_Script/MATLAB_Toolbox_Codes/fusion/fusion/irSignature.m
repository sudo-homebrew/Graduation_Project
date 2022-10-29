classdef irSignature < fusion.internal.interfaces.BaseSignature & ...
        fusion.internal.mixin.CustomDisplayMCOS

%IRSIGNATURE  Infrared (IR) platform signature
%   irsig = IRSIGNATURE creates an infrared (IR) signature which models the
%   contrast radiant intensity of a platform with respect to the
%   background. The default IR signature returned models a platform with a
%   50 dBw/sr signature at all view angles.
%
%   irsig = IRSIGNATURE(..., 'Name', value) specifies additional
%   name-value pair arguments that define the properties described below:
%
%   IRSIGNATURE properties:
%    Pattern        - Contrast radiant intensity IR pattern in dbW/sr.
%    Azimuth        - Azimuth view angles where IR pattern is sampled.
%    Elevation      - Elevation view angles where IR pattern is sampled.
%    Frequency      - Frequencies where IR pattern is sampled.
%
%   IRSIGNATURE methods:
%    value          - Retrieve the IR signature's value.
%    toStruct       - Convert object to a struct
%
%   EXAMPLE: Create an azimuth dependent IR signature.
% 
%   % Define azimuth samples.
%   az = -180:180;
%
%   % Define elevation span where pattern is valid.
%   el = [-5 5];
%
%   % Create an IR signature pattern.
%   pat = 50*cosd(az).^2;
%   irSig = irSignature('Pattern', pat, 'Azimuth', az, 'Elevation', el);
%
%   % Plot pattern.
%   plot(irSig.Azimuth, irSig.Pattern)
%   xlabel('Azimuth (deg)'); ylabel('RCS (dBW/sr)');
%   title('Infrared Signature Pattern')
%
%   % Get IR value at 0 degrees azimuth and 0 degrees elevation.
%   value(irSig, 0, 0)
%
%   % Get IR value outside of valid elevation span.
%   value(irSig, 0, 6)
% 
% See also: irSensor, rcsSignature.

%   Copyright 2018 The MathWorks, Inc.

%#codegen
    
    % Define defaults for public properties. These are used by the PV pair
    % parser during construction.
    properties (Hidden, Constant)
        DefaultPattern = 50*ones(2,2,1)
        DefaultAzimuth = [-180 180]
        DefaultElevation = [-90 90]
        DefaultFrequency = [0 1e20]
    end
    
    methods
        function obj = irSignature(varargin)
            obj@fusion.internal.interfaces.BaseSignature(varargin{:});
        end
    end
    
    % --------------
    % Implementation
    % --------------
    methods
        function vals = value(obj, azsIn, elsIn)
            %VALUE  Interpolate the signature's pattern value
            %  vals = VALUE(obj, azs, els) returns the signature's pattern
            %  value by linearly interpolating the signature, obj, at the
            %  specified azimuth, azs, and elevation, els, view angles. azs
            %  and els are in degrees and are defined in the body frame of
            %  the pattern.
            %
            %  If all but one of the inputs is scalar, the other input will
            %  be expanded to the same size as the non-scalar input.
            %  Otherwise, azs and els must have the same size.
            
            % Scalar expand
            if isscalar(azsIn) && ~isscalar(elsIn)
                % Elevation non-scalar
                els = elsIn;
                sz = size(els);
                
                azs = scalarExpand(azsIn,sz);
            elseif ~isscalar(azsIn) && isscalar(elsIn)
                % Azimuth non-scalar
                azs = azsIn;
                sz = size(azs);
                
                els = scalarExpand(elsIn,sz);
            elseif numel(azsIn)~=numel(elsIn)
                coder.internal.error('shared_radarfusion:BaseSignature:inputMismatch');
            else
                azs = azsIn;
                els = elsIn;
            end
            
            freqsIn = mean(obj.Frequency);
            if isscalar(els)
                freqs = freqsIn;
            else
                freqs = freqsIn*ones(size(els));
            end
            vals = value@fusion.internal.interfaces.BaseSignature(obj, azs, els, freqs);
        end
    end
    
    % --------------
    % Custom display
    % --------------
    methods (Access = protected)
        function groups = getPropertyGroups(obj)
            if ~isscalar(obj)
                groups = getPropertyGroups@matlab.mixin.CustomDisplay(obj);
            else
                propList = struct('Pattern',obj.Pattern,...
                    'Azimuth',obj.Azimuth,...
                    'Elevation',obj.Elevation,...
                    'Frequency', obj.Frequency);
                groups = matlab.mixin.util.PropertyGroup(propList);
            end
        end
    end
    
    % ----------
    % Validation
    % ----------
    methods(Static, Access = {?fusion.internal.interfaces.BaseSignature})
        function checkPattern(val)
            validateattributes(val, {'double','single'}, ...
                {'2d','real','finite'}, ...
                '', 'Pattern');
        end
    end
end


function valOut = scalarExpand(valIn,sz)
valOut = valIn(1)*ones(sz,'like',valIn);
end
