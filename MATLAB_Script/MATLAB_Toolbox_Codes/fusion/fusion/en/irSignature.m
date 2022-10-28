classdef irSignature< fusion.internal.interfaces.BaseSignature & fusion.internal.mixin.CustomDisplayMCOS
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

    methods
        function out=irSignature
        end

        function out=getPropertyGroups(~) %#ok<STOUT>
        end

        function out=value(~) %#ok<STOUT>
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
        end

    end
end
