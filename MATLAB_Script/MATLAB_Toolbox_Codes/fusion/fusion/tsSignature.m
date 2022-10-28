classdef tsSignature < fusion.internal.interfaces.BaseSignature
%TSSIGNATURE  Target strength (TS) signature for sonar
%  TSSIGNATURE defines the target strength (TS) used to model the
%  interaction of a platform with acoustic emitters, signals,
%  and sensors.
%
%   ts = TSSIGNATURE creates a default TS signature which models a
%   platform with a -30 dB cross-section at all view angles.
%
%   ts = TSSIGNATURE(..., 'Name', value) specifies additional
%   name-value pair arguments that define the properties described below:
%
%  TSSIGNATURE properties:
%    Pattern        - TS pattern.
%    Azimuth        - Azimuth view angles where TS pattern is sampled.
%    Elevation      - Elevation view angles where TS pattern is sampled.
%    Frequency      - Frequencies where TS pattern is sampled.
%
%  TSSIGNATURE methods:
%    value          - Retrieve the TS signature's value.
%    toStruct       - Convert object to a struct
% 
% See also: rcsSignature, sonarEmitter, sonarSensor.

%   Copyright 2018-2020 The MathWorks, Inc.

%#codegen
    
    % Define defaults for public properties. These are used by the PV pair
    % parser during construction.
    properties (Hidden, Constant)
        DefaultPattern = -30*ones(2,2,1)
        DefaultAzimuth = [-180 180]
        DefaultElevation = [-90 90]
        DefaultFrequency = [0 100e6]
    end
    
    methods
        function obj = tsSignature(varargin)
            obj@fusion.internal.interfaces.BaseSignature(varargin{:});
        end
    end
end
