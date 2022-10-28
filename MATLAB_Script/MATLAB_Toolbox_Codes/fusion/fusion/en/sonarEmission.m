classdef sonarEmission< fusion.internal.interfaces.BaseEmission & radarfusion.internal.scenario.mixin.Emission
%SONAREMISSION  Report for a single acoustic signal emission
%   sonarSignal = SONAREMISSION creates a sonarSignal which represents the 
%   acoustic signals from emitters, channels, and sensors.
%
%   sonarSignal = sonarEmission(..., 'Name', value) specifies additional
%   name-value pair arguments that define the properties described below:
%
%   SONAREMISSION properties:
%    PlatformID             - Unique identifier of the platform from which the 
%                             emitted signal originated.
%    EmitterIndex           - Unique identifier of the emitter from which the 
%                             emitted signal originated.
%    OriginPosition         - Location of the emitter in scenario coordinates.
%    OriginVelocity         - Velocity of the emitter in scenario coordinates.
%    Orientation            - Orientation of the emitter in scenario coordinates.
%    FieldOfView            - Field of view of the emitter.
%    SourceLevel            - Source level in dB relative to the intensity of a
%                             sound wave having an rms pressure of 1 micro-pascal.  
%    TargetStrength         - Cumulative target strength of the signal
%                             in dB
%    CenterFrequency        - Center frequency of the signal in Hz.
%    Bandwidth              - Half power bandwidth of the signal in Hz.
%    WaveformType           - Identifier of the signal's waveform type.
%    ProcessingGain         - Processing gain associated with the signal's
%                             waveform in dB.
%    PropagationRange       - Total distance over which the signal has propagated
%                             in meters.
%    PropagationRangeRate   - Total range rate for the path over which the signal
%                             has propagated in meters per second.
%
% See also: radarEmission, sonarEmitter.

 
%   Copyright 2018 The MathWorks, Inc.

    methods
        function out=sonarEmission
        end

    end
    properties
        SourceLevel;

        % TargetStrength Sonar target strength
        %   Cumulative target strength of the source platform emitting the signal in dB.
        %
        %   Default: 0
        TargetStrength;

    end
end
