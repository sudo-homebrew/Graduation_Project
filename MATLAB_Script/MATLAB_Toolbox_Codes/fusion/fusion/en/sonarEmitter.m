classdef sonarEmitter< fusion.internal.remotesensors.ScanningEmitter & fusion.internal.remotesensors.mixin.DisplayUtils & radarfusion.internal.scenario.mixin.Emitter & scenario.internal.mixin.Perturbable
%sonarEmitter  Acoustic signals and interferences generator
%   emitter = sonarEmitter(emitterIndex) returns an acoustic frequency
%   emitter that simulates sonar emissions.
%     
%   emitter = sonarEmitter(emitterIndex, scanConfig) configures the sonar
%   emitter to use a predefined scan configuration, scanConfig. scanConfig
%   can be one of 'No scanning' | 'Raster' | 'Rotator' | 'Sector'.
%
%   emitter = sonarEmitter(..., 'Name', value) returns an sonarEmitter
%   object by specifying its properties as name-value pair arguments.
%   Unspecified properties have default values. See the list of properties
%   below.
%
%   Step method syntax:
%   
%   EMSONAR = step(EMITTER, PLAT, TIME) simulates a signal emitted from the
%   platform the emitter is mounted on, PLAT, at the simulation time, TIME.
%   PLAT is either a scalar struct of the current platform's pose or a
%   platform object as returned from the platform method on
%   trackingScenario or attached to the scenario's Platform property. TIME
%   is a scalar value in seconds. EMSONAR is a scalar struct representing
%   the sonar emission. The sonar emission updates at the rate defined by
%   the UpdateRate property.
%
%   A platform structure is a structure which can have the fields listed 
%   <a href="matlab:help fusion.internal.interfaces.DataStructures/platformStruct">here</a>.
%
%   EMSONAR is a sonarEmission object. Its properties are listed <a href="matlab:help sonarEmission">here</a>.
%
%   [EMSONAR, CONFIG] = step(EMITTER, PLAT, TIME) optionally returns the
%   current configuration of the emitter as an emitter configuration
%   struct. Its fields are listed <a href="matlab:help fusion.internal.interfaces.DataStructures/emitterConfigStruct">here</a>.
%
%   sonarEmitter properties:
%     EmitterIndex              - Unique identifier of emitter system
%     UpdateRate                - Emitter update rate
%     MountingLocation          - Emitter's mounting location on platform
%     MountingAngles            - Emitter's mounting angles on platform
%     FieldOfView               - Angular field of view
%     ScanMode                  - Scan mode used by emitter
%     ElectronicScanLimits      - Electronic beam scan limits
%     ElectronicAngle           - Electronic beam scan angle (read-only)
%     LookAngle                 - Look angle of emitter (read-only)
%     HasElevation              - Enable elevation scanning
%     SourceLevel               - Sonar source level
%     CenterFrequency           - Center frequency of emitter
%     Bandwidth                 - Half power bandwidth of emitter
%     WaveformType              - Type of waveform emitted
%     ProcessingGain            - Processing gain from emitted waveform
%
%   sonarEmitter methods:
%     step            - Emit sonar emission
%     perturbations   - Define perturbations to the sonarEmitter
%     perturb         - Apply perturbations to the sonarEmitter
%     release         - Allow property value and input characteristics changes
%     clone           - Create sonarEmitter object with same property values
%     isLocked        - Locked status (logical)
%     <a href="matlab:help matlab.System/reset   ">reset</a>           - Reset states of sonarEmitter object
%     <a href="matlab:help coverageConfig">coverageConfig</a>  - Report the sonarEmitter object scanning coverage configuration
%
%   % EXAMPLE: Model a sonar emitter.
%
%   % Create a platform to mount the emitter on.
%   plat = struct( ...
%       'PlatformID', 1, ...
%       'Position', [0 0 0]);
% 
%   % create the emitter
%   emitter = sonarEmitter(1, 'No scanning');
%
%   % Emit the waveform.
%   time = 0;
%   emTx = emitter(plat, time)
%
%   See also: sonarSensor, underwaterChannel, sonarEmission,
%   trackingScenario.

 
%   Copyright 2018-2021 The MathWorks, Inc.

    methods
        function out=sonarEmitter
            % Support name-value pair arguments when constructing object
        end

        function out=defaultOutput(~) %#ok<STOUT>
        end

        function out=defaultPerturbations(~) %#ok<STOUT>
        end

        function out=getPropertyGroups(~) %#ok<STOUT>
        end

        function out=getPropertyGroupsImpl(~) %#ok<STOUT>
            % Define property section(s) for System block dialog
        end

        function out=integerProperties(~) %#ok<STOUT>
            %integerProperties Get the list of integer-valued properties
        end

        function out=isInactivePropertyImpl(~) %#ok<STOUT>
        end

        function out=isInputComplexityMutableImpl(~) %#ok<STOUT>
        end

        function out=isInputSizeMutableImpl(~) %#ok<STOUT>
        end

        function out=loadObjectImpl(~) %#ok<STOUT>
            % Set properties in object obj to values in structure s
        end

        function out=resetImpl(~) %#ok<STOUT>
        end

        function out=saveObjectImpl(~) %#ok<STOUT>
            % Set properties in structure s to values in object obj
        end

        function out=setupImpl(~) %#ok<STOUT>
        end

        function out=stepImpl(~) %#ok<STOUT>
        end

        function out=validateInputsImpl(~) %#ok<STOUT>
            % Validate inputs to the step method at initialization (or
            % whenever an input size changes)
        end

        function out=validatePropertiesImpl(~) %#ok<STOUT>
            % Validate related or interdependent property values
        end

    end
    properties
        %Bandwidth Half power bandwidth of emitter
        %   Specify the bandwidth (in Hz) of the operational band for the
        %   transmitter as a scalar.
        %
        %   Default: 2000
        Bandwidth;

        %CenterFrequency Center frequency of emitter
        %   Specify the center frequency (in Hz) of the operational band
        %   for the transmitter as a scalar.
        %
        %   Default: 20000
        CenterFrequency;

        %ProcessingGain Processing gain from emitted waveform
        %   Processing gain achieved (in dB) when demodulating the emitted
        %   signal's waveform. Processing gain is achieved by emitting a
        %   signal over a bandwidth which is greater than the minimum
        %   bandwidth necessary to send the information contained in the
        %   signal.
        %
        %   Default: 0
        ProcessingGain;

        %ScanMode  Scan mode used by sonar
        %   Specify the scan mode used by the sonar as one of 'No scanning'
        %   or 'Electronic'.  When set to 'No scanning', no scanning is 
        %   performed by the sonar. The sonar beam is along the antenna's
        %   boresight. When set to 'Electronic', the sonar scans electronically
        %   across the azimuth and elevation limits specified by the 
        %   ElectronicScanLimits property; the scan positions step
        %   by the sonar's field of view between dwells.
        %
        %   Default: 'Electronic'
        ScanMode;

        %SourceLevel Sonar source level
        %   Specify the sonar source level of the transmitter in decibels
        %   relative to the intensity of a sound wave having an rms
        %   pressure of 1 micropascal.  Units are in dB / 1 micropascal.
        %
        %   Default: 140
        SourceLevel;

        %WaveformType Type of waveform
        %   A user-defined nonnegative integer used to classify the type of
        %   waveform transmitted.
        %
        %   Default: 0
        WaveformType;

    end
end
