classdef sonarSensor< fusion.internal.remotesensors.ScanningSensor & fusion.internal.remotesensors.mixin.DisplayUtils & radarfusion.internal.scenario.mixin.DetectionSensor & scenario.internal.mixin.Perturbable
%sonarSensor  Active or passive sonar detections generator
%   sensor = sonarSensor(sensorIndex) returns a statistical model to
%   generate detections from sonar emissions.
%     
%   sensor = sonarSensor(sensorIndex, scanConfig) configures the sonar
%   sensor to use a predefined scan configuration, scanConfig. scanConfig
%   can be one of 'No scanning' | 'Raster' | 'Rotator' | 'Sector'.
%
%   sensor = sonarSensor(..., 'Name', value) returns a sonarSensor object
%   by specifying its properties as name-value pair arguments. Unspecified
%   properties have default values. See the list of properties below.
%
%   Step method syntax:
%   
%   DETS = step(SENSOR, EMSONAR, TIME) generates passive detections from
%   the sonar emissions in the L-element array of structs, EMSONAR, at the
%   simulation time, TIME. TIME is a scalar value in seconds. EMSONAR is a
%   struct array representing the sonar emissions to use to generate
%   detections. The sonarSensor generates detections at the rate defined by
%   the UpdateRate property.
%
%   EMSONAR is the sonarEmission object. Its properties are listed <a href="matlab:help sonarEmission">here</a>.
%
%   The Measurement and MeasurementNoise properties of the returned
%   objectDetections, DETS, are reported in the coordinate system specified
%   by sonarSensor's DetectionCoordinates property as given in the table
%   below.
%
%       Detection           | Measurement       
%       Coordinates         | Dimensions        
%       --------------------+-------------------
%       Scenario            | [x;y;z;vx;vy;vz]
%       Body                | [x;y;z;vx;vy;vz]
%       Sensor rectangular  | [x;y;z;vx;vy;vz]
%       Sensor spherical    | [az;el;rg;rr]
%
%   When 'DetectionMode' is 'passive', detections are always returned in
%   the 'Sensor spherical' coordinate frame.
%
%   When detections are reported in rectangular coordinates, velocities are
%   only reported when the HasRangeRate property is true.
%
%   When detections are reported in spherical coordinates, elevation and
%   range rate are only reported when their enabling properties
%   HasElevation and HasRangeRate are set to true.
%
%   The MeasurementParameters property of the returned objectDetection
%   objects is set to an array of measurement parameter structures. The
%   fields are listed <a href="matlab:help fusion.internal.interfaces.DataStructures/measurementParametersStruct">here</a>.
%   When these transformations are applied in order, the reported
%   measurements will be transformed to the top-level frame. When HasINS is
%   true, the top-level frame is the scenario coordinate frame, otherwise
%   it is the coordinate frame of the platform on which the sensor is
%   mounted.
%
%   The ObjectAttributes property for each detection is set to a struct
%   with the following fields:
%       TargetIndex     Identifier of the platform from which the detected
%                       signal was last reflected. For false alarms, this
%                       value is negative.
%
%       EmitterIndex    Index of the emitter from which the detected signal
%                       was emitted.
%
%       SNR             Signal-to-noise ratio of the detection in decibels.
%
%   When 'DetectionMode' is set to 'passive', the following fields are also
%   reported on the ObjectAttributes struct:
%       CenterFrequency     Measured center frequency of the detected sonar
%                           signal in Hz.
%
%       Bandwidth           Measured bandwidth of the detected sonar signal
%                           in Hz.
%
%       WaveformType        Identifier of the waveform type that was
%                           classified by the passive sensor for the
%                           detected signal.
%
%
%   DETS = step(SENSOR, EMSONAR, TXCONFIGS, TIME) generates monostatic
%   detections from the sonar emissions. TXCONFIGS is a struct array
%   defining the configurations of the emitters at the current simulation
%   time. This array must contain the configuration of the sonarEmitter
%   whose EmitterIndex matches the value of the sonarSensor's EmitterIndex
%   property. The fields of the sonar emitter configuration structure are
%   listed <a href="matlab:help fusion.internal.interfaces.DataStructures/emitterConfigStruct">here</a>.
%
%   DETS = step(SENSOR, ..., INS, TIME) includes pose information for the
%   sensor's platform provided by the INS input with the reported
%   detections. This input is enabled when the HasINS property is set to
%   true. The INS information can be used by tracking and fusion algorithms
%   to estimate the target positions in the NED frame. INS is a struct with
%   the following fields:
%
%        Position      Position of the INS receiver in the local NED
%                      coordinate system specified as a real finite 1-by-3 
%                      array in meters.
% 
%        Velocity      Velocity of the INS receiver in the local NED 
%                      coordinate system specified as a real finite 1-by-3 
%                      array in meters per second.
%
%       Orientation    Orientation of the INS with respect to the local NED
%                      coordinate system specified as a scalar quaternion
%                      or a 3-by-3 real-valued orthonormal frame rotation
%                      matrix. Defines the frame rotation from the local
%                      NED coordinate system to the current INS body
%                      coordinate system. This is also referred to as a
%                      "parent to child" rotation.
%
%   [..., NUMDETS, CONFIG] = step(...) optionally returns the number of
%   valid detections in the cell array DETS and the configuration of the
%   sensor at the current simulation time, CONFIG. The fields of the sensor
%   configuration structure are listed <a href="matlab:help fusion.internal.interfaces.DataStructures/sensorConfigStruct">here</a>.
%
%   When the sonarSensor's MaxNumDetectionsSource property is set to
%   'Auto', NUMDETS is always set to the length of DETS. When
%   MaxNumDetectionsSource is set to 'Property', DETS is always a cell
%   array with length determined by the value of the MaxNumDetections
%   property. In this case, the first NUMDETS elements of DETS hold valid
%   detections and the remaining elements of DETS are set to a default
%   value.
%   
%   Detections can only be reported by sonarSensor at time intervals given
%   by the reciprocal of the UpdateRate property. The IsValidTime flag on
%   CONFIG is set to false when detection updates are requested at times
%   that are not aligned with the configured update rate.
%
%   sonarSensor properties:
%     SensorIndex               - Unique identifier of sensor system
%     UpdateRate                - Sensor update rate
%     DetectionMode             - Mode used to generate detections
%     EmitterIndex              - Unique identifier of monostatic emitter
%     MountingLocation          - Sensor's mounting location on platform
%     MountingAngles            - Sensor's mounting angles on platform
%     FieldOfView               - Angular field of view
%     ScanMode                  - Scan mode used by Sensor
%     MechanicalAngle           - Mechanical antenna angle (read-only)
%     ElectronicScanLimits      - Electronic beam scan limits
%     ElectronicAngle           - Electronic beam scan angle (read-only)
%     LookAngle                 - Look angle of sensor (read-only)
%     HasElevation              - Enable elevation scanning and measurement
%     CenterFrequency           - Center frequency of sensor
%     Bandwidth                 - Half power bandwidth of sensor
%     WaveformTypes             - Types of waveforms detected by sensor
%     ConfusionMatrix           - Probability of classifying a waveform
%     AmbientNoiseLevel         - Ambient noise (spectrum) level
%     FalseAlarmRate            - Rate at which false alarms are reported
%     AzimuthResolution         - Azimuthal resolution
%     ElevationResolution       - Elevation resolution
%     RangeResolution           - Range resolution
%     RangeRateResolution       - Range rate resolution
%     AzimuthBiasFraction       - Fractional azimuthal bias component
%     ElevationBiasFraction     - Fractional elevation bias component
%     RangeBiasFraction         - Fractional range bias component
%     RangeRateBiasFraction     - Fractional range rate bias component
%     HasRangeRate              - Enable range rate measurements
%     HasRangeAmbiguities       - Enable range ambiguities
%     HasRangeRateAmbiguities   - Enable range rate ambiguities
%     MaxUnambiguousRange       - Maximum unambiguous detection range
%     MaxUnambiguousRadialSpeed - Maximum unambiguous radial speed
%     HasINS                    - Enable input of platform's pose
%     HasNoise                  - Add noise to measurements
%     HasFalseAlarms            - Enable false detections
%     MaxNumDetectionsSource    - Source of maximum number of detections
%     MaxNumDetections          - Maximum number of reported detections
%     DetectionCoordinates      - Coordinates used to report detections
%
%   sonarSensor methods:
%     step            - Generate detections
%     perturbations   - Define perturbations to the sonarSensor
%     perturb         - Apply perturbations to the sonarSensor
%     release         - Allow property value and input characteristics changes
%     clone           - Create sonarSensor object with same property values
%     isLocked        - Locked status (logical)
%     <a href="matlab:help matlab.System/reset   ">reset</a>           - Reset states of sonarSensor object
%     <a href="matlab:help coverageConfig">coverageConfig</a>  - Report the sonarSensor object scanning coverage configuration
%     
%   % EXAMPLE: Detect a sonar emission using a passive sensor.
%
%   % Create a sonar emission.
%   orient = quaternion([180 0 0],'eulerd','zyx','frame');
%   sonarEmiss = sonarEmission('PlatformID', 1, 'EmitterIndex', 1, ...
%               'OriginPosition', [30 0 0], 'Orientation', orient, ...
%               'SourceLevel', 140, 'TargetStrength', 100);
%
%   % Create a passive sensor.
%   sensor = sonarSensor(1,'No scanning');
%
%   % Detect the sonar emission.
%   time = 0;
%   [dets, numDets, config] = sensor(sonarEmiss, time)
%
%   See also: sonarEmitter, underwaterChannel, sonarEmission, 
%   trackingScenario.

 
%   Copyright 2018-2021 The MathWorks, Inc.

    methods
        function out=sonarSensor
            % Support name-value pair arguments when constructing object
        end

        function out=addFalseAlarms(~) %#ok<STOUT>
        end

        function out=computeTruth(~) %#ok<STOUT>
        end

        function out=defaultObjectAttributes(~) %#ok<STOUT>
        end

        function out=defaultOutput(~) %#ok<STOUT>
            % Returns a representative "default" output generated by this
            % object
        end

        function out=defaultPerturbations(~) %#ok<STOUT>
        end

        function out=defaultTruthStruct(~) %#ok<STOUT>
        end

        function out=generateDetections(~) %#ok<STOUT>
            % If no targets lie inside the receiver's field of view, there
            % are no detections to generate
        end

        function out=getMeasurementLength(~) %#ok<STOUT>
            % Length of measurement property for objectDetection objects
            % returned by the sensor
        end

        function out=getNumInputsImpl(~) %#ok<STOUT>
            % Define total number of inputs for system with optional inputs
        end

        function out=getNumMeasDims(~) %#ok<STOUT>
        end

        function out=getPropertyGroups(~) %#ok<STOUT>
        end

        function out=getPropertyGroupsImpl(~) %#ok<STOUT>
            % Define property section(s) for System block dialog
        end

        function out=getPropertyGroupsLongImpl(~) %#ok<STOUT>
        end

        function out=integerProperties(~) %#ok<STOUT>
            %integerProperties Get the list of integer-valued properties
        end

        function out=isInactivePropertyImpl(~) %#ok<STOUT>
            % Return false if property is visible based on object 
            % configuration, for the command line and System block dialog
        end

        function out=isInputComplexityMutableImpl(~) %#ok<STOUT>
        end

        function out=isInputSizeMutableImpl(~) %#ok<STOUT>
        end

        function out=loadObjectImpl(~) %#ok<STOUT>
            % Set properties in object obj to values in structure s
        end

        function out=parseSensorInput(~) %#ok<STOUT>
        end

        function out=resetImpl(~) %#ok<STOUT>
        end

        function out=saveObjectImpl(~) %#ok<STOUT>
            % Set properties in structure s to values in object obj
        end

        function out=setupImpl(~) %#ok<STOUT>
            % Perform one-time calculations, such as computing constants
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
        %AmbientNoiseLevel  Ambient noise (spectrum) level
        %   This is the ambient isotropic noise spectrum level (in dB) 
        %   relative to the intensity of a plane wave with 1 micropascal
        %   rms pressure in a 1 hertz frequency band.  
        %
        %   Default: 70
        AmbientNoiseLevel;

        %AzimuthBiasFraction  Fractional azimuthal bias component
        %   Specify a nonnegative scalar defining the azimuthal bias
        %   component of the sensor as a fraction of the sensor's azimuthal
        %   resolution defined by the AzimuthResolution property value.
        %   This value sets a lower bound on the azimuthal accuracy of the
        %   sensor.
        %
        %   Default: 0.1
        AzimuthBiasFraction;

        %AzimuthResolution  Azimuthal resolution (deg)
        %   Specify a positive scalar defining the azimuthal resolution of
        %   the sensor. The sensor's azimuthal resolution defines the
        %   minimum separation in azimuth angle at which the sensor can
        %   distinguish two targets. This typically corresponds to the
        %   azimuthal 3 dB beamwidth of the sensor. Defined in degrees.
        %
        %   Default: 1
        AzimuthResolution;

        %Bandwidth  Half power bandwidth of sensor's operational band (Hz)
        %
        %   Default: 2000
        Bandwidth;

        %CenterFrequency  Center frequency of sensor's operational band (Hz)
        %
        %   Default: 20000
        CenterFrequency;

        %ConfusionMatrix Probability of correctly classifying a detected waveform
        %   An L-by-L matrix of values between 0 and 1 whose rows sum to 1,
        %   where L is the number of waveform types that can be detected by
        %   the sensor, as indicated by the value set in the WaveformTypes
        %   property. Each index (i,j) indicates the probability of
        %   classifying the waveform i as the waveform j.
        %
        %   When specified as a scalar between 0 and 1, the value is
        %   expanded along the diagonal of the confusion matrix.
        %
        %   When specified as a vector, it must have the same number of
        %   elements as the WaveformTypes property.
        %
        %   When defined as a scalar or a vector, the off diagonal values
        %   are set to (1-val)/(L-1).
        %
        %   Default: 1
        ConfusionMatrix;

        %DetectionCoordinates  Coordinate system used to report detections
        %   Specify coordinate system used to report detections as one of
        %   'Scenario' | 'Body' | 'Sensor rectangular' | 'Sensor
        %   spherical'. 'Scenario' can only be selected when the sensor's
        %   HasINS property is set to true. When set to 'Scenario',
        %   detections are reported in the scenario's coordinate frame.
        %   When set to 'Body', detections are reported in the body frame
        %   of the sensor's platform using the rectangular coordinate
        %   system. When set to 'Sensor rectangular', detections are
        %   reported using the sensor's coordinate frame using the
        %   rectangular coordinate system. When set to 'Sensor spherical',
        %   detections are reported using a spherical coordinate system
        %   centered at the sensor and aligned with the orientation of the
        %   sensor mounted on the platform. Measurements are ordered as
        %   [az, el, range, range rate]. Angles are in degrees, range in
        %   meters, and range rate in meters per second. Reporting of
        %   elevation and range rate depends on the corresponding
        %   HasElevation and HasRangeRate property values. When
        %   DetectionMode is set to 'passive', detections are only reported
        %   in the 'Sensor spherical' coordinate system.
        %
        %   Default: 'Body'
        DetectionCoordinates;

        %DetectionMode  Mode used to generate detections from sonar emissions
        %   Mode used to generate detections from received RF emissions
        %   specified as one of 'passive' | 'monostatic'. When set to 'passive',
        %   the sensor operates passively. When set to 'monostatic', the sensor 
        %   generates detections from reflected signals originating from a 
        %   collocated sonarEmitter.
        %
        %   Default: 'passive'
        DetectionMode;

        %ElevationBiasFraction  Fractional elevation bias component
        %   Specify a nonnegative scalar defining the elevation bias
        %   component of the sensor as a fraction of the sensor's elevation
        %   resolution defined by the ElevationResolution property value.
        %   This value sets a lower bound on the elevation accuracy of the
        %   sensor. This property only applies when you set HasElevation
        %   property to true.
        %
        %   Default: 0.1
        ElevationBiasFraction;

        %ElevationResolution  Elevation resolution (deg)
        %   Specify a positive scalar defining the elevation resolution of
        %   the sensor. The sensor's elevation resolution defines the
        %   minimum separation in elevation angle at which the sensor can
        %   distinguish two targets. This typically corresponds to the
        %   elevation 3 dB beamwidth of the sensor. This property only
        %   applies when you set HasElevation property to true. Defined in
        %   degrees.
        %
        %   Default: 1
        ElevationResolution;

        %EmitterIndex Unique identifier of monostatic emitter
        %   Used to identify the monostatic emitter providing the reference
        %   signal to the sensor. This property must be set when the
        %   'DetectionMode' property is set to 'monostatic'. There is no
        %   default value.
        EmitterIndex;

        %FalseAlarmRate  Rate at which false alarms are reported
        %   Specify a scalar value on the interval [1e-7 1e-3] defining the
        %   probability of reporting a false detection within each
        %   resolution cell of the sensor. Resolution cells are determined
        %   from the AzimuthResolution and when enabled the
        %   ElevationResolution properties.
        %
        %   Default: 1e-6
        FalseAlarmRate;

        %HasFalseAlarms  Enable false detections
        %   Set this property to true to include false alarms in the
        %   reported detections. Set this property to false to report only
        %   true detections.
        %
        %   Default: true
        HasFalseAlarms;

        %HasRangeAmbiguities  Enable range ambiguities
        %   Set to true to model a sensor which cannot resolve range
        %   ambiguities. When a sensor cannot resolve range ambiguities,
        %   targets at ranges beyond the MaxUnambiguousRange property value
        %   are wrapped into the interval of [0 MaxUnambiguousRange]. When
        %   false, targets are reported at their unwrapped range.
        %
        %   Default: false
        HasRangeAmbiguities;

        %HasRangeRate  Enable range rate measurements
        %   Set to true to model a sensor capable of estimating range rate
        %   from target detections. Set to false to model a sensor which
        %   cannot measure range rate.
        %
        %   Default: false
        HasRangeRate;

        %HasRangeRateAmbiguities  Enable range rate ambiguities
        %   Set to true to model a sensor which cannot resolve range rate
        %   ambiguities. When a sensor cannot resolve range rate
        %   ambiguities, targets at range rates outside of the interval
        %   MaxUnambiguousRadialSpeed are wrapped into the interval defined
        %   by the MaxUnambiguousRadialSpeed property. When false, targets
        %   at range rates outside of the MaxUnambiguousRadialSpeed
        %   property interval are not wrapped to into the range rate
        %   interval. This property only applies when you set HasRangeRate
        %   property to true.
        %
        %   Default: false
        HasRangeRateAmbiguities;

        %MaxUnambiguousRadialSpeed  Maximum unambiguous detection radial speed (m/s)
        %   Specify a scalar value greater than 0 defining the maximum
        %   radial speed at which the sensor can unambiguously resolve the
        %   range rate of a target. Targets detected at range rates whose
        %   magnitude is greater than the maximum unambiguous radial speed
        %   will be wrapped into the range rate interval
        %   [-MaxUnambiguousRadialSpeed MaxUnambiguousRadialSpeed]. This
        %   property only applies to true target detections when you set
        %   HasRangeRate property to true and set the
        %   HasRangeRateAmbiguities property to true.
        %
        %   This property also defines the range rate interval over which
        %   false target detections will be generated. This property only
        %   applies to false target detections when you set HasFalseAlarms
        %   property to true and HasRangeRate property to true.
        %   
        %   Specified in meters per second.
        %
        %   Default: 200
        MaxUnambiguousRadialSpeed;

        %MaxUnambiguousRange  Maximum unambiguous detection range (m)
        %   Specify a scalar value greater than 0 defining the maximum
        %   range at which the sensor can unambiguously resolve the range
        %   of a target. Targets detected at ranges beyond the unambiguous
        %   range will be wrapped into the range interval [0
        %   MaxUnambiguousRange]. This property only applies to true target
        %   detections when you set HasRangeAmbiguities property to true.
        %
        %   This property also defines the maximum range at which false
        %   alarms will be generated. This property only applies to false
        %   target detections when you set HasFalseAlarms property to
        %   true.
        %
        %   Specified in meters.
        %
        %   Default: 100e3
        MaxUnambiguousRange;

        %RangeBiasFraction  Fractional range bias component
        %   Specify a nonnegative scalar defining the range bias component
        %   of the sensor as a fraction of the sensor's range resolution
        %   defined by the RangeResolution property value. This value sets
        %   a lower bound on the range accuracy of the sensor.
        %
        %   Default: 0.05
        RangeBiasFraction;

        %RangeRateBiasFraction  Fractional range rate bias component
        %   Specify a nonnegative scalar defining the range rate bias
        %   component of the sensor as a fraction of the sensor's range
        %   rate resolution defined by the RangeRateResolution property
        %   value. This value sets a lower bound on the range rate accuracy
        %   of the sensor. This property only applies when you set
        %   HasRangeRate property to true.
        %
        %   Default: 0.05
        RangeRateBiasFraction;

        %RangeRateResolution  Range rate resolution (m/s)
        %   Specify a positive scalar defining the range rate resolution of
        %   the sensor. The sensor's range rate resolution defines the
        %   minimum separation in range rate at which the sensor can
        %   distinguish two targets. This property only applies when you
        %   set HasRangeRate property to true. Defined in meters per
        %   second.
        %
        %   Default: 10
        RangeRateResolution;

        %RangeResolution  Range resolution (m)
        %   Specify a positive scalar defining the range resolution of the
        %   sensor. The sensor's range resolution defines the minimum
        %   separation in range at which the sensor can distinguish two
        %   targets. Defined in meters.
        %
        %   Default: 100
        RangeResolution;

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

        %WaveformTypes Types of waveforms detected by receiver
        %   A vector of nonnegative integers identifying the types of
        %   waveforms detectable by this sensor.
        %
        %   Default: 0
        WaveformTypes;

        pHasRange;

        pHasRangeRate;

    end
end
