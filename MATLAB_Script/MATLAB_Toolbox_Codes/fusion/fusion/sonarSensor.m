classdef(Sealed, StrictDefaults) sonarSensor < fusion.internal.remotesensors.ScanningSensor & ...
        fusion.internal.remotesensors.mixin.DisplayUtils & ...
        radarfusion.internal.scenario.mixin.DetectionSensor & ...
        scenario.internal.mixin.Perturbable
    
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
%     reset           - Reset states of sonarSensor object
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

%#codegen

    properties(Nontunable)
        %DetectionMode  Mode used to generate detections from sonar emissions
        %   Mode used to generate detections from received RF emissions
        %   specified as one of 'passive' | 'monostatic'. When set to 'passive',
        %   the sensor operates passively. When set to 'monostatic', the sensor 
        %   generates detections from reflected signals originating from a 
        %   collocated sonarEmitter.
        %
        %   Default: 'passive'
        DetectionMode = 'passive'
    end
    properties(Constant, Hidden)
        DetectionModeSet = matlab.system.StringSet({'passive','monostatic'});
    end
    
    properties(Nontunable)
        %EmitterIndex Unique identifier of monostatic emitter
        %   Used to identify the monostatic emitter providing the reference
        %   signal to the sensor. This property must be set when the
        %   'DetectionMode' property is set to 'monostatic'. There is no
        %   default value.
        EmitterIndex {mustBeScalarOrEmpty, mustBePositive, mustBeInteger}
    end
    
    properties(Nontunable)
        %CenterFrequency  Center frequency of sensor's operational band (Hz)
        %
        %   Default: 20000
        CenterFrequency = 20000
        
        %Bandwidth  Half power bandwidth of sensor's operational band (Hz)
        %
        %   Default: 2000
        Bandwidth = 2000;
        
        %WaveformTypes Types of waveforms detected by receiver
        %   A vector of nonnegative integers identifying the types of
        %   waveforms detectable by this sensor.
        %
        %   Default: 0
        WaveformTypes = 0
        
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
        ConfusionMatrix = 1
        
        %AmbientNoiseLevel  Ambient noise (spectrum) level
        %   This is the ambient isotropic noise spectrum level (in dB) 
        %   relative to the intensity of a plane wave with 1 micropascal
        %   rms pressure in a 1 hertz frequency band.  
        %
        %   Default: 70
        AmbientNoiseLevel = 70

        %FalseAlarmRate  Rate at which false alarms are reported
        %   Specify a scalar value on the interval [1e-7 1e-3] defining the
        %   probability of reporting a false detection within each
        %   resolution cell of the sensor. Resolution cells are determined
        %   from the AzimuthResolution and when enabled the
        %   ElevationResolution properties.
        %
        %   Default: 1e-6
        FalseAlarmRate = 1e-6
    end
    
    % Resolution properties
    properties(Nontunable)
        %AzimuthResolution  Azimuthal resolution (deg)
        %   Specify a positive scalar defining the azimuthal resolution of
        %   the sensor. The sensor's azimuthal resolution defines the
        %   minimum separation in azimuth angle at which the sensor can
        %   distinguish two targets. This typically corresponds to the
        %   azimuthal 3 dB beamwidth of the sensor. Defined in degrees.
        %
        %   Default: 1
        AzimuthResolution = 1
    end

    properties(Dependent,Nontunable)
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
        ElevationResolution
    end

    properties(Nontunable)
        %RangeResolution  Range resolution (m)
        %   Specify a positive scalar defining the range resolution of the
        %   sensor. The sensor's range resolution defines the minimum
        %   separation in range at which the sensor can distinguish two
        %   targets. Defined in meters.
        %
        %   Default: 100
        RangeResolution = 100

        %RangeRateResolution  Range rate resolution (m/s)
        %   Specify a positive scalar defining the range rate resolution of
        %   the sensor. The sensor's range rate resolution defines the
        %   minimum separation in range rate at which the sensor can
        %   distinguish two targets. This property only applies when you
        %   set HasRangeRate property to true. Defined in meters per
        %   second.
        %
        %   Default: 10
        RangeRateResolution = 10
    end
    
    % Bias properties
    properties(Nontunable)
        %AzimuthBiasFraction  Fractional azimuthal bias component
        %   Specify a nonnegative scalar defining the azimuthal bias
        %   component of the sensor as a fraction of the sensor's azimuthal
        %   resolution defined by the AzimuthResolution property value.
        %   This value sets a lower bound on the azimuthal accuracy of the
        %   sensor.
        %
        %   Default: 0.1
        AzimuthBiasFraction = 0.1
        
        %ElevationBiasFraction  Fractional elevation bias component
        %   Specify a nonnegative scalar defining the elevation bias
        %   component of the sensor as a fraction of the sensor's elevation
        %   resolution defined by the ElevationResolution property value.
        %   This value sets a lower bound on the elevation accuracy of the
        %   sensor. This property only applies when you set HasElevation
        %   property to true.
        %
        %   Default: 0.1
        ElevationBiasFraction = 0.1
        
        %RangeBiasFraction  Fractional range bias component
        %   Specify a nonnegative scalar defining the range bias component
        %   of the sensor as a fraction of the sensor's range resolution
        %   defined by the RangeResolution property value. This value sets
        %   a lower bound on the range accuracy of the sensor.
        %
        %   Default: 0.05
        RangeBiasFraction = 0.05

        %RangeRateBiasFraction  Fractional range rate bias component
        %   Specify a nonnegative scalar defining the range rate bias
        %   component of the sensor as a fraction of the sensor's range
        %   rate resolution defined by the RangeRateResolution property
        %   value. This value sets a lower bound on the range rate accuracy
        %   of the sensor. This property only applies when you set
        %   HasRangeRate property to true.
        %
        %   Default: 0.05
        RangeRateBiasFraction = 0.05
    end
    
    % "Has" properties
    properties(Nontunable)
        %HasRangeRate  Enable range rate measurements
        %   Set to true to model a sensor capable of estimating range rate
        %   from target detections. Set to false to model a sensor which
        %   cannot measure range rate.
        %
        %   Default: false
        HasRangeRate (1, 1) logical = false
        
        %HasRangeAmbiguities  Enable range ambiguities
        %   Set to true to model a sensor which cannot resolve range
        %   ambiguities. When a sensor cannot resolve range ambiguities,
        %   targets at ranges beyond the MaxUnambiguousRange property value
        %   are wrapped into the interval of [0 MaxUnambiguousRange]. When
        %   false, targets are reported at their unwrapped range.
        %
        %   Default: false
        HasRangeAmbiguities (1, 1) logical = false
        
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
        HasRangeRateAmbiguities (1, 1) logical = false
    end
    
    properties(Nontunable)
        %HasFalseAlarms  Enable false detections
        %   Set this property to true to include false alarms in the
        %   reported detections. Set this property to false to report only
        %   true detections.
        %
        %   Default: true
        HasFalseAlarms (1, 1) logical = true
    end
    
    % Limit properties
    properties(Nontunable)
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
        MaxUnambiguousRange = 100e3

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
        MaxUnambiguousRadialSpeed = 200
    end
    
    properties(Nontunable)
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
        ScanMode = 'Electronic'
    end
    
    properties(Constant, Hidden)
        ScanModeSet = matlab.system.StringSet({'No scanning','Electronic'});
    end
    
    properties(Nontunable)
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
        DetectionCoordinates = 'Body'
    end
    properties(Constant, Hidden)
        DetectionCoordinatesSet = matlab.system.StringSet({'Scenario','Body','Sensor rectangular','Sensor spherical'});
    end
    
    properties(Nontunable, Access = private)
        pConfusionMatrix
    end
    properties(Access = private)
        pElevationResolution = 1
    end
    
    properties(Nontunable, Access = protected)
        pHasRange
        pHasRangeRate
    end
    
    % -------------------
    % Setters and getters
    % -------------------
    methods
        function val = get.DetectionCoordinates(obj)
            if strcmpi(obj.DetectionMode(1),'p')
                % DetectionCoordinates is not active or displayed in this
                % mode, but it should report the coordinate frame used by
                % the sensor, since the user may still query it.
                val = 'Sensor spherical';
            else
                val = obj.DetectionCoordinates;
            end
        end
        
        function set.EmitterIndex(obj, val)
            if ~isempty(val)
                obj.EmitterIndex = val;
            end
        end
        
        function set.CenterFrequency(obj, val)
            obj.checkScalarRealPositiveFinite(val, mfilename, 'CenterFrequency');
            obj.CenterFrequency = val;
        end
        
        function set.Bandwidth(obj,val)
            obj.checkScalarRealPositiveFinite(val, mfilename, 'Bandwidth');
            obj.Bandwidth = val;
        end
        
        function set.WaveformTypes(obj,val)
            obj.checkVectorNonnegativeIndex(val, mfilename, 'WaveformTypes');
            obj.WaveformTypes = val;
        end
        
        function set.AmbientNoiseLevel(obj,val)
            obj.checkScalarRealFinite(val, mfilename, 'AmbientNoiseLevel');
            obj.AmbientNoiseLevel = val;
        end
                
        function set.ConfusionMatrix(obj,val)
            validateattributes(val,{'double'},{'2d','>=',0,'<=',1},mfilename,'ConfusionMatrix');
            obj.ConfusionMatrix = val;
        end
                
        function set.FalseAlarmRate(obj,val)
            validateattributes(val,{'double'},{'scalar','real','>=',1e-7,'<=',1e-3},mfilename,'FalseAlarmRate');
            obj.FalseAlarmRate = val;
        end
        
        function set.AzimuthResolution(obj,val)
            obj.checkScalarRealPositiveFinite(val, mfilename, 'AzimuthResolution');
            obj.AzimuthResolution = val;
        end
        
        function set.ElevationResolution(obj, val)
            obj.checkScalarRealPositiveFinite(val, mfilename, 'ElevationResolution');
            obj.pElevationResolution = val;
        end
        function val = get.ElevationResolution(obj)
            val = obj.pElevationResolution;
        end
        
        function set.RangeResolution(obj, val)
            obj.checkScalarRealPositiveFinite(val, mfilename, 'RangeResolution');
            obj.RangeResolution = val;
        end
        
        function set.RangeRateResolution(obj, val)
            obj.checkScalarRealPositiveFinite(val, mfilename, 'RangeRateResolution');
            obj.RangeRateResolution = val;
        end
        
        function set.AzimuthBiasFraction(obj,val)
            validateattributes(val,{'double'},{'scalar','real','>=',0,'<=',1},mfilename,'AzimuthBiasFraction');
            obj.AzimuthBiasFraction = val;
        end
        
        function set.ElevationBiasFraction(obj,val)
            validateattributes(val,{'double'},{'scalar','real','>=',0,'<=',1},mfilename,'ElevationBiasFraction');
            obj.ElevationBiasFraction = val;
        end
        
        function set.RangeBiasFraction(obj,val)
            validateattributes(val,{'double'},{'scalar','real','>=',0,'<=',1},mfilename,'RangeBiasFraction');
            obj.RangeBiasFraction = val;
        end
        
        function set.RangeRateBiasFraction(obj,val)
            validateattributes(val,{'double'},{'scalar','real','>=',0,'<=',1},mfilename,'RangeRateBiasFraction');
            obj.RangeRateBiasFraction = val;
        end
        
        function set.MaxUnambiguousRange(obj,val)
            obj.checkScalarRealPositiveFinite(val, mfilename, 'MaxUnambiguousRange');
            obj.MaxUnambiguousRange = val;
        end
        
        function set.MaxUnambiguousRadialSpeed(obj,val)
            obj.checkScalarRealPositiveFinite(val, mfilename, 'MaxUnambiguousRadialSpeed');
            obj.MaxUnambiguousRadialSpeed = val;
        end
    end

    % -----------
    % Constructor
    % -----------
    methods
        function obj = sonarSensor(varargin)
            % Support name-value pair arguments when constructing object
            obj@fusion.internal.remotesensors.ScanningSensor(varargin{:});
            obj@radarfusion.internal.scenario.mixin.DetectionSensor('sonarEmission');
        end
    end
    
    % --------------
    % Implementation
    % --------------
    methods(Hidden)
        function flag = isDetectionGenerator(~)
            % Returns true if device returns detections
            flag = true;
        end
        
        function flag = requiresSignals(~)
            % Returns true if the device requires a signal input
            flag = true;
        end
        
        function flag = requiresConfig(obj)
            % Returns true if the device requires a config input
            flag = strcmpi(obj.DetectionMode, 'monostatic');
        end
    end
    methods(Access = protected)
        function setupImpl(obj, varargin)
            % Perform one-time calculations, such as computing constants
            setupImpl@fusion.internal.remotesensors.ScanningSensor(obj, varargin{:});
            
            if strcmpi(obj.DetectionMode,'passive')
                obj.pHasRange = false;
                obj.pHasRangeRate = false;
                obj.pDetectionCoordinates = 'Sensor spherical';
                
                % Scalar expand confusion matrix
                val = obj.ConfusionMatrix;
                numWfms = numel(obj.WaveformTypes);
                if numWfms>1 && (isscalar(val) || isvector(val))
                    if isscalar(val)
                        mtxD = val*eye(numWfms);
                        mtxND = (1-val)/(numWfms-1)*(ones(numWfms)-eye(numWfms));
                    else % it's a vector
                        mtxD = diag(val);
                        mtxND = diag(1-val)/(numWfms-1).*(ones(numWfms)-eye(numWfms));
                    end
                    obj.pConfusionMatrix = mtxD+mtxND;
                else
                    obj.pConfusionMatrix = val;
                end
            else
                obj.pHasRange = true;
                obj.pHasRangeRate = obj.HasRangeRate;
                obj.pDetectionCoordinates = obj.DetectionCoordinates;
            end
            obj.pCanMeasureRange = obj.pHasRange;
            obj.pCanMeasureRangeRate = obj.pHasRangeRate;
        end
        
        function resetImpl(obj)
            resetImpl@fusion.internal.remotesensors.ScanningSensor(obj);
        end

        function [detections, numDets, config] = stepImpl(obj, varargin)
            [signals, refSignal, refConfig, ins, time] = parseSensorInput(obj, varargin{:});
            
            [detections, numDets] = initializeDetections(obj);
            
            % Update sensor's scan position
            if strcmpi(obj.DetectionMode, 'monostatic')
                isValidTime = refConfig(1).IsValidTime;
            else
                stepImpl@fusion.internal.remotesensors.ScanningSensor(obj, time);
                isValidTime = isValidUpdateTime(obj, time);
            end
            
            if isValidTime
                
                signals = receivedSignals(obj,signals,refSignal);

                % Transform signals into the sensor's frame
                signals = bodyToSensor(obj,signals,refConfig);
                
                % Compute detection truth from signals
                truth = computeTruth(obj,signals,refSignal);
                
                [meas,covs,snrdB,tgtIDs,sysIDs,wfmIDs] = generateDetections(obj,truth);
                
                % Add false alarms
                [measFA, covsFA, snrdBFA, tgtIDsFA, sysIDsFA, wfmIDsFA] = addFalseAlarms(obj, meas, covs, snrdB, tgtIDs, sysIDs, wfmIDs);
                
                % Number of detections that will be reported by the sensor
                numDets = length(tgtIDsFA);
                if strcmpi(obj.MaxNumDetectionsSource,'Property')
                    numDets = min(numDets,obj.MaxNumDetections);
                end
                
                % If detections are generated, then transform into the correct
                % coordinate frame and assemble into objectDetection objects
                if numDets>0
                    % If the output size is variable (auto), then allocate
                    % space for output, otherwise, it has already been
                    % allocated in the call above to |initializeDetections|
                    if strcmpi(obj.MaxNumDetectionsSource,'Auto')
                        detections = repmat(defaultOutput(obj),numDets,1);
                    end
                    
                    % Limit number of reported detections to maximum number
                    % that can be reported by the sensor
                    measFA = measFA(:,1:numDets);
                    covsFA = covsFA(:,:,1:numDets);
                    tgtIDsFA = tgtIDsFA(1:numDets);
                    sysIDsFA = sysIDsFA(1:numDets);
                    wfmIDsFA = wfmIDsFA(1:numDets);
                    snrdBFA = snrdBFA(1:numDets);
                    
                    % Convert to requested coordinate frame for reported
                    % detections
                    [measCoords, covsCoords] = convertToRequestedCoords(obj,measFA,covsFA,refConfig,ins);
                    
                    % Assemble detections into objectDetection objects
                    if strcmpi(obj.DetectionMode,'passive')
                        attribs = {'TargetIndex',tgtIDsFA,'EmitterIndex',sysIDsFA,'WaveformType',wfmIDsFA,'SNR',snrdBFA};
                    else
                        attribs = {'TargetIndex',tgtIDsFA,'EmitterIndex',sysIDsFA,'SNR',snrdBFA};
                    end
                    detsAssigned = assembleDetections(obj,ins,time,measCoords,covsCoords,obj.HasElevation,obj.pHasRange,obj.pHasRangeRate,attribs,refConfig);
                    for m = 1:numDets
                        detections{m} = detsAssigned{m};
                    end
                    detections = reshape(detections,[],1);
                end
                
                % Latch current step time
                obj.pHasFirstUpdate = true;
                obj.pTimeLastUpdate = time;
            end
            
            config = assembleConfig(obj, refConfig, isValidTime, ins);
        end
        
        function [measFA, covsFA, snrdBFA, tgtIDsFA, sysIDsFA, wfmIDsFA] = addFalseAlarms(obj, meas, covs, snrdB, tgtIDs, sysIDs, wfmIDs)
            [measFA, covsFA, snrdBFA, tgtIDsFA] = addFalseAlarms@fusion.internal.remotesensors.ScanningSensor(obj, meas, covs, snrdB, tgtIDs);

            isFA = tgtIDsFA<0;
            numFA = sum(isFA);
            
            sysIDsFA = NaN(size(tgtIDsFA));
            sysIDsFA(~isFA) = sysIDs;
            sysIDsFA(isFA) = -1;
            
            wfmIDsFA = NaN(size(tgtIDsFA));
            if strcmpi(obj.DetectionMode,'passive')
                wfmIDsFA(~isFA) = wfmIDs;
                
                % Apply confusion matrix to false alarms
                numWfms = size(obj.ConfusionMatrix,1);
                allWfms = obj.WaveformTypes;
                iWfmsFA = find(isFA);
                for m = 1:numFA
                    iRow = floor(numWfms*rand(obj))+1;
                    f = cumsum(obj.pConfusionMatrix(iRow,:));
                    rnddraw = randdraw(obj);
                    iWfm = find(f>=rnddraw,1);
                    wfmIDsFA(iWfmsFA(m)) = allWfms(iWfm);
                end
            end
        end
        
        function [meas,covs,snrOut,tgtIDsOut,sysIDsOut,wfmIDsOut] = generateDetections(obj,truth)
            
            % If no targets lie inside the receiver's field of view, there
            % are no detections to generate
            if isempty(truth)
                num = getNumMeasDims(obj);
                meas = zeros(num,0);
                covs = zeros(num,num,0);
                snrOut = zeros(1,0);
                tgtIDsOut = zeros(1,0);
                sysIDsOut = zeros(1,0);
                wfmIDsOut = zeros(1,0);
                return
            end
            
            % Merge targets sharing resolution cells
            [resCells,uCells] = getResolutionCells(obj,truth);
            numResolved = size(uCells,1);
            
            if obj.HasNoise
                est = addNoiseToPointTargets(obj,truth);
            else
                est = truth;
            end
            
            % Initialize detections
            num = getNumMeasDims(obj);
            meas = zeros(num,numResolved);
            covs = zeros(num,num,numResolved);
            tgtIDsOut = NaN(1,numResolved);
            sysIDsOut = NaN(1,numResolved);
            wfmIDsOut = NaN(1,numResolved);
            snrOut = NaN(1,numResolved);
            
            azIdx = getMeasDimIdx(obj,'az');
            elIdx = getMeasDimIdx(obj,'el');
            rgIdx = getMeasDimIdx(obj,'rg');
            rrIdx = getMeasDimIdx(obj,'rr');
            
            % Exposes Pd for testing purposes
            obj.pTargetPds = NaN(numResolved,1);
            
            numDets = 0;
            for iCell = 1:numResolved
                % Find all targets in this resolution cell
                iTgts = find(all(bsxfun(@eq,uCells(iCell,:),resCells),2));

                % Can target be detected?
                snrdB = est.SNRdB(iTgts);
                iGd = ~isnan(snrdB);
                if ~any(iGd)
                    continue % no
                end
                iTgts = iTgts(iGd);
                snrdB = snrdB(iGd);
                snr = fusion.internal.UnitConversions.db2pow(snrdB);
                
                % Model Pd
                [~,iMax] = max(snrdB);
                cellSNR = sum(snr);
                cellSNRdB = fusion.internal.UnitConversions.pow2db(cellSNR);
                Pd = getPd(obj,cellSNRdB);
                
                % Save off Pd to expose for test
                obj.pTargetPds(iCell) = Pd;
                
                rnddraw = randdraw(obj);
                if rnddraw<Pd
                    numDets = numDets+1;
                    
                    tgtIDsOut(numDets) = est.PlatformID(iTgts(iMax));
                    sysIDsOut(numDets) = est.EmitterIndex(iTgts(iMax));
                    wfmIDsOut(numDets) = est.WaveformType(iTgts(iMax));
                    snrOut(numDets) = cellSNRdB;
                    
                    % Multiple targets within a resolution cell are merged
                    % by centroiding their individual contributions. This
                    % mimics what is typically done within a sensor's
                    % measurement estimation block
                    
                    azMerged = obj.centroid(est.Azimuth(iTgts),snr);
                    azSig = getAzimuthSigma(obj,cellSNR);
                    meas(azIdx,numDets) = azMerged;
                    covs(azIdx,azIdx,numDets) = azSig^2;
                    
                    if obj.HasElevation
                        elMerged = obj.centroid(est.Elevation(iTgts),snr);
                        elSig = getElevationSigma(obj,cellSNR);
                        meas(elIdx,numDets) = elMerged;
                        covs(elIdx,elIdx,numDets) = elSig^2;
                    end
                    
                    if obj.pHasRange
                        rgMerged = obj.centroid(est.Range(iTgts),snr);
                        rgSig = getRangeSigma(obj,cellSNR);
                        meas(rgIdx,numDets) = rgMerged;
                        covs(rgIdx,rgIdx,numDets) = rgSig^2;
                    end
                    
                    if obj.pHasRangeRate
                        rrMerged = obj.centroid(est.RangeRate(iTgts),snr);
                        rrSig = getRangeRateSigma(obj,cellSNR);
                        meas(rrIdx,numDets) = rrMerged;
                        covs(rrIdx,rrIdx,numDets) = rrSig^2;
                    end
                end
            end
            % Only keep indices w/ detections
            meas = meas(:,1:numDets);
            covs = covs(:,:,1:numDets);
            tgtIDsOut = tgtIDsOut(1:numDets);
            sysIDsOut = sysIDsOut(1:numDets);
            wfmIDsOut = wfmIDsOut(1:numDets);
            snrOut = snrOut(1:numDets);
            
            % Apply confusion matrix to waveforms
            if strcmpi(obj.DetectionMode,'passive')
                allWfms = obj.WaveformTypes;
                for m = 1:numel(wfmIDsOut)
                    thisWfm = wfmIDsOut(m);
                    iRow = allWfms==thisWfm;
                    probs = obj.pConfusionMatrix(iRow,:);
                    f = cumsum(probs);
                    rnddraw = randdraw(obj);
                    iWfm = find(f>=rnddraw,1);
                    wfmIDsOut(m) = allWfms(iWfm);
                end
            end
        end

        function truth = computeTruth(obj,signals, refSignal)
            
            numSignals = numel(signals);
            
            % Apply field of view constraints
            if numSignals<1
                truth = defaultTruthStruct(obj,signals);
            else
                % Compute transmitter positions in spherical coordinates
                txPos = reshape(matlabshared.tracking.internal.fusion.CodegenUtils.concatFieldValues(signals,'OriginPosition'),3,[]);
                txVel = reshape(matlabshared.tracking.internal.fusion.CodegenUtils.concatFieldValues(signals,'OriginVelocity'),3,[]);
                
                % Convert point targets to spherical coordinates
                [th,phi,rho] = cart2sph(txPos(1,:),txPos(2,:),txPos(3,:));
                az = rad2deg(th(:));
                el = rad2deg(phi(:));
                rg = rho(:);
                
                % Compute range-rate including refraction effects
                [x,y,z] = sph2cart(deg2rad(az),deg2rad(el),1);
                rgDir = [x(:) y(:) z(:)]';
                rr = dot(rgDir,txVel,1)';
                
                % obtain (cylindrical) spreading and propagation losses.
                spreadingLoss = 10*log10(rg);
                centerFreq = matlabshared.tracking.internal.fusion.CodegenUtils.concatFieldValues(signals,'CenterFrequency')';
                propLossdB = rg.*thorpeLoss(centerFreq);
                
                % Compute ambient noise power at receiver
                noisePower = obj.AmbientNoiseLevel + 10*log10(obj.Bandwidth);
                
                srclvldB = matlabshared.tracking.internal.fusion.CodegenUtils.concatFieldValues(signals,'SourceLevel')';
                tsdB = matlabshared.tracking.internal.fusion.CodegenUtils.concatFieldValues(signals,'TargetStrength')';
                snrdB = srclvldB(:) + tsdB - propLossdB - spreadingLoss - noisePower;

                propRg = matlabshared.tracking.internal.fusion.CodegenUtils.concatFieldValues(signals,'PropagationRange')';
                propRR = matlabshared.tracking.internal.fusion.CodegenUtils.concatFieldValues(signals,'PropagationRangeRate')';

                rg = (rg+propRg(:))/2;
                rr = (rr+propRR(:))/2;
                
                % Find signals which can be detected
                % - A signal can be detected if:
                %   1. It is a waveform type known by the sensor
                %   2. It lies completely within the sensor's operational bandwidth
                % - Otherwise, it is considered interference
                % - Detectable signal waveforms are assumed to be
                %   orthogonal (i.e. they do not interfere, even if they
                %   occupy the same spectrum)
                [opFc,opBW,opWfms] = getReceiverWaveforms(obj,refSignal);
                opFMin = opFc-opBW/2;
                opFMax = opFc+opBW/2;
                
                numSig = numel(signals);
                isDetectable = false(numSig,1);
                for m = 1:numSig
                    thisSig = signals(m);
                    
                    thisFc = thisSig.CenterFrequency;
                    thisBW = thisSig.Bandwidth;
                    thisFmin = thisFc-thisBW/2;
                    thisFmax = thisFc+thisBW/2;
                    thisWfm = thisSig.WaveformType;
                   
                    isDetectable(m) = any(ismember(thisWfm,opWfms)) && ...
                        thisFmin>=opFMin && thisFmax<=opFMax;
                end

                interference = signals(~isDetectable);
                fcInt = matlabshared.tracking.internal.fusion.CodegenUtils.concatFieldValues(interference,'CenterFrequency');
                bwInt = matlabshared.tracking.internal.fusion.CodegenUtils.concatFieldValues(interference,'Bandwidth');
                fMinInt = fcInt(:)-bwInt(:)/2;
                fMaxInt = fcInt(:)+bwInt(:)/2;
                snrdBInt = snrdB(~isDetectable);
                azInt = az(~isDetectable);
                elInt = el(~isDetectable);
                
                signals = signals(isDetectable);
                snrdB = snrdB(isDetectable);
                az = az(isDetectable);
                el = el(isDetectable);
                rg = rg(isDetectable);
                rr = rr(isDetectable);
                
                % Add processing gain and interference to SNR for detectable signals
                if obj.HasElevation
                    intGrid = [azInt(:)/obj.AzimuthResolution elInt(:)/obj.ElevationResolution];
                    sigGrid = [az(:)/obj.AzimuthResolution el(:)/obj.ElevationResolution];
                else
                    intGrid = azInt(:)/obj.AzimuthResolution;
                    sigGrid = az(:)/obj.AzimuthResolution;
                end
                intGrid = round(intGrid);
                sigGrid = round(sigGrid);
                
                numSig = numel(signals);
                for m = 1:numSig
                    thisCell = sigGrid(m,:);
                    thisSig = signals(m);
                    
                    % Find this signal's spectrum
                    thisFc = thisSig.CenterFrequency;
                    thisBW = thisSig.Bandwidth;
                    thisFmin = thisFc-thisBW/2;
                    thisFmax = thisFc+thisBW/2;
                    
                    % Find all interfering signals that fall in this same (az,el) cell
                    isInCell = all(bsxfun(@eq,intGrid,thisCell),2);
                    
                    % Find the fraction of the signal's spectrum occupied
                    % by each of the interfering signals
                    theseFMin = max(fMinInt(isInCell),thisFmin);
                    theseFMax = min(fMaxInt(isInCell),thisFmax);
                    theseBW = (theseFMax-theseFMin);
                    theseFrac = theseBW./thisBW;
                    theseSNR = fusion.internal.UnitConversions.db2pow(snrdBInt(isInCell));
                    theseSNRdB = fusion.internal.UnitConversions.pow2db(theseFrac(:).*theseSNR(:));
                    
                    % Adjust detectable signal's SNR
                    procGain = thisSig.ProcessingGain;
                    snrdB(m) = snrdB(m) + procGain - sum(theseSNRdB(:));
                end
                
                truth = defaultTruthStruct(obj,signals);
                truth.SNRdB(:) = snrdB;
                truth.Azimuth(:) = az;
                truth.Elevation(:) = el;
                truth.Range(:) = rg;
                truth.RangeRate(:) = rr;
            end
        end        
    end
    
    methods(Access = protected)
        function truth = defaultTruthStruct(~, signals)
            numSigs = numel(signals);
            truth = struct( ...
                'PlatformID', zeros(1,numSigs), ...
                'EmitterIndex', zeros(1,numSigs), ...
                'CenterFrequency', zeros(1,numSigs), ...
                'Bandwidth', zeros(1,numSigs), ...
                'RandomSignal', false(1,numSigs), ...
                'WaveformType', zeros(1,numSigs), ...
                'Azimuth', zeros(1,numSigs), ...
                'Elevation', zeros(1,numSigs), ...
                'Range', zeros(1,numSigs), ...
                'RangeRate', zeros(1,numSigs), ...
                'TargetStrength',  zeros(1,numSigs), ...
                'SNRdB', zeros(1,numSigs));
            
            % Copy over values in signals to truth
            flds = fieldnames(truth);
            if isstruct(signals)
                copyFields = fieldnames(signals);
            else
                copyFields = fieldnames(fusion.internal.interfaces.DataStructures.radarEmissionStruct());
            end
            for iFld = 1:numel(flds)
                thisFld = flds{iFld};
                if any(strcmpi(thisFld, copyFields))
                    truth.(thisFld)(:) = matlabshared.tracking.internal.fusion.CodegenUtils.concatFieldValues(signals,thisFld);
                end
            end
        end
    end
    
    methods(Access = protected)
        function [emiss, refEmiss, refConfig, ins, time] = parseSensorInput(obj, varargin)
            iArg = 1;
            
            emissIn = varargin{iArg};
            emiss = parseEmissions(obj, emissIn);
            
            iArg = iArg+1;
            
            if strcmpi(obj.DetectionMode,'monostatic')
                configs = varargin{iArg};
                [refEmiss, refConfig] = parseMonostaticInputs(obj, emiss, configs);
                iArg = iArg+1;
            else
                refEmiss = [];
                refConfig = [];
            end
            
            if obj.HasINS
                insIn = varargin{iArg};
                obj.checkNumericFields(insIn,mfilename,'ins');
                ins = assembleINS(obj,insIn);
                iArg = iArg+1;
            else
                ins = [];
            end
            
            time = varargin{iArg};
        end
    end
    
    methods(Access = private)
        function emiss = parseEmissions(obj, emissIn)
            % Parse emission input.
            
            % Validate emission input.
            emissValid = validateEmissions(obj, emissIn);

            % Return the emissions as structures
            numEM = numel(emissValid);
            emiss = repmat(fusion.internal.interfaces.DataStructures.sonarEmissionStruct(false),numEM,1);
            flds = fieldnames(emiss);
            
            % Return emissions with orientation using rotation matrices,
            % not quaternions
            for iEM = 1:numEM
                thisEmissIn = fusion.internal.remotesensors.CommonUtilities.getIndexed(emissValid,iEM);
                
                [orient, wasQuat] = fusion.internal.remotesensors.CommonUtilities.getRotmat(thisEmissIn.Orientation);
                
                if isstruct(emissIn) && (~wasQuat || coder.target('MATLAB'))
                    % If its a structure and the orientation was not a
                    % quaternion, we can just copy the input to the output.
                    %
                    % If we are not generating code, then MATLAB allows us
                    % to change the orientation datatype on assignment.
                    
                    if wasQuat
                        thisEmissIn.Orientation = orient;
                    end
                    thisEmissOut = thisEmissIn;
                else
                    % Otherwise, we need to copy over each of the fields
                    % from the input to the output structure.
                    thisEmissOut = emiss(iEM);
                    thisEmissOut.Orientation = orient;
                    
                    for iFld = 1:numel(flds)
                        thisFld = flds{iFld};
                        if ~strcmp(thisFld, 'Orientation')
                            % Orientation was copied over above.
                            thisEmissOut.(thisFld) = thisEmissIn.(thisFld);
                        end
                    end
                end
                
                emiss(iEM) = thisEmissOut;
            end
        end
        
        function emiss = validateEmissions(obj, emissIn)
            % Validate emission input.
            
            if isnumeric(emissIn) && isempty(emissIn)
                % Allow [] to be passed in for empty arrays of emissions
                emiss = emissIn;
            else
                if isstruct(emissIn)
                    % Allow users to pass in structs
                    type = fusion.internal.interfaces.DataStructures.structType(emissIn);
                    cond = ~strcmpi(type, 'sonarEmissionStruct');
                    if cond
                        if coder.target('MATLAB')
                            hereString = getString(message('shared_radarfusion:RemoteSensors:here'));
                            hereLink = ['<a href="matlab:help fusion.internal.interfaces.DataStructures/sonarEmissionStruct">' hereString '</a>'];
                            errMsg = getString(message('shared_radarfusion:RemoteSensors:expectStructType','sonarEmission',hereLink,'EMSONAR'));
                            error('shared_radarfusion:RemoteSensors:expectStructType',errMsg);
                        else
                            coder.internal.errorIf(cond,'shared_radarfusion:RemoteSensors:expectStructTypeCG','''sonarEmission''','EMSONAR');
                        end
                    end
                    
                    obj.checkNumericFields(emissIn,mfilename,'EMSONAR');
                    
                    emiss = fusion.internal.interfaces.DataStructures.assembleStructs(emissIn);
                else
                    validateattributes(emissIn,{'struct','sonarEmission','cell'},{},mfilename,'EMSONAR');
                    
                    cond = iscell(emissIn) && ~fusion.internal.remotesensors.CommonUtilities.allValidTypes(emissIn, 'sonarEmission');
                    coder.internal.errorIf(cond,'shared_radarfusion:RemoteSensors:IncorrectInputClass','EMSONAR','sonarEmission');
                    
                    % If emissIn is sonarEmission class then all fields
                    % will be already initialized and validated
                    emiss = emissIn;
                end
            end
        end
        
        function [refSignal, refConfig] = parseMonostaticInputs(obj, signals, configs)
            txID = matlabshared.tracking.internal.fusion.CodegenUtils.concatFieldValues(signals,'EmitterIndex');
            isDP = matlabshared.tracking.internal.fusion.CodegenUtils.concatFieldValues(signals,'PropagationRange') == 0;
            iSig = find(obj.EmitterIndex==txID(:) & isDP(:),1);
            refSignal = signals(iSig);
            
            obj.checkNumericFields(configs,mfilename,'configs');
            
            validateattributes(configs,{'struct'},{},mfilename,'TXCONFIGS');
            type = fusion.internal.interfaces.DataStructures.structType(configs);
            cond = ~strcmpi(type, 'emitterConfigStruct');
            if cond
                if coder.target('MATLAB')
                    hereString = getString(message('shared_radarfusion:RemoteSensors:here'));
                    hereLink = ['<a href="matlab:help(''fusion.internal.interfaces.DataStructures/emitterConfigStruct'')">' hereString '</a>'];
                    errMsg = getString(message('shared_radarfusion:RemoteSensors:expectStructType','txConfig',hereLink,'TXCONFIGS'));
                    error('shared_radarfusion:RemoteSensors:expectStructType',errMsg);
                else
                    coder.internal.errorIf(cond,'shared_radarfusion:RemoteSensors:expectStructTypeCG','txConfig','TXCONFIGS');
                end
            end
            
            txID = matlabshared.tracking.internal.fusion.CodegenUtils.concatFieldValues(configs,'EmitterIndex');
            iConfig = find(obj.EmitterIndex==txID(:),1);
            
            cond = isempty(iConfig);
            if cond
                coder.internal.error('shared_radarfusion:RemoteSensors:emitterConfigNotFound',obj.EmitterIndex);
            end
            
            refConfig = configs(iConfig);
        end
        
        function signalsOut = receivedSignals(obj,signalsIn,refSignal)
            % Returns only signals which fall within the receiver's
            % bandwidth and where the receiver falls within the
            % transmitter's field of view
            
            % Are any signals present at the receiver's input?
            numSig = numel(signalsIn);
            if numSig<1
                signalsOut = signalsIn;
                return % no
            end
            
            % Remove signals that do fall within the receiver's operational bandwidth
            [rxFc,rxBW] = getReceiverWaveforms(obj,refSignal);
            inBand = fusion.internal.remotesensors.EmissionUtilities.isEmissionInReceiverBand(rxFc,rxBW,signalsIn);
            signals1 = signalsIn(inBand);
            
            % Are any signals left?
            numSig = numel(signals1);
            if numSig<1
                signalsOut = signals1;
                return % no
            end
            
            % Remove signals where receiver is outside of transmitter's field of view
            [rxPos,rxOrient,rxFoV] = getReceiverBeam(obj,refSignal);
            iKeep = false(numSig,1);
            for iSig = 1:numSig
                % Signal is in platform's body frame
                thisSig = signals1(iSig);
                inFoV = fusion.internal.remotesensors.EmissionUtilities.isRxInTxFOV(rxPos,thisSig);
                iKeep(iSig) = inFoV;
            end
            signals2 = signals1(iKeep);
            
            % Are any signals left?
            numSig = numel(signals2);
            if numSig<1
                signalsOut = signals2;
                return % no
            end
            
            % Remove signals when the transmitter is not in the receiver's field of view
            iKeep = true(numSig,1);
            for iSig = 1:numSig
                % Signal is already in platform's body frame
                thisSig = signals2(iSig);
                inFoV = fusion.internal.remotesensors.EmissionUtilities.isTxInRxFOV(rxPos,rxOrient,rxFoV,thisSig);
                iKeep(iSig) = inFoV;
            end
            signals3 = signals2(iKeep);
            
            % Are any signals left?
            numSig = numel(signals3);
            if numSig<1
                signalsOut = signals3;
                return % no
            end
            
            % Remove reflected (i.e. not direct path) signals that are not
            % reflected to this detector's platform
            iKeep = true(numSig,1);
            for iSig = 1:numSig
                % Signal is already in platform's body frame
                thisSig = signals3(iSig);
                
                % Is this a direct-path signal?
                if thisSig.PropagationRange == 0
                    continue % Yes, skip to next signal
                end
                
                % Is this a reflected signal?
                isReflected = fusion.internal.remotesensors.EmissionUtilities.isReflectedToPlatform(thisSig);
                iKeep(iSig) = isReflected;
            end
            signalsOut = signals3(iKeep);
        end
        
        function [rxPos,rxOrient,rxFoV] = getReceiverBeam(obj,refSignal)
            % Returns the receiver's mounting location, orientation, and
            % field of view
            
            if strcmpi(obj.DetectionMode,'monostatic')
                rxPos = refSignal(1).OriginPosition(:);
                rxOrient = refSignal(1).Orientation;
                rxFoV = refSignal(1).FieldOfView;
            else
                rxPos = obj.MountingLocation(:);
                rxOrient = getLookOrientation(obj);
                rxFoV = obj.FieldOfView;
            end
        end
        
        function [fc,bw,wfms] = getReceiverWaveforms(obj,refSignal)
            % Returns waveform characteristics: center frequency,
            % bandwidth, and supported waveform types
            
            if strcmpi(obj.DetectionMode,'monostatic')
                fc = refSignal(1).CenterFrequency;
                bw = refSignal(1).Bandwidth;
                wfms = refSignal(1).WaveformType;
            else
                fc = obj.CenterFrequency;
                bw = obj.Bandwidth;
                wfms = obj.WaveformTypes;
            end
        end
    end
    
    % -----------------
    % Input validation:
    % -----------------
    methods(Access = protected)
        function num = getNumInputsImpl(obj)
            % Define total number of inputs for system with optional inputs
            
            % Signals and time
            num = 2;
            
            % Reference signal input
            if strcmpi(obj.DetectionMode,'monostatic')
                num = num+1; % refConfig
            end
            
            if obj.HasINS
                num = num+1;
            end
        end
        
        function validateInputsImpl(obj, varargin)
            % Validate inputs to the step method at initialization (or
            % whenever an input size changes)
            
            % signals and txConfig are currently checked in the input parsing
            [~, ~, ~, ~, time] = parseSensorInput(obj, varargin{:});
            
            validateInputsImpl@fusion.internal.remotesensors.ScanningMount(obj, time);
        end

        function flag = isInputSizeMutableImpl(~, ~)
            flag = true;
        end
        
        function flag = isInputComplexityMutableImpl(~, ~)
            flag = true;
        end
    end
    
    methods(Access = protected)
        function validatePropertiesImpl(obj)
            % Validate related or interdependent property values

            validatePropertiesImpl@fusion.internal.remotesensors.ScanningSensor(obj);
            
            % EmitterIndex must be specified by user when DetectionMode is 'monostatic'
            if strcmpi(obj.DetectionMode,'monostatic')
                cond = ~coder.internal.is_defined(obj.EmitterIndex);
                coder.internal.errorIf(cond,'shared_radarfusion:RemoteSensors:mustSpecifyProperty1WhenProperty2IsSetTo','EmitterIndex','DetectionMode','monostatic');
            end
            
            % ConfusionMatrix must be a scalar, a vector of length L, or an
            % LxL matrix with rows that sum to 1, where L =
            % numel(obj.WaveformTypes)
            if strcmpi(obj.DetectionMode,'passive')
                val = obj.ConfusionMatrix;
                
                % Check for required size
                reqLen = length(obj.WaveformTypes);
                cond = ~isscalar(val) && ...
                    (isvector(val) && length(val)~=reqLen) || ...
                    (ismatrix(val) && size(val,1)~=reqLen || size(val,2)~=reqLen);
                if cond
                    coder.internal.errorIf(cond,'shared_radarfusion:RemoteSensors:wrongConfusionMatrixSize');
                end
                
                % Check matrix rows sum to 1
                cond = ~isvector(val) && ~all(sum(val,2)==1);
                if cond
                    coder.internal.errorIf(cond,'shared_radarfusion:RemoteSensors:rowsDoNotSumTo','ConfusionMatrix','1');
                end
            end

            % HasINS must be true when DetectionCoordinates is set to 'Scenario'
            if strcmpi(obj.DetectionMode,'monostatic')
                cond = ~obj.HasINS && strcmpi(obj.DetectionCoordinates,'Scenario');
                if cond
                    coder.internal.errorIf(cond,'shared_radarfusion:RemoteSensors:scenarioNoINS');
                end
            end
        end
    end
    
    % ---------
    % Save/load
    % ---------
    methods(Access = protected)
        function loadObjectImpl(obj,s,wasLocked)
            % Set properties in object obj to values in structure s

            % Set private and protected properties
            if wasLocked
                obj.pConfusionMatrix = s.pConfusionMatrix;
                obj.pHasRange = s.pHasRange;
                obj.pHasRangeRate = s.pHasRangeRate;
            end

            if wasLocked || ~isempty(s.EmitterIndex)
                obj.EmitterIndex = s.EmitterIndex;
            end
            s = rmfield(s,'EmitterIndex');
            
            % Set perturbation related properties
            loadPerts(obj,s);

            % Set public properties and states
            loadObjectImpl@fusion.internal.remotesensors.ScanningSensor(obj,s,wasLocked);
        end

        function s = saveObjectImpl(obj)
            % Set properties in structure s to values in object obj

            % Set public properties and states
            s = saveObjectImpl@fusion.internal.remotesensors.ScanningSensor(obj);

            % Set private and protected properties
            if isLocked(obj)
                s.pConfusionMatrix = obj.pConfusionMatrix;
                s.pHasRange = obj.pHasRange;
                s.pHasRangeRate = obj.pHasRangeRate;
            end
            
            % Set perturbation related properties
            s = savePerts(obj, s);
        end
    end
    
    % --------------------
    % Detection definition
    % --------------------
    methods(Access = protected)
        function [det,argsToBus] = defaultOutput(obj)
            % Returns a representative "default" output generated by this
            % object
            
            len = getMeasurementLength(obj);
            attribs = defaultObjectAttributes(obj);
            ins = struct(...
                'Position',zeros(1,3),...
                'Velocity',zeros(1,3),...
                'Orientation',eye(3));
            det = assembleDetections(obj,ins,zeros(1),zeros(len,1),zeros(len),obj.HasElevation,obj.pHasRange,obj.pHasRangeRate,attribs);
            numDets = zeros(1);
            
            ins = struct( ...
                'Orientation', zeros(3), ...
                'Position', zeros(1,3), ...
                'Velocity', zeros(1,3));
            transforms = configTransforms(obj,ins,obj.HasElevation,obj.pHasRange,obj.pHasRangeRate);
            config = systemConfig(obj,false,false,transforms);
            argsToBus = {numDets,config};
        end
        
        function attribs = defaultObjectAttributes(obj)
            if strcmpi(obj.DetectionMode,'passive')
                attribs = {'TargetIndex',zeros(1),'EmitterIndex',zeros(1),'WaveformType',zeros(1),'SNR',zeros(1)};
            else
                attribs = {'TargetIndex',zeros(1),'EmitterIndex',zeros(1),'SNR',zeros(1)};
            end
        end
        
        function num = getNumMeasDims(obj)
            num = 1; % Always has azimuth
            if obj.HasElevation
                num = num+1;
            end
            if obj.pHasRange
                num = num+1;
            end
            if obj.pHasRangeRate
                num = num+1;
            end
        end
        
        function len = getMeasurementLength(obj)
            % Length of measurement property for objectDetection objects
            % returned by the sensor
            if strcmpi(obj.pDetectionCoordinates,'Sensor spherical')
                len = getNumMeasDims(obj);
            else
                % Body or Sensor Cartesian coordinates: 3 pos + 3 vel
                len = 3;
                if obj.pHasRangeRate
                    len = len+3;
                end
            end
        end
    end
    
    % ----------------
    % Display settings
    % ----------------
    
    % General
    methods(Access = protected)
        function flag = isInactivePropertyImpl(obj,prop)
            % Return false if property is visible based on object 
            % configuration, for the command line and System block dialog
            if obj.pIsSettingConfig
                flag = false;
                return
            end
            
            flag = isInactivePropertyImpl@fusion.internal.remotesensors.ScanningSensor(obj,prop);
            
            % Disable elevation properties
            flag = flag || ~obj.HasElevation && ...
                any(strcmpi(prop,{'ElevationResolution','ElevationBiasFraction'}));
            
            % Disable range and range-rate properties for angle only
            % measurements
            flag = flag || strcmpi(obj.DetectionMode, 'passive') && ...
                any(strcmpi(prop,{'RangeResolution','RangeRateResolution','RangeBiasFraction','RangeRateBiasFraction', ...
                'HasRangeRate','HasRangeAmbiguities','HasRangeRateAmbiguities', ...
                'MaxUnambiguousRange','MaxUnambiguousRadialSpeed', ...
                'DetectionCoordinates'}));
            
            % Disable passive only properties for monostatic configuration
            flag = flag || strcmpi(obj.DetectionMode, 'monostatic') && ...
                any(strcmpi(prop,{'UpdateRate','ScanMode', ...
                'MountingLocation', 'MountingAngles', ...
                'FieldOfView','MaxMechanicalScanRate','MechanicalScanLimits','MechanicalAngle',...
                'CenterFrequency','Bandwidth','WaveformTypes','ConfusionMatrix'}));

            % Disable monostatic only properties for passive configuration
            flag = flag || ~strcmpi(obj.DetectionMode, 'monostatic') && ...
                any(strcmpi(prop,{'EmitterIndex'}));
            
            % Disable range-rate properties when range-rate is not enabled
            flag = flag || ~obj.HasRangeRate && strcmpi(prop,'HasRangeRateAmbiguities');
        end
    end
    
    % MATLAB display
    methods(Access=protected)
        function shortGroups = getPropertyGroups(obj)
            longGroups = getPropertyGroupsLongImpl(obj);
            shortGroups = longGroups(1:4);
            
            % Only keep FAR and AmbientNoiseLevel
            detGroup = shortGroups(4);
            detList = detGroup.PropertyList(1:2);
            detGroup = matlab.mixin.util.PropertyGroup(detList);
            shortGroups(4) = detGroup;
        end
    
        function groups = getPropertyGroupsLongImpl(obj)
            groups = sonarSensor.getPropertyGroupsImpl();
            groups = obj.convertSystemToMixinGroup(groups);
        end
    end
    
    % Simulink dialog
    methods(Static, Access = protected)
        function groups = getPropertyGroupsImpl
            % Define property section(s) for System block dialog
            scanningGroups = getPropertyGroupsImpl@fusion.internal.remotesensors.ScanningSensor();
            
            mainSection = scanningGroups(1);
            mntSection = scanningGroups(2);
            detSection = scanningGroups(3);
            hasSection = scanningGroups(4);
            detFmtSection = scanningGroups(5);
            
            pList = {'DetectionMode','EmitterIndex'};
            mainSection.PropertyList = [mainSection.PropertyList,pList];
            
            pList = {'MaxUnambiguousRange','MaxUnambiguousRadialSpeed'};
            detSection.PropertyList = [pList,detSection.PropertyList];
            
            pList = {'FalseAlarmRate','AmbientNoiseLevel','CenterFrequency','Bandwidth','WaveformTypes','ConfusionMatrix'};
            rxSection = matlab.system.display.Section('PropertyList',pList);
            
            pList = {'AzimuthResolution','ElevationResolution','RangeResolution','RangeRateResolution'};
            resSection = matlab.system.display.Section('PropertyList',pList);
            
            pList = {'AzimuthBiasFraction','ElevationBiasFraction','RangeBiasFraction','RangeRateBiasFraction'};
            biasSection = matlab.system.display.Section('PropertyList',pList);
            
            pList = {'HasElevation', ...
                'HasRangeRate','HasRangeAmbiguities','HasRangeRateAmbiguities', ...
                'HasINS', 'HasNoise', ...
                'HasFalseAlarms'};
            hasSection.PropertyList = pList;
            
            pList = {'DetectionCoordinates'};
            detFmtSection.PropertyList = [detFmtSection.PropertyList,pList];
            
            groups = [mainSection, mntSection, detSection, rxSection, resSection, biasSection, hasSection, detFmtSection];
        end
    end
    
    % -------------
    % Perturbations
    % -------------
    methods(Access = protected)
        function perts = defaultPerturbations(obj)
            if strcmpi(obj.DetectionMode, "passive")
                perturbableProps = {"FalseAlarmRate", "AmbientNoiseLevel", ...
                    "CenterFrequency", "Bandwidth", ...
                    "WaveformTypes", "ConfusionMatrix", "AzimuthResolution", ...
                    "ElevationResolution", "AzimuthBiasFraction", ...
                    "ElevationBiasFraction"}; %#ok<CLARRSTR>
            elseif strcmpi(obj.DetectionMode, "monostatic")
                perturbableProps = {"FalseAlarmRate", "AmbientNoiseLevel", ...
                    "AzimuthResolution", ...
                    "ElevationResolution", "RangeResolution", "RangeRateResolution", ...
                    "AzimuthBiasFraction", "ElevationBiasFraction", ...
                    "RangeBiasFraction", "RangeRateBiasFraction"}; %#ok<CLARRSTR>
            end
            perts = struct(...
                'Property', perturbableProps, ...
                'Type', "None", ...
                'Value', {{NaN, NaN}}...
                );
        end
        
        function intProps = integerProperties(~)
            %integerProperties Get the list of integer-valued properties
            intProps = "WaveformTypes";
        end
    end

    % --------
    % Simulink
    % --------
    methods(Static, Hidden)    
        function flag = isAllowedInSystemBlock
            flag = false;
        end
    end
end

function y = thorpeLoss(f)
%thorpeLoss  Frequency-dependent attenuation 
    % Compute frequency-dependent attenuation in dB/m using Thorpe's
    % equation (pg. 108 in [1]).
    freqkHz=f/1e3;         
    y = ((3.3*10^-3)+(.11*freqkHz.^2)./(1+freqkHz.^2)+(44*freqkHz.^2)./(4100+freqkHz.^2)+(3*10^-4)*freqkHz.^2);
    y = y/1000; % dB/m
end

