classdef irSensor< fusion.internal.remotesensors.ScanningSensor & fusion.internal.remotesensors.mixin.DisplayUtils & radarfusion.internal.scenario.mixin.DetectionSensor & scenario.internal.mixin.Perturbable
%irSensor Infrared (IR) detections generator
%   sensor = irSensor(sensorIndex) returns a statistical model to simulate
%   detections for an infrared sensor.
%
%   sensor = irSensor(sensorIndex, scanConfig) configures the IR sensor to
%   use a predefined scan configuration, scanConfig. scanConfig can be one
%   of 'No scanning' | 'Raster' | 'Rotator' | 'Sector'.
%
%   sensor = irSensor(..., 'Name', value) returns an irSensor object by
%   specifying its properties as name-value pair arguments. Unspecified
%   properties have default values. See the list of properties below.
%
%   Step method syntax:
%   
%   DETS = step(SENSOR, TARGETS, TIME) generates detections from the
%   L-element array of structs, TARGETS, at the current simulation time,
%   TIME. TIME is a scalar value in seconds. DETS is an M-element cell
%   array of objectDetection objects. The sensor generates detections at
%   the rate defined by the UpdateRate property. The detections are
%   returned in the sensor's spherical coordinate frame as [az;el]. When
%   HasElevation is false, only azimuth is reported.
%
%   TARGETS is a struct array of platform structures. The fields are listed
%   <a href="matlab:help fusion.internal.interfaces.DataStructures/platformStruct">here</a>.
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
%       TargetIndex     Unique identifier of the platform which generated
%                       the detection. For false alarms, this value is
%                       negative.
%
%       SNR             Signal-to-noise ratio of the detection in decibels.
%
%   DETS = step(SENSOR, TARGETS, INS, TIME) includes pose information for
%   the sensor's platform provided by the INS input with the reported
%   detections. This input is enabled when the HasINS property is set to
%   true. The INS information can be used by tracking and fusion algorithms
%   to estimate the target positions in the NED frame. INS is a struct with
%   the following fields:
%
%        Position      Position of the GPS receiver in the local NED
%                      coordinate system specified as a real finite 1-by-3 
%                      array in meters.
% 
%        Velocity      Velocity of the GPS receiver in the local NED 
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
%   When the irSensor's MaxNumDetectionsSource property is set to 'Auto',
%   NUMDETS is always set to the length of DETS. When
%   MaxNumDetectionsSource is set to 'Property', DETS is always a cell
%   array with length determined by the value of the MaxNumDetections
%   property. In this case, the first NUMDETS elements of DETS hold valid
%   detections and the remaining elements of DETS are set to a default
%   value.
%   
%   Detections can only be reported by irSensor at time intervals given by
%   the reciprocal of the UpdateRate property. The IsValidTime flag on the
%   CONFIG structure is set to false when detection updates are requested
%   at times that are not aligned with the configured update rate.
%
%   irSensor can also be used to generate detections using targets
%   generated by <a
%   href="matlab:help('trackingScenario')">trackingScenario</a>.
%
%   irSensor properties:
%     SensorIndex               - Unique identifier of sensor
%     UpdateRate                - Sensor update rate
%     ScanMode                  - Scan mode used by sensor
%     MountingLocation          - Sensor's mounting location on platform
%     MountingAngles            - Sensor's mounting angles on platform
%     FieldOfView               - Angular field of view for a sensor dwell
%                                 (read-only)
%     MaxMechanicalScanRate     - Maximum mechanical scan rate
%     MechanicalScanLimits      - Mechanical scan limits
%     MechanicalAngle           - Mechanical antenna angle (read-only)
%     LookAngle                 - Look angle of sensor (read-only)
%     LensDiameter              - Diameter of the circular lens
%     FocalLength               - Focal length of the circular lens
%     NumDetectors              - Number of detectors
%     CutoffFrequency           - Cutoff frequency of the modulation
%                                 transfer function
%     DetectorArea              - Area of the single detector element 
%     Detectivity               - Material detectivity of a detector
%                                 element
%     NoiseEquivalentBandwidth  - Noise equivalent bandwidth of the sensor
%     FalseAlarmRate            - Rate at which false alarms are reported
%     AzimuthResolution         - Azimuthal resolution (read-only)
%     ElevationResolution       - Elevation resolution (read-only)
%     AzimuthBiasFraction       - Fractional azimuthal bias component
%     ElevationBiasFraction     - Fractional elevation bias component
%     HasElevation              - Elevation scanning and measurements
%     HasAngularSize            - Enable angular size measurement
%     HasINS                    - Enable input of platform's pose
%     HasNoise                  - Add noise to measurements
%     HasFalseAlarms            - Enable false detections
%     HasOcclusion              - Enable occlusion of extended objects
%     MinClassificationArea     - Minimum image size for classification
%     MaxAllowedOcclusion       - Maximum allowed occlusion for detector
%     MaxNumDetectionsSource    - Source of maximum number of detections
%     MaxNumDetections          - Maximum number of reported detections
%
%   irSensor methods:
%     step            - Generate infrared detections from targets
%     perturbations   - Define perturbations to the irSensor
%     perturb         - Apply perturbations to the irSensor
%     release         - Allow property value and input characteristics changes
%     clone           - Create irSensor object with same property values
%     isLocked        - Locked status (logical)
%     <a href="matlab:help matlab.System/reset   ">reset</a>           - Reset states of irSensor object
%     <a href="matlab:help coverageConfig">coverageConfig</a>  - Report the irSensor object scanning coverage configuration
%
%   % Example: Detect a target with an IR sensor.
%
%   % Create a target.
%   tgt = struct( ...
%       'PlatformID', 1, ...
%       'Position', [10e3 0 0], ...
%       'Speed', 900*1e3/3600);
%
%   % Create an IR sensor.
%   sensor = irSensor(1);
%
%   % Generate detection from target.
%   time = 0;
%   [dets, numDets, config] = sensor(tgt, time)
%
%   See also: fusionRadarSensor, trackingScenario.

 
%   Copyright 2018-2021 The MathWorks, Inc.

    methods
        function out=irSensor
            % Support name-value pair arguments when constructing object
        end

        function out=defaultObjectAttributes(~) %#ok<STOUT>
        end

        function out=defaultOutput(~) %#ok<STOUT>
        end

        function out=defaultPerturbations(~) %#ok<STOUT>
        end

        function out=getAzimuthSigma(~) %#ok<STOUT>
            % SNR dependent azimuth standard deviation
            % SNR is linear (not in decibels)
        end

        function out=getElevationSigma(~) %#ok<STOUT>
            % SNR dependent elevation standard deviation
            % SNR is linear (not in decibels)
        end

        function out=getFieldOfViewImpl(~) %#ok<STOUT>
        end

        function out=getMeasurementLength(~) %#ok<STOUT>
            % Length of measurement property for objectDetection objects
            % returned by the sensor
        end

        function out=getNumMeasDims(~) %#ok<STOUT>
            % Number of dimensions measured by sensor
        end

        function out=getPropertyGroups(~) %#ok<STOUT>
        end

        function out=getPropertyGroupsImpl(~) %#ok<STOUT>
            % Define property section(s) for System block dialog
        end

        function out=getPropertyGroupsLongImpl(~) %#ok<STOUT>
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

        function out=saveObjectImpl(~) %#ok<STOUT>
            % Set properties in structure s to values in object obj
        end

        function out=setupImpl(~) %#ok<STOUT>
        end

        function out=stepImpl(~) %#ok<STOUT>
        end

    end
    properties
        %AzimuthBiasFraction  Fractional azimuthal bias component
        %   Specify a nonnegative scalar defining the azimuthal bias
        %   component of the sensor as a fraction of the sensor's azimuthal
        %   resolution defined by the AzimuthResolution property value.
        %   This value sets a lower bound on the azimuthal accuracy of the
        %   sensor. This property only applies for modes where the sensor
        %   is scanning.
        %
        %   Default: 0.1
        AzimuthBiasFraction;

        %AzimuthResolution  Azimuthal resolution (deg)
        %   A read-only property defining the azimuthal resolution of the
        %   sensor. The sensor's azimuthal resolution defines the minimum
        %   separation in azimuth angle at which the sensor can distinguish
        %   two targets. The azimuth resolution is derived from the focal
        %   length of the lens and the number of columns in the detector's
        %   imaging plane. Defined in degrees.
        AzimuthResolution;

        %CutoffFrequency Cutoff frequency of the modulation transfer function (MTF)
        %   Specify the cutoff frequency corresponding to the modulation
        %   transfer function (MTF) of the infrared sensing system as a
        %   scalar value in Hz. The cutoff frequency determines the
        %   standard deviation of the sensor's angular measurements when
        %   the ScanMode property is set to a scanning mode. Increasing the
        %   MTF cutoff frequency decreases the standard deviation of the
        %   angular measurements. When the ScanMode property is set to 'No
        %   scanning', the standard deviation of the sensor's angular
        %   measurements is determined only by the sensor's angular
        %   resolution.
        %
        %   Default: 20e3
        CutoffFrequency;

        %Detectivity Specific detectivity of the detector material
        %   Specify the detectivity of the material used to fabricate the
        %   detectors as a scalar value in units of cm-sqrt(Hz)/W.
        %
        %   Default: 1.2e10
        Detectivity;

        %DetectorArea Area of a single infrared detector element
        %   Area of a single infrared detector element (pixel) in
        %   square-meters.
        %
        %   Default: 1.44e-6
        DetectorArea;

        %ElevationBiasFraction  Fractional elevation bias component
        %   Specify a nonnegative scalar defining the elevation bias
        %   component of the radar as a fraction of the radar's elevation
        %   resolution defined by the ElevationResolution property value.
        %   This value sets a lower bound on the elevation accuracy of the
        %   radar. This property only applies when you set HasElevation
        %   property to true and the sensor is scanning.
        %
        %   Default: 0.1
        ElevationBiasFraction;

        %ElevationResolution  Elevation resolution (deg)
        %   A read-only property defining the elevation resolution of the
        %   sensor. The sensor's elevation resolution defines the minimum
        %   separation in elevation angle at which the sensor can
        %   distinguish two targets. The elevation resolution is derived
        %   from the focal length of the lens and the number of rows in the
        %   detector's imaging plane.  This property only applies when you
        %   set HasElevation property to true. Defined in degrees.
        ElevationResolution;

        %FalseAlarmRate  Rate at which false alarms are reported
        %   Specify a scalar value on the interval [1e-7 1e-3] defining the
        %   probability of reporting a false detection within each
        %   resolution cell of the sensor. Resolution cells are determined
        %   from the AzimuthResolution and when enabled the
        %   ElevationResolution properties.
        %
        %   Default: 1e-6
        FalseAlarmRate;

        %FocalLength Focal length of sensor's circular lens
        %   FocalLength is a scalar.  f = F * s, where F is the focal
        %   length in millimeters, and s is the number of pixels per
        %   millimeter. Thus, f is in pixels.
        %
        %   Default: 800
        FocalLength;

        %HasAngularSize  Enable angular size reporting
        %   Set this property to true to additionally return azimuth 
        %   and elevation size in the reported detections.  Set this
        %   property to false to only report only azimuth and elevation 
        %   locations without their angular extent.
        %
        %   Default: false
        HasAngularSize;

        %HasFalseAlarms  Enable false detections
        %   Set this property to true to include false alarms in the
        %   reported detections. Set this property to false to report only
        %   true detections.
        %
        %   Default: true
        HasFalseAlarms;

        %HasOcclusion  Enable occlusion of extended objects
        %   Set this property to true to model occlusion of objects by the
        %   spatial extent of other objects in the scenario. Set this
        %   property to false to disable occlusion of objects. Setting this
        %   property to false will also disable the merging of objects
        %   sharing a common sensor resolution cell. This ensures that
        %   every object in the scenario has an opportunity to generate a
        %   detection.
        %
        %   Default: true
        HasOcclusion;

        %LensDiameter Diameter of sensor's circular lens
        %   Diameter of the sensor's circular lens in meters.
        %
        %   Default: 8e-2
        LensDiameter;

        %MaxAllowedOcclusion  Maximum allowed occlusion for detector
        %   Specify a real scalar on the interval [0,1) defining the maximum
        %   occlusion of an object that can still be detected by the camera.
        %   Represented as the fraction of the object's total surface area
        %   invisible to the camera.
        %  
        %   Default: 0.5
        MaxAllowedOcclusion;

        %MinClassificationArea  Minimum image size for classification
        %   Set MinClassificationArea to the minimum area (in square
        %   pixels) of the bounding box defined by the azimuth and
        %   elevation extent required to properly classify the object. If
        %   the reported area is less than the minimum, then the reported
        %   ClassID is set to zero in the returned objectDetection.
        %   Otherwise, the ClassID is taken from the corresponding target
        %   in the input TARGETS struct.
        %
        %   Default: 100
        MinClassificationArea;

        %NoiseEquivalentBandwidth Noise equivalent bandwidth of the sensor
        %   Specify the noise equivalent bandwidth of the sensor as a
        %   scalar value in Hz.
        %
        %   Default: 30
        NoiseEquivalentBandwidth;

        %NumDetectors Number of detectors
        %   Number of infrared detectors in the sensor's imaging plane as a
        %   2-element vector. The first element defines the number of rows
        %   in the imaging plane and the second element defines the number
        %   of columns in the imaging plane. The number of rows corresponds
        %   to the sensor's elevation resolution and the number of columns
        %   corresponds to the sensor's azimuth resolution.
        %
        %   Default: [1000 1000]
        NumDetectors;

        %ScanMode  Scan mode used by sensor
        %   Specify the scan mode used by the sensor as one of 'No
        %   scanning' | 'Mechanical'. When set to 'No scanning', no
        %   scanning is performed by the sensor. When set to 'Mechanical',
        %   the sensor scans mechanically across the azimuth and elevation
        %   limits specified by the MechanicalScanLimits property.
        %
        %   In all scan modes except 'No scanning', the scan positions step
        %   by the sensor's field of view between dwells.
        %
        %   Default: 'Mechanical'
        ScanMode;

    end
end