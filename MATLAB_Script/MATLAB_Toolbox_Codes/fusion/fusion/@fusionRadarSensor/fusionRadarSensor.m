classdef fusionRadarSensor < matlabshared.tracking.internal.fusion.EmissionsRadarDataGenerator
%fusionRadarSensor  Sensor simulation object used to generate
%radar detections and tracks
%   RDR = fusionRadarSensor returns a statistical model to simulate
%   detections for a scanning radar.
%
%   RDR = fusionRadarSensor(ID) returns the statistical radar target report
%   generator, RDR, with its SensorIndex property set to ID.
% 
%   RDR = fusionRadarSensor(..., CFG) returns the statistical radar target
%   report generator, RDR, configured with the scan configuration, CFG. CFG
%   can be one of 'No scanning' | 'Raster' | 'Rotator' | 'Sector'.
%
%   RDR = fusionRadarSensor(..., 'Name', value) returns a fusionRadarSensor
%   object by specifying its properties as name-value pair arguments.
%   Unspecified properties have default values. See the list of properties
%   below.
%
%   Step method syntax: click on <a href="matlab:help('fusionRadarSensor/stepImpl')">step</a> for more details. The step method
%   generated target reports from the scenario truth data. When the radar
%   is configured to report detections, a cell array of objectDetection
%   objects are returned. When configured to report tracks, an array of
%   objectTrack objects are returned.
%
%   System objects may be called directly like a function instead of using
%   the step method. For example, y = step(obj) and y = obj() are
%   equivalent.
%
%   fusionRadarSensor properties:
%     SensorIndex               - Unique identifier of sensor
%     UpdateRate                - Update rate (Hz)
%     MountingLocation          - Mounted location (x,y,z) of radar (m)
%     MountingAngles            - Mounting rotation angles of radar (deg)
%     DetectionMode             - Detection mode used to generate target reports
%     ScanMode                  - Surveillance scan mode
%     MaxAzimuthScanRate        - Maximum mechanical azimuth scan rate (deg/s)
%     MaxElevationScanRate      - Maximum mechanical elevation scan rate (deg/s)
%     MechanicalAzimuthLimits   - Mechanical azimuth scan limits (deg)
%     MechanicalElevationLimits - Mechanical elevation scan limits (deg)
%     ElectronicAzimuthLimits   - Electronic azimuth scan limits (deg)
%     ElectronicElevationLimits - Electronic elevation scan limits (deg)
%     MechanicalAngle           - Current mechanical scan angle (deg)
%     ElectronicAngle           - Current electronic scan angle (deg)
%     LookAngle                 - Current look angle (deg)
%     InterferenceInputPort     - Enable interference input
%     EmissionsInputPort        - Enable emissions input
%     EmitterIndex              - Unique identifier of external monostatic emitter
%     CenterFrequency           - Center frequency of radar (Hz)
%     Bandwidth                 - Half power bandwidth of operational band (Hz)
%     WaveformTypes             - Types of waveforms detected by receiver
%     ConfusionMatrix           - Probability of correctly classifying a detected waveform
%     Sensitivity               - Minimum operational sensitivity of receiver (dBmi)
%     DetectionThreshold        - Minimum SNR required to declare a detection (dB)
%     DetectionProbability      - Probability of detecting the reference target
%     ReferenceRange            - Range of the reference target (m)
%     ReferenceRCS              - Radar cross section of the reference target (dBsm)
%     FalseAlarmRate            - Probability of a false observation in a resolution cell
%     FieldOfView               - Total angular field of view (deg)
%     RangeLimits               - Minimum and maximum ranges (m)
%     RangeRateLimits           - Minimum and maximum range rates (m/s)
%     MaxUnambiguousRange       - Maximum unambiguous detection range (m)
%     MaxUnambiguousRadialSpeed - Maximum unambiguous detection radial speed (m/s)
%     AzimuthResolution         - Azimuthal resolution (deg)
%     ElevationResolution       - Elevation resolution (deg)
%     RangeResolution           - Range resolution (m)
%     RangeRateResolution       - Range rate resolution (m/s)
%     AzimuthBiasFraction       - Fractional azimuthal bias component
%     ElevationBiasFraction     - Fractional elevation bias component
%     RangeBiasFraction         - Fractional range bias component
%     RangeRateBiasFraction     - Fractional range rate bias component
%     HasElevation              - Enable elevation angle measurements
%     HasRangeRate              - Enable range rate measurements
%     FilterInitializationFcn   - A handle to a function that initializes
%                                 a tracking filter based on a detection
%     ConfirmationThreshold     - Mc and Nc values for track confirmation
%     DeletionThreshold         - Md and Nd values for track deletion
%     HasNoise                  - Add measurement noise to observations
%     HasFalseAlarms            - Add false alarms to observations
%     HasOcclusion              - Enable line-of-sight occlusion
%     HasRangeAmbiguities       - Enable range ambiguities
%     HasRangeRateAmbiguities   - Enable range rate ambiguities
%     HasINS                    - Enable INS struct input
%     MaxNumReportsSource       - Source of maximum number of target reports
%     MaxNumReports             - Maximum number of targets reported
%     TargetReportFormat        - Format of generated target reports
%     DetectionCoordinates      - Coordinate frame used to report detections
%     TrackCoordinates          - Coordinate frame used to report tracks
%     Profiles                  - Physical characteristics of platforms
%
%   fusionRadarSensor methods:
%     <a href="matlab:help('fusionRadarSensor/stepImpl')">step</a>          - Generate statistical radar target reports
%     perturbations  - Define perturbations to the fusionRadarSensor
%     perturb        - Apply perturbations to the fusionRadarSensor
%     release        - Allow property value and input characteristics changes
%     clone          - Create fusionRadarSensor object with same property values
%     isLocked       - Locked status (logical)
%     reset          - Reset states of fusionRadarSensor object
%     coverageConfig - Report the fusionRadarSensor object scanning coverage configuration
%
%   % EXAMPLE 1: Model an air traffic control tower
%
%   % Create targets.
%   tgt1 = struct( ...
%       'PlatformID', 1, ...
%       'Position', [0 -50e3 -1e3], ...
%       'Velocity', [0 900*1e3/3600 0]);
% 
%   tgt2 = struct( ...
%       'PlatformID', 2, ...
%       'Position', [20e3 0 -500], ...
%       'Velocity', [700*1e3/3600 0 0]);
% 
%   tgt3 = struct( ...
%       'PlatformID', 3, ...
%       'Position', [-20e3 0 -500], ...
%       'Velocity', [300*1e3/3600 0 0]);
% 
%   % Create an airport surveillance radar 15 meters above the ground
%   rpm = 12.5;
%   fov = [1.4;5]; % [azimuth; elevation]
% 
%   scanrate = rpm*360/60;  % deg/s
%   updaterate = scanrate/fov(1); % Hz
% 
%   sensor = fusionRadarSensor(1, 'Rotator', ...
%       'UpdateRate', updaterate, ...
%       'MountingLocation', [0 0 -15], ...
%       'MaxAzimuthScanRate', scanrate, ...
%       'FieldOfView', fov, ...
%       'AzimuthResolution', fov(1));
% 
%   % Generate detections from a full scan of the radar
%   simTime = 0;
%   detBuffer = {};
%   while true
%       [dets, numDets, config] = sensor([tgt1 tgt2 tgt3], simTime);
%       detBuffer = [detBuffer;dets]; %#ok<AGROW>
%   
%       % Is full scan complete?
%       if config.IsScanDone
%           break % yes
%       end
%       simTime = simTime+1/sensor.UpdateRate;
%   end
%   
%   radarPosition = [0, 0, 0];
%   tgtPositions = [tgt1.Position;tgt2.Position;tgt3.Position];
%
%   clrs = lines(3);
%
%   figure; hold on;
%   plot3(radarPosition(1),radarPosition(2),radarPosition(3),'Marker','s',...
%       'DisplayName','Radar','MarkerFaceColor',clrs(1,:),'LineStyle', 'none');
%
%   % Plot truth
%   plot3(tgtPositions(:,1),tgtPositions(:,2),tgtPositions(:,3),'Marker', '^',...
%       'DisplayName','Truth','MarkerFaceColor',clrs(2,:),'LineStyle', 'none');
%
%   % Plot detections
%   if ~isempty(detBuffer)
%       detPos = cellfun(@(d)d.Measurement(1:3),detBuffer,...
%           'UniformOutput',false);
%       detPos = cell2mat(detPos')';
%       plot3(detPos(:,1),detPos(:,2),detPos(:,3),'Marker', 'o',...
%           'DisplayName','Detections','MarkerFaceColor',clrs(3,:),'LineStyle', 'none');
%   end
%
%   xlabel('X(m)');
%   ylabel('Y(m)');
%   axis('equal');
%   legend();
%
%   % EXAMPLE 2: Model an array that scans in azimuth and elevation
%
%   % Create targets.
%   tgt1 = struct( ...
%       'PlatformID', 1, ...
%       'Position', [0 -50e3 -1e3], ...
%       'Speed', 900*1e3/3600);
%   
%   tgt2 = struct( ...
%       'PlatformID', 2, ...
%       'Position', [20e3 0 -500], ...
%       'Speed', 700*1e3/3600);
%   
%   tgt3 = struct( ...
%       'PlatformID', 3, ...
%       'Position', [-20e3 0 -500], ...
%       'Speed', 300*1e3/3600);
% 
%   % Create a radar which uses an electronic raster scan within a volume
%   % of interest south of the radar's location
%   sensor = fusionRadarSensor(1, 'Raster', 'ScanMode', 'Electronic',...
%               'MountingAngles', [180 0 0]);
% 
%   % Generate detections from a full scan of the radar
%   simTime = 0;
%   detBuffer = {};
%   while true
%       [dets, numDets, config] = sensor([tgt1 tgt2 tgt3], simTime);
%       detBuffer = [detBuffer;dets]; %#ok<AGROW>
%   
%       % Is full scan complete?
%       if config.IsScanDone
%           break % yes
%       end
%       simTime = simTime+1/sensor.UpdateRate;
%   end
% 
%   radarPosition = [0, 0, 0];
%   tgtPositions = [tgt1.Position;tgt2.Position;tgt3.Position];
%
%   clrs = lines(3);
%
%   figure; hold on;
%   plot3(radarPosition(1),radarPosition(2),radarPosition(3),'Marker','s',...
%       'DisplayName','Radar','MarkerFaceColor',clrs(1,:),'LineStyle', 'none');
%
%   % Plot truth
%   plot3(tgtPositions(:,1),tgtPositions(:,2),tgtPositions(:,3),'Marker', '^',...
%       'DisplayName','Truth','MarkerFaceColor',clrs(2,:),'LineStyle', 'none');
%
%   % Plot detections
%   if ~isempty(detBuffer)
%       detPos = cellfun(@(d)d.Measurement(1:3),detBuffer,...
%           'UniformOutput',false);
%       detPos = cell2mat(detPos')';
%       plot3(detPos(:,1),detPos(:,2),detPos(:,3),'Marker', 'o',...
%           'DisplayName','Detections','MarkerFaceColor',clrs(3,:),'LineStyle', 'none');
%   end
%
%   xlabel('X(m)');
%   ylabel('Y(m)');
%   axis('equal');
%   legend();
%
%   See also: radarEmitter, radarChannel, objectDetection, objectTrack,
%   trackerGNN, trackingScenario

% References:
% [1] M.Richards, Fundamentals of Radar Signal Processing, 2nd ed.,
% McGraw-Hill Professional Engineering, 2014.
%
% [2] Joint Range Instrumentation Accuracy Improvement Group, "Radar Loop
% Gain Measurements", Range Commanders Council, White Sands, New Mexico,
% 1998
%
% [3] A.W.Doerry, "Radar Range Measurements in the Atmosphere," Sandia
% Report, 2013
%
% [4] A.W.Doerry, "Earth Curvature and Atmospheric Refraction Effects on
% Radar Signal Propagation," Sandia Report, 2013

% Copyright 2020-2021 The MathWorks, Inc.

    properties(Access = protected)
        Version
    end
    
    properties(Constant, Hidden)
        ScanModeSet = matlab.system.internal.MessageCatalogSet({ ...
            'shared_tracking:EmissionsRadarDataGenerator:NoScanning', ...
            'shared_tracking:EmissionsRadarDataGenerator:Mechanical', ...
            'shared_tracking:EmissionsRadarDataGenerator:Electronic', ...
            'shared_tracking:EmissionsRadarDataGenerator:MechanicalAndElectronic'});
    end

    properties(Nontunable)
        %ScanMode  Surveillance scan mode
        %   Specify the scan mode used by the sensor as one of 'No
        %   scanning' | 'Mechanical' | 'Electronic' | 'Mechanical and
        %   electronic'. When set to 'No scanning', no scanning is
        %   performed by the sensor. When set to 'Mechanical', the sensor
        %   scans mechanically across the azimuth limits specified by the
        %   MechanicalAzimuthLimits property and across the elevation
        %   limits specified by the MechanicalElevationLimits property.
        %   When set to 'Electronic', the sensor scans electronically
        %   across the azimuth limits specified by the
        %   ElectronicAzimuthLimits property and across the elevation
        %   limits specified by the ElectronicElevationLimits property.
        %   When set to 'Mechanical and electronic', the sensor
        %   mechanically scans the antenna boresight across the mechanical
        %   scan limits and electronically scans beams relative to the
        %   antenna boresight across the electronic scan limits. The total
        %   field of regard scanned in this mode is the combination of the
        %   mechanical and electronic scan limits.
        %
        %   In all scan modes except 'No scanning', the scan positions step
        %   by the sensor's field of view between dwells.
        %
        %   Default: 'Mechanical'
        ScanMode = 'Mechanical'
    end

    properties(Dependent)
        %LookAngle  Current look angle (deg)
        %   LookAngle is a read-only property reporting the current look
        %   angle which is the combination of the device's current
        %   mechanical and electronic angles. If ScanMode is 'Mechanical',
        %   then LookAngle will match MechanicalAngle, since no electronic
        %   scanning is present. When ScanMode is 'Electronic', LookAngle
        %   will match ElectronicAngle.
        LookAngle
    end    

    properties(Nontunable)
        %DetectionProbability  Probability of detecting the reference target
        %   Probability of detecting the reference target as a scalar value
        %   on the interval (0,1].
        %
        %   Default: 0.9
        DetectionProbability = 0.9

        %ReferenceRange  Range of the reference target
        %   Range of the reference target in meters (m).
        %
        %   Default: 100e3
        ReferenceRange = 100e3

        %ReferenceRCS  Radar cross section of the reference target (dBsm)
        %   Radar cross section (RCS) of the reference target as a scalar
        %   value in decibel square meters (dBsm).
        %
        %   Default: 0
        ReferenceRCS = 0

        %FalseAlarmRate  Probability of a false observation in a resolution cell
        %   Specify a scalar value defining the probability of reporting a
        %   false detection within an observation cell.
        %
        %   Default: 1e-6
        FalseAlarmRate = 1e-6
    end

    % Constructor
    methods
        function obj = fusionRadarSensor(varargin)
            % Support name-value pair arguments when constructing object
            obj@matlabshared.tracking.internal.fusion.EmissionsRadarDataGenerator(varargin{:});
            
            obj.Version = ver('fusion');
        end
    end
    
    methods
        function val = get.LookAngle(obj)
            val = obj.getLookAngle();
        end

        function set.LookAngle(obj, val)
            obj.setLookAngle(val);
        end
    end

    % Most of the configuration takes place in the setupImpl of the parent class
    
    % All of the computation at the moment is done in the stepImpl of the base class
    methods(Access = protected)
        [rpts,numRpts,config] = stepImpl(obj,varargin)
        
        function loadObjectImpl(obj,s,wasLocked)
            % Set properties in object obj to values in structure s
            
            % Set public properties and states
            loadObjectImpl@matlabshared.tracking.internal.fusion.EmissionsRadarDataGenerator(obj,s,wasLocked);
        end
        
        function s = saveObjectImpl(obj)
            % Set properties in structure s to values in object obj
            
            % Set public properties and states
            s = saveObjectImpl@matlabshared.tracking.internal.fusion.EmissionsRadarDataGenerator(obj);
        end

        function flag = isInputSizeMutableImpl(~, ~)
            flag = true;
        end
        
        function flag = isInputComplexityMutableImpl(~, ~)
            flag = true;
        end
    end
end

% LocalWords:  rdr dBmi dBsm Nc fov scanrate updaterate dets clrs McGraw Doerry Sandia
