classdef trackingSensorConfiguration< fusion.internal.AbstractTrackingSensorConfiguration & matlabshared.tracking.internal.fusion.CustomDisplay
% trackingSensorConfiguration Represent a sensor configuration for tracking
% config = trackingSensorConfiguration(sensorIndex) creates a sensor
% configuration for a sensor to be used with the trackerPHD and
% trackerGridRFS. The config allows you to specify the sensor parameters
% like clutter density, sensor limits and resolution for each sensor used
% with the tracker. It also allows you to specify how the sensor perceives
% the state of a track to detect it using properties like
% SensorTransformFcn, SensorTransformParameters and
% FilterInitializationFcn.
%
% When used with trackerPHD, the configuration enables the tracker to
% perform four main routine operations:
%
% 1. Evaluate the probability of detection at points in state-space.
% 2. Compute expected number of detections from a target.
% 3. Initiate components in the probability hypothesis density.
% 4. Obtain the clutter density of the sensor.
%
% When used with trackerGridRFS, the configuration assists the tracker to
% project sensor data on the 2-D grid. The tracker uses the
% SensorTransformParameters to calculate the ego vehicle's and sensor's
% location and orientation in the tracking coordinate frame. The tracker
% uses the SensorLimits to calculate the field of view and maximum range of
% the sensor. The SensorTransformFcn and FilterInitializationFcn properties
% of trackingSensorConfiguration are not relevant for trackerGridRFS.
% 
% config = trackingSensorConfiguration(sensorIndex) creates a
% trackingSensorConfiguration with SensorIndex set to the sensorIndex. The
% config created by this syntax is representative of radar sensor with
% field of view 20 and 5 in azimuth and elevation respectively.It can
% detect targets up to a maximum range of 1000 meters. The resolutions are
% set to 4 deg, 2 deg and 10 meters in azimuth, elevation and range
% respectively. In addition to the properties above, the config proposes
% components in the density using a constant velocity model. It specifies
% the probability of detection to be 0.9 and assumes a clutter density of
% 1e-3 false alarms per unit volume.
%
% config = trackingSensorConfiguration(sensorIndex,'Name',value)
% allows specifying properties of the object during construction.
%
% sensorConfiguration properties:
% SensorIndex               -   Unique identifier for the sensor
% IsValidTime               -   A flag indicating if this sensor should be
%                               used for updating the tracker
% SensorLimits              -   Limits on sensor's ability to detect a
%                               target
% SensorResolution          -   Resolution of each parameter provided in
%                               SensorLimits
% SensorTransformFcn        -   A function handle to transform a track
%                               state to sensor's detection space
% SensorTransformParameters -   A struct or array of struct which contains
%                               information about transformation of track
%                               state to sensor's detection space
% DetectionProbability      -   Probability of detecting a target inside
%                               SensorLimits
% ClutterDensity            -   Expected number of false alarms per unit
%                               volume of the sensor
% FilterInitializationFcn   -   A function_handle to specify a filter
%                               initialization function
% MaxNumDetsPerObject       -   Maximum number of detections that the
%                               sensor can generate per object
%
% % Example: Represent a radar model using trackingSensorConfiguration
% % -------------------------------------------------------------------
% % Consider a radar with the ability to detect targets inside its
% % angular field of view with maximum range of 500 meters and maximum
% % range-rate of 50 m/s
% azLimits = [-10 10];
% elLimits = [-2.5 2.5];
% rangeLimits = [0 500];
% rangeRateLimits = [-50 50];
% sensorLimits = [azLimits;elLimits;rangeLimits;rangeRateLimits];
% sensorResolution = [5 2 10 3]; % [az el r rr];
% 
% % Specifying the function handle to transform the state into
% % [az;el;range;rr]. The signature of this function is similar to the
% % cvmeas measurement model. Hence, you can use the function cvmeas as
% % SensorTransformFcn
% transformFcn = @cvmeas;
% 
% % To specify the parameters required for cvmeas, you can use the
% % SensorTransformParameters property of the config. Here, we represent
% % that the sensor is mounted at the center of the platform and the
% % platform is located at [100;30;20] and is moving with a velocity of
% % [-5;4;2] units in the scenario.
% 
% % This structure defines the sensor mounting on the platform.
% params(1) = struct('Frame','Spherical','OriginPosition',[0;0;0],...
%                  'OriginVelocity',[0;0;0],'HasRange',true,...
%                  'HasVelocity',true);
% % This structure defines the platform location, velocity, and orientation
% % in the scenario.
% params(2) = struct('Frame','Rectangular','OriginPosition',[100;30;20],...
%                   'OriginVelocity',[-5;4;2],'HasRange',true,...
%                   'HasVelocity',true);
% 
% % Creating the configuration
% config = trackingSensorConfiguration('SensorIndex',3,'SensorLimits',sensorLimits,...
%                                      'SensorResolution',sensorResolution,...
%                                      'SensorTransformParameters',params,...
%                                      'SensorTransformFcn',@cvmeas);
%
%
% % Example2: Represent a radar model using trackingSensorConfiguration
% % using fusionRadarSensor model.
% % -------------------------------------------------------------------
% % Consider a radar with the ability to detect targets inside its angular
% % field of view with maximum range of 500 meters and maximum range-rate of
% % 50 m/s.
% 
% sensor = fusionRadarSensor(1, 'FieldOfView',[20 5],'RangeLimits',[0 500], ...
% 'HasRangeRate',true,'HasElevation',true,'RangeRateLimits',[-50 50], ...
% 'AzimuthResolution',5,'RangeResolution',10,'ElevationResolution',2, ...
% 'RangeRateResolution',3);
% 
% % Specifying the function handle to transform the state into
% % [az;el;range;rr]. The signature of this function is similar to the
% % cvmeas measurement model. Hence, you can use the function cvmeas as
% % SensorTransformFcn
% transformFcn = @cvmeas;
%  
% config = trackingSensorConfiguration(sensor,'SensorTransformFcn',transformFcn);
% 
% % This structure defines the platform location and velocity in the 
% % scenario.
% config.SensorTransformParameters(2).OriginPosition = [100;30;20];
% config.SensorTransformParameters(2).OriginVelocity = [-5;4;2];
%
% See also: partitionDetections, cvmeas, initcvggiwphd, initcaggiwphd,
% initctggiwphd.

 
% Copyright 2018-2021 The MathWorks, Inc.

    methods
        function out=trackingSensorConfiguration
            % obj = trackingSensorConfiguration(1);
            % obj = trackingSensorConfiguration('SensorIndex',1);
            % Both signatures above create default values.
        end

        function out=allocateMemory(~) %#ok<STOUT>
            % Is SensorIndex specified as property or first variable
        end

        function out=getPropertyGroups(~) %#ok<STOUT>
        end

    end
    properties
        % ClutterDensity Expected number of false alarms per unit volume
        % from this sensor.
        ClutterDensity;

        % DetectionProbability Probability of detecting a target inside the
        % SensorLimits
        DetectionProbability;

        %FilterInitializationFcn  Filter initialization function name
        %   Specify the function for initializing the PHD filter used
        %   by tracker. The function must support the following signatures:
        %       filter = FilterInitializationFcn()
        %       filter = FilterInitializationFcn(detections)
        %
        %   filter     - a valid PHD filter that has the components of
        %                density initialized.
        %   detections - a cell array of objectDetection that initiates
        %                components in the density
        %   The no-input argument function signature allows you to specify
        %   the birth density in PHD filter without using detections
        FilterInitializationFcn;

        % IsValidTime A flag indicating if the tracker must be updated
        % with this sensor.
        % Default: false.
        IsValidTime;

        % MaxNumDetsPerObject Maximum number of detections the sensor can
        % report for a given object. For sensors reporting one detection
        % per object, this number should be set to 1.
        MaxNumDetsPerObject;

        % MinDetectionProbability Probability of detecting a target
        % estimated to be outside the SensorLimits.
        MinDetectionProbability;

        % SensorIndex Unique identifier for the sensor
        SensorIndex;

        % SensorLimits Upper and lower limits of the sensor's ability to
        % detect a target. For example, [minAz maxAz;minEl maxEl;minR maxR]
        % for a sensor which can detect all targets inside its angular
        % field of view and within a certain range. Az, El and R refers to
        % azimuth, elevation and range respectively.
        SensorLimits;

        % SensorResolution Resolution of the sensor for each parameter
        % specified in the SensorLimits. For example,
        % [azResolution;elResolution;rangeResolution] for a sensor which
        % can detect all targets inside its angular field of view and
        % within a certain range. Resolution must be a N-by-1 vector, where
        % N is the number of rows in the Limits. For a sensor which does
        % not have a resolution in one of the parameters, you can specify
        % its resolution as equal to the maximum minus minimum limits of
        % the parameter, which is inferred as one resolution cell per
        % parameter.
        SensorResolution;

        % SensorTransformFcn A function to transform a track state to the
        % sensor detection state. For example, [x;vx;y;vy;z;vz] in
        % scenario to [az;el;range] in sensor frame. It must support the
        % following signature function
        % 	detStates = SensorTransformFcn(trackStates,params)
        % where params are the values stored in SensorTransformParameters
        % The signature of the function is similar to measurement models,
        % therefore you can use a measurement function cvmeas as the
        % SensorTransformFcn.
        %
        % When used with gmphd for non-extended targets or with ggiwphd:
        %
        % detStates is a N-by-M matrix, where N is the number of rows in
        % the SensorLimits and M is the number of input states.
        %
        % When used with gmphd for extended targets:
        %
        % The SensorTransformFcn allows you to specify multiple detStates
        % per trackState. In this case, detStates can be a N-by-M-by-S
        % matrix, where S are the number of sources on the state. For
        % example, if the target is described by a rectangular state, the
        % sources can be the corners of the rectangle. If any of the source
        % falls inside the SensorLimits, the target is declared detectable.
        % To calculate expected number of detections for extended targets,
        % the trackingSensorConfiguration uses the spread (max - min) of
        % each detStates of the source and uses the ratio of spreads and
        % resolution on each limit to calculate the expected number of
        % detections. You can override this by providing an optional output
        % from the SensorTransformFcn:
        %
        % [..., Nexp] = SensorTransformFcn(trackStates, params)
        %
        % where Nexp are the expected number of detections from
        % each state.
        %
        SensorTransformFcn;

        % SensorTransformParameters Additional parameters passed as input to
        % the TransformFcn. For example, to transform a state
        % [x;vx;y;vy;z;vz] in scenario to [az;el;range] in sensor's frame,
        % we need the information about sensor and its mounting platform.
        % You can define these using an array of struct in the
        % following manner:
        %
        % First define the state of sensor with respect to the platform
        % origin.
        % senToPlatStruct =
        % struct('Frame','Spherical','OriginPosition',[0;0;0],...
        %         'OriginVelocity',[0;0;0],'Orientation',eye(3),...
        %         'HasRange',true,'HasVelocity',false,'HasAzimuth',true,...
        %         'HasElevation',true);
        %
        % Then, define the state of the platform with respect to the
        % scenario origin.
        %
        % platToScenarioStruct =
        % struct('Frame','Rectangular','OriginPosition',[100;50;0],...
        %         'OriginVelocity',[10;5;2],'Orientation',eye(3),...
        %         'HasRange',true,'HasVelocity',true,'HasAzimuth',true,...
        %         'HasElevation',true);
        %
        % The cvmeas function returns the azimuth, elevation and range
        % using the following transformParameters.
        % transformParameters = [senToPlatStruct;platToScenarioStruct];
        SensorTransformParameters;

    end
end
