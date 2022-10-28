classdef trackerGridRFS< matlabshared.tracking.internal.fusion.AbstractTracker
% trackerGridRFS A grid-based multi-object tracker
% tracker = trackerGridRFS create a multi-object, multi-sensor
% grid-based tracker to track objects. The tracker is aimed to track
% dynamic objects around an autonomous system using high resolution
% sensor data such as point clouds and radar detections. The tracker
% uses the RFS-based approach combined with Dempster-Shafer
% approximations defined in [1] to estimate the dynamic characteristics
% of the grid cells. To extract objects from the grid, the tracker uses
% a cell-to-track based association scheme [2].
%
% The tracker initializes, confirms and delete tracks automatically by
% using the following algorithm:
%
% The tracker projects sensor data from all sensors on 2-dimensional
% grid maps to represent the occupancy and free evidence in a
% Dempster-Shafer framework.
%
% The tracker uses a particle-based approach to estimate the dynamic
% state of the 2-D grid. This helps the tracker to classify each cell
% as dynamic or static.
%
% The tracker then associates each dynamic grid cell with the existing
% tracks using a gated Nearest-Neighbor approach. It also initializes
% new tracks using unassigned dynamic grid cells. A track is created
% with a 'Tentative' status and if it is updated enough times, its
% status will change to 'Confirmed' (See ConfirmationThreshold).
% Alternatively, if the ObjectClassID of the track is set to a positive
% value after track initialization (See TrackInitializationFcn), it
% will be confirmed immediately. Unassigned tracks are predicted to
% current time (coasting) and tracks with more misses that allowed (see
% DeletionThreshold) will deleted.
%
% tracker = trackerGridRFS('Name', value) creates a trackerGridRFS
% object by specifying its properties as name-value pair arguments.
% Unspecified properties have default values. See the list of
% properties below.
% 
% Step method syntax:
%
% confTracks = step(tracker, sensorData, time) updates the
% tracker with the current sensor information.
%
% confTracks is an array of objectTrack objects, where each element
% describes the track of an object. The state-convention of the track
% depends on the MotionModel property.
%
% sensorData is an n-element struct array. Each element of the array
% must define the measurement from a high resolution sensor using the
% following fields:
%
%   Time                  - Time of sensor data report (sec)
%   SensorIndex           - Unique identifier of the sensor
%   Measurement           - A P-by-N array which defines the N
%                           measurements obtained by the sensor at the
%                           reported time. Each measurement is a vector
%                           of length P. The measurements define the
%                           positional aspects of returns in
%                           Rectangular or Spherical frame. 
%   MeasurementParameters - A struct defining the transformation from
%                           state of the particles to sensor
%                           measurements. See |cvmeas| for an example.
%
% time is a positive scalar value denoting current time in seconds.
%
% confTracks = step(tracker, sensorData, configs, time) allows you to
% update the configurations of the sensor. You can use this syntax when
% HasSensorConfigurationsInput is set to true.
%
% configs is an array of structs or a cell array of
% trackingSensorConfiguration objects. If provided as a struct, configs
% can contain a subset of properties of trackingSensorConfiguration
% class or the configuration output of sensor model. The fields on the
% sensor configuration output struct are defined
% <a href="matlab:help('fusion.internal.interfaces.DataStructures/sensorConfigStruct')">here</a>.
%
% [confTracks,tentTracks,allTracks] = step(...) also outputs the list
% of tentative tracks and all tracks.
%
% [confTracks,tentTracks,allTracks,map] = step(...) also outputs the
% the dynamic map estimate from the tracker, map. See
% dynamicEvidentialGridMap for details on accessing estimate from the
% map.
%
% trackerGridRFS properties:
%   TrackerIndex                 - Unique Identifier of the tracker
%   SensorConfigurations         - Configuration of the sensors
%   HasSensorConfigurationsInput - Enable update of sensor configurations
%   StateParameters              - StateParameters for output tracks
%   MaxNumSensors                - Maximum number of sensors
%   MaxNumTracks                 - Maximum number of tracks
%   GridLength                   - Dimensions of the grid along X (m)
%   GridWidth                    - Dimensions of the grid along Y (m)
%   GridResolution               - Resolution of the grid (cells/m)
%   GridOriginInLocal            - Location of left corner of grid in 
%                                  Local or Ego coordinates
%   MotionModel                  - Motion model for tracking
%   VelocityLimits               - Minimum and maximum velocity of 
%                                  objects (m/s)
%   AccelerationLimits           - Minimum and maximum acceleration of 
%                                  objects (m/s^2)
%   TurnRateLimits               - Minimum and maximum turn-rate (deg/s)
%   ProcessNoise                 - Process noise covariance matrix
%   HasAdditiveProcessNoise      - A flag to indicate if process noise
%                                  is additive
%   NumParticles                 - Number of persistent particles
%   NumBirthParticles            - Number of new-born particles per step
%   BirthProbability             - Probability of birth in a cell per
%                                  step
%   DeathRate                    - Rate of decay of targets per unit
%                                  time
%   FreeSpaceDiscountFactor      - Reliability in free space prediction
%   Clustering                   - Clustering method used for new object
%                                  extraction
%   ClusteringThreshold          - Threshold for DBSCAN clustering
%   MinNumCellsPerCluster        - Minimum number of cells per cluster
%                                 for DBSCAN
%   TrackInitializationFcn       - A function to initialize a new track
%   TrackUpdateFcn               - A function to update an existing 
%                                  track
%   AssignmentThreshold          - Threshold for assigning dynamic grid
%                                  cells to tracks
%   ConfirmationThreshold        - Threshold for confirmation of a track
%                                  using history-based logic [M N]
%   DeletionThreshold            - Threshold for deletion of a track
%                                  using history-based logic [P Q]
%   NumTracks                    - Current number of tracks 
%   NumConfirmedTracks           - Current number of confirmed tracks
%   UseGPU                       - Enable dynamic map estimate using
%                                  gpuArray
%
% trackerGridRFS methods:
%   step                    - Creates, updates, and deletes tracks
%   predictTracksToTime     - Predict the tracks to a time stamp
%   predictMapToTime        - Predict the dynamic map to a time stamp
%   showDynamicMap          - Show dynamic map visualization
%   release                 - Allow property value and input characteristics change
%   clone                   - Creates a copy of the trackerGridRFS
%   isLocked                - Locked status (logical)
%   <a href="matlab:help matlab.System/reset   ">reset</a>                   - Resets states of the trackerGridRFS
%   
% trackerGridRFS Static methods:
%   defaultTrackInitialization - Default initialization of rectangular track
%   defaultTrackUpdate         - Default update of rectangular track
% 
% References: 
%
% [1] Nuss, Dominik, et al. "A random finite set approach
% for dynamic occupancy grid maps with real-time application." The
% International Journal of Robotics Research 37.8 (2018): 841-866.
% 
% [2] Steyer, Sascha, Georg Tanzmeister, and Dirk Wollherr. "Object
% tracking based on evidential dynamic occupancy grids in urban
% environments." 2017 IEEE Intelligent Vehicles Symposium (IV). IEEE,
% 2017.
%
% Example: Track targets in a tracking scenario
% ---------------------------------------------
% % Create a tracking scenario
% scene = trackingScenario('UpdateRate',5,'StopTime',5);
% 
% % For reproducible results
% rng(2021);
% 
% % Add a platform with the sensor
% plat = platform(scene);
% lidar = monostaticLidarSensor(1,'DetectionCoordinates','Body');
% 
% % Add two targets within -50 and 50
% for i = 1:2
%     target = platform(scene);
%     x = 50*(2*rand - 1);
%     y = 50*(2*rand - 1);
%     vx = 5*(2*rand - 1);
%     vy = 5*(2*rand - 1);
%     target.Trajectory.Position = [x y 0];
%     target.Trajectory.Velocity = [vx vy 0];
%     target.Trajectory.Orientation = quaternion([atan2d(vy,vx),0,0],'eulerd','ZYX','frame');
%     target.Mesh = extendedObjectMesh('sphere');
%     target.Dimensions = struct('Length',4,...
%         'Width',4,...
%         'Height',2,...
%         'OriginOffset',[0 0 0]);
% end
% 
% % Configuration of the sensor for tracking
% config = trackingSensorConfiguration(1,...
%     'SensorLimits',[-180 180;0 100],...
%     'SensorTransformParameters',struct,...
%     'IsValidTime',true);
% 
% % Create a tracker
% tracker = trackerGridRFS('SensorConfigurations',config,...
%     'AssignmentThreshold',5,...
%     'MinNumCellsPerCluster',4,...
%     'ClusteringThreshold',3);
% 
% % Visualization of tracks and truth
% tp = theaterPlot('XLimits',[-50 50],'YLimits',[-50 50]);
% trkPlotter = trackPlotter(tp,'DisplayName','Tracks','MarkerFaceColor','g');
% tthPlotter = platformPlotter(tp,'DisplayName','Truths','MarkerFaceColor','r','ExtentAlpha',0.2);
% 
% % Advance scenario and run the tracker on lidar data
% while advance(scene)
%     % Current time
%     time = scene.SimulationTime;
%     
%     % Generate point cloud
%     tgtMeshes = targetMeshes(plat);
%     [ptCloud, config] = lidar(tgtMeshes, time);
%     
%     % Format the data for the tracker
%     sensorData = struct('Time',time,...
%         'SensorIndex',1,...
%         'Measurement',ptCloud',...
%         'MeasurementParameters',struct...
%         );
%     
%     % Call tracker using sensorData
%     tracks = tracker(sensorData, time);
%     
%     % Visualize tracks
%     pos = zeros(numel(tracks),3);
%     vel = zeros(numel(tracks),3);
%     orient = quaternion.ones(numel(tracks),1);
%     dim = repmat(plat.Dimensions,numel(tracks),1);
%     ids = string([tracks.TrackID]);
%     
%     for i = 1:numel(tracks)
%         vel(i,:) = [tracks(i).State(2);tracks(i).State(4);0];
%         pos(i,:) = [tracks(i).State(1);tracks(i).State(3);0];
%         dim(i).Length = tracks(i).State(6);
%         dim(i).Width = tracks(i).State(7);
%         orient(i) = quaternion([tracks(i).State(5) 0 0],'eulerd','ZYX','frame');
%     end
%     trkPlotter.plotTrack(pos,dim,orient,ids);
%     
%     % Visualize platforms
%     pos = vertcat(tgtMeshes.Position);
%     meshes = vertcat(tgtMeshes.Mesh);
%     orient = vertcat(tgtMeshes.Orientation);
%     tthPlotter.plotPlatform(pos,meshes,orient);
% end
%
%
% See also: trackingSensorConfiguration, dynamicEvidentialGridMap

     
    % Copyright 2020-2021 The MathWorks, Inc.

    methods
        function out=trackerGridRFS
            % Create the projector
        end

        function out=cloneImpl(~) %#ok<STOUT>
        end

        function out=defaultTrackInitialization(~) %#ok<STOUT>
        end

        function out=defaultTrackUpdate(~) %#ok<STOUT>
        end

        function out=getClusteringFcn(~) %#ok<STOUT>
            % f = getClusteringFcn(obj) captures both DBSCAN and
            % CustomClusteringFcn using a single function handle
        end

        function out=getExpectedProcessNoiseSize(~) %#ok<STOUT>
        end

        function out=getNumInputsImpl(~) %#ok<STOUT>
        end

        function out=getPropertyGroups(~) %#ok<STOUT>
        end

        function out=isAllowedInSystemBlock(~) %#ok<STOUT>
        end

        function out=isInactivePropertyImpl(~) %#ok<STOUT>
        end

        function out=loadObjectImpl(~) %#ok<STOUT>
        end

        function out=predictMapToTime(~) %#ok<STOUT>
            % predictMapToTime Predicts the map to a timestamp
            % map = predictMapToTime(tracker, time) predicts the dynamic
            % map estimate from the tracker to a time instant, time. This
            % method does not change the state of the tracker.
            % 
            % ... = predictMapToTime(..., 'WithStateAndCovariance', tf)
            % allows you to control whether the state and state covariance
            % of the map should be predicted by specifying tf as true or
            % false. If tf is specified as false, only the evidences and
            % occupancy of the map is predicted. The state, state
            % covariance and the classification of a cell as static or
            % dynamic is not predicted. Specifying tf as false allows you
            % to predict the occupancy of the environment faster. The
            % default value of tf is true.
        end

        function out=predictTracksToTime(~) %#ok<STOUT>
            %predictTracksToTime Predicts the tracks to a time stamp
            %   predictedTracks = predictTracksToTime(obj, trackID, time)
            %   predicts the state of the track specified by trackID to
            %   time instant time. time must be greater than the last
            %   update time provided to the tracker in the previous step.
            %
            %   ... = predictTracksToTime(obj, category, time) allows
            %   you to predict the states of all the tracks that match the
            %   category. Valid values for category are 'all', 'confirmed',
            %   or 'tentative'.
            %
            %   ... = predictTracksToTime(...,'WithCovariance',tf) allows
            %   you to specify whether the state covariance of each track
            %   should be predicted as well by setting the tf flag to true
            %   or false. Predicting the covariance is slower than
            %   predicting just the track states. The default is false.
            %
            % Note: the tracker must be updated at least once to be able to
            % predict tracks.
        end

        function out=releaseImpl(~) %#ok<STOUT>
            % Release resources
        end

        function out=resetImpl(~) %#ok<STOUT>
            % Reset the tracker to its original state
        end

        function out=saveObjectImpl(~) %#ok<STOUT>
        end

        function out=setAndValidateProcessNoise(~) %#ok<STOUT>
        end

        function out=setEstimatorProperty(~) %#ok<STOUT>
        end

        function out=setExtractorStateParameters(~) %#ok<STOUT>
        end

        function out=setStateParameters(~) %#ok<STOUT>
        end

        function out=setupImpl(~) %#ok<STOUT>
            % Get class from sensor data
        end

        function out=setupProjector(~) %#ok<STOUT>
            % All properties are already synced. set UseGPU and ClassToUse.
            % The RFS filter is initialized in setup, so this projector
            % needs to be setup right now
        end

        function out=showDynamicMap(~) %#ok<STOUT>
            % showDynamicMap(tracker) plots the dynamic occupancy grid map
            % in local coordinates. The cells which are declared static are
            % visualized using a gray-scale image, where the grayness
            % represents the occupancy probability of the cell. To
            % visualize dynamic cells, the tracker uses HSV (hue,
            % saturation, value) values on an RGB colormap. The HSV values
            % for each grid cell are calculated using the following
            % convention:
            %
            % Hue is defined as orientation/360, where orientation is the
            % direction of the velocity vector in degrees. As hue increases
            % from 0 to 1, the color transitions from red to orange,
            % yellow, green, cyan, blue, magenta and finally back to red.
            %
            % Saturation is defined as the Mahalanobis distance (d) between
            % the velocity distribution of the grid cell and zero velocity.
            % A cell with d > 4 is drawn with full saturation (equal to 1)
            %
            % Value is defined as the occupancy probability of the cell.
            %
            % showDynamicMap(tracker,Name,value) allows specifying
            % additional name-value pairs defined below.
            %
            % 'PlotVelocity'  - A flag to control if velocity vectors must
            %                   be plotted in the visualization. If
            %                   specified as true, the velocity vector for
            %                   each dynamic cell is plotter at the center
            %                   of the grid cell. The length of the vector
            %                   represents the velocity magnitude in
            %                   tracking frame.
            %               
            %                   Default: false
            %
            % 'Parent'        - Axes to plot the map, specified as an axes
            %                   handle.
            %
            %                   Default: gca
            %
            % 'FastUpdate'    - Boolean value used to speed up show method
            %                   for existing map plots. If you have
            %                   previously plotted your map on the axes,
            %                   specify 1 to perform a lightweight update
            %                   to the map in the figure.
            %
            %                   Default: true
            %
            % 'InvertColors'  - Boolean value to specify if the colors of
            %                   the static occupied cells should be
            %                   inverted. In the default settings, empty
            %                   spaces are white and occupied spaces are
            %                   black. When this flag is specified as true,
            %                   the colors are inverted.
            %               
            %                   Default: false
            %
            % See also: hsv2rgb, rgb2hsv
        end

        function out=stepImpl(~) %#ok<STOUT>
        end

        function out=validateInputsImpl(~) %#ok<STOUT>
        end

        function out=validateProcessNoiseSize(~) %#ok<STOUT>
        end

        function out=validatePropertiesImpl(~) %#ok<STOUT>
        end

        function out=validateTimeStamps(~) %#ok<STOUT>
            % Validate tracker update time
        end

    end
    properties
        % AccelerationLimits Specify the minimum and maximum of
        % acceleration (m/s^2) in tracking coordinate frame as a 2-by-2
        % matrix. The first row corresponds to the acceleration limits in X
        % direction and second row corresponds to the acceleration limits
        % in Y direction. This property is only active when MotionModel is
        % set to 'constant-acceleration'
        %
        % Default: [-5 5;-5 5]
        AccelerationLimits;

        % Threshold for assigning a dynamic grid cell to a track
        % Specify the threshold that controls the assignment of a dynamic
        % grid cell to a track as a positive scalar value. A dynamic grid
        % cell can only be associated to a track if its distance from the
        % track is less than the AssignmentThreshold.
        %
        % Increase the value if a dynamic cell is not being assigned to a
        % track. Decrease the value if there are dynamic cells being
        % assigned to a track they should be not assigned to (too far).
        %
        % Default: 30
        AssignmentThreshold;

        % BirthProbability The probability of new born target in each grid
        % cell.
        % Specify the birth probability as a positive scalar between 0 and
        % 1. The birth probability controls the weights of new particles
        % generated in a grid-cell.
        %
        % Default: 0.01
        BirthProbability;

        % Clustering Specify Clustering as one of the following: 'DBSCAN',
        % 'Custom'. When specified as 'DBSCAN', the clustering of
        % unassigned dynamic grid cells will be done using the DBSCAN
        % algorithm. See clusterDBSCAN for more details. The data used for
        % clustering grid cells is its entire state. You can configure the
        % DBSCAN algorithm by specifying the "ClusteringThreshold" and
        % "MinNumCellsPerCluster" property of the tracker.
        %
        % When specified as 'Custom', the CustomClusteringFcn property of
        % the tracker must be specified.
        Clustering;

        % Epsilon Threshold for a neighborhood search query for DBSCAN.
        % Specify Epsilon as positive scalar or a n-element array, where n
        % is the number of state variables. It defines a radius around a
        % core point. When specified as an array, the ith element of
        % Epsilon determines the threshold for ith state. This property is
        % only active when Clustering is set to 'DBSCAN'
        %
        % Default: 5
        ClusteringThreshold;

        % ConfirmationThreshold Specify the confirmation threshold as [M
        % N], where a track will be confirmed if it receives at least M out
        % of N updates.
        %
        % Default: [2 3]
        ConfirmationThreshold;

        % CustomClusteringFcn Custom function for performing clustering of
        % unassigned grid cells.
        % Specify CustomClusteringFcn as a function_handle of an character
        % vector or a string which defines the name of the function. The
        % function must support the following signature:
        %
        % function idx = CustomClusteringFcn(dynamicGridCells)
        %
        % dynamicGridCells is a struct containing information about dynamic
        % grid cells which need to be clustered. The struct has the
        % following fields:
        %
        % Width             - Width of the cell
        % GridIndices       - Indices of the grid cells an N-by-2 array,
        %                     where N is the number of unassigned cells
        % State             - States of the grid cells as P-by-N array where P
        %                     is the number of state variables
        % StateCovariance   - Uncertainty in state-estimate of each cell
        %                     as a P-by-P-by-N array
        % OccupancyMass     - Occupancy belief mass of the cells as N-element array
        % FreeMass          - Free belief mass of the cells as N-element array
        %
        % idx must be a N-element array defining a the cluster index for
        % each dynamic grid cell.
        CustomClusteringFcn;

        % DeathRate The rate of a target death per unit time.
        %
        % Specify the death rate as a positive scalar value between 0 and
        % 1. The death rate translates to a probability of survival per
        % step by using the following relation:
        % Ps = (1 - DeathRate)^dT
        % where dT is the time difference between current and last update.
        % Ps is the probability of survival. 
        %
        % This is a tunable property and its value can be changed after the
        % tracker is stepped.
        %
        % Default: 1e-3
        DeathRate;

        % DeletionThreshold Specify the deletion threshold as [P R], where a
        % track will be deleted if in the last R updates, at least P
        % times it was not assigned to any dynamic grid cell
        %
        % Default: [5 5]
        DeletionThreshold;

        % FreeSpaceDiscountFactor Confidence in free-space prediction per
        % unit time. Specify the confidence in prediction of belief mass
        % for free state.
        %
        % In the prediction stage, the belief mass for a cell to be in the
        % "Free" state, is reduced by FreeSpaceDiscountFactor
        % according to the following equation
        %
        % mk|k-1(F) = (FreeSpaceDiscountFactor)^dT*mk(F)
        %
        % This is a tunable property and its value can be changed after the
        % tracker is stepped.
        %
        % Default: 0.8
        FreeSpaceDiscountFactor;

        % GridLength Dimension of the grid in x-direction of the local
        % coordinate.
        % Specify the length as a positive scalar value describing
        % the length of the 2-D grid.
        %
        % Default: 100
        GridLength;

        %  GridOriginInLocal Location of the grid in local coordinates
        %  A vector defining the [X Y] location of the bottom-left
        %  corner of the grid, relative to the local frame.
        % Default: [-50 50]
        GridOriginInLocal;

        % GridResolution Resolution of the grid.
        % Specify the resolution of the grid as a positive scalar
        % describing number of cells per meter of the grid in both x and y
        % direction.
        %
        % Default: 1
        GridResolution;

        % GridWidth Dimension of the grid in y-direction of the local
        % coordinate.
        % Specify the width as a positive scalar value describing
        % the width of the 2-D grid.
        %
        % Default: 100
        GridWidth;

        % HasAdditiveProcessNoise A flag indicating if the process noise in
        % state transition is of additive nature.
        %
        % Default: false
        HasAdditiveProcessNoise;

        % HasSensorConfigurationsInput Update sensor configurations with
        % time
        %   Set this property to true if you want to update the
        %   configurations of the sensor with time. When this flag is set
        %   to true, the tracker must be called with the configuration
        %   input as follows: ... = step(tracker,sensorData,config,time)
        %
        %   Default: false
        HasSensorConfigurationsInput;

        % MaxNumSensors Maximum number of sensors that are attached to the
        % autonomous system.
        %
        % Default: 20
        MaxNumSensors;

        % MaxNumTracks   Maximum number of tracks
        %   Set the maximum number of tracks the tracker can maintain as a
        %   positive real integer.
        MaxNumTracks;

        % MinNumPoints Minimum number of required points in a cluster
        % Specify MinNumPoints as a positive integer used as a threshold to
        % determine whether a point is a core point in DBSCAN algorithm.
        % This property is only active when Clustering is set to 'DBSCAN'
        %
        % Default: 2
        MinNumCellsPerCluster;

        % MotionModel Define the motion model for the particles and the
        % tracks as of the following:
        % "constant-velocity", "constant-acceleration" or "Constant
        % turn-rate"
        % The definition of state-spaces for each of the model is described
        % in the table below.
        %
        % | MotionModel             | Particle state    | Object state
        % -----------------------------------------------------------------
        % | "constant-velocity"     | [x;vx;y;vy]       | [x;vx;y;vy;yaw;L;W]
        % | "Constant acceleration" | [x;vx;ax;y;vy;ay] | [x;vx;ax;y;vy;ay;yaw;L;W]
        % | "constant-turnrate"     | [x;vx;y;vy;w]     | [x;vx;y;vy;w;yaw;L;W]
        % where the variables have the following definition:
        %
        % x     - position in x direction of the tracking frame (m)
        % y     - position in y direction of the tracking frame (m)
        % vx    - velocity in x direction of the tracking frame (m/s)
        % vy    - velocity in y direction of the tracking frame (m/s)
        % ax    - acceleration in x direction of the tracking frame (m/s^2)
        % ay    - acceleration in y direction of the tracking frame (m/s^2)
        % w     - yaw-rate in degrees/s in the tracking frame (deg/s)
        % yaw   - yaw angle of the object in tracking frame (deg)
        % L     - Length of the object (m)
        % W     - Width of the object (m)
        MotionModel;

        % NumBirthParticles Number of new born particles per step.
        % Specify number of new particles initialized on the grid per step.
        % The location of these particles is determined by the tracker
        % using the mismatch between predicted and updated occupancy belief
        % masses as well as the BirthProbability. A reasonable value is
        % approximately 10 percent of the persistent particles.
        %
        % Default: 10000
        NumBirthParticles;

        % NumConfirmed The total number of confirmed tracks the
        % trackerGridRFS is maintaining. This is a read-only property.
        NumConfirmedTracks;

        % NumParticles Number of persistent particles on the grid
        % Specify number of particles as positive scalar. Higher number of
        % particles result in better estimation with an increased
        % computational cost.
        %
        % Default: 100000
        NumParticles;

        % NumTracks The total number of tracks the trackerGridRFS is
        % maintaining. This is a read-only property.
        NumTracks;

        % ObjectProcessNoise Process noise for the object state
        ObjectProcessNoise;

        % ProcessNoise Process noise covariance matrix. The process noise
        % defines the process noise for particles and objects. When adding
        % process noise to particles, samples are drawn by assuming
        % Gaussian noise distribution.
        %
        % For using ProcessNoise for the objects, the tracker uses an
        % Extended Kalman Filter approach to predict the tracks to a future
        % time.
        %
        % With non-additive noise, the process define the covariance
        % according to the convention of the motion model
        % | Motion Model          | Number of terms | Convention of terms
        % -----------------------------------------------------
        % | constant-velocity     | 2               | acceleration in x, y
        % | constant-acceleration | 2               | jerk in x, y
        % | constant-turnrate     | 3               | acceleration in x, y and angular acceleration
        %
        % With additive noise, the process defines the covariance with the
        % same size of terms as the state. The process noise for object
        % states, yaw, length and width are assumed zero
        %
        % This is a tunable property and its value can be changed after the
        % tracker is stepped. 
        %
        % Default: eye(n), where n is the number of terms
        ProcessNoise;

        % SensorConfigurations Current configurations of the sensor.
        %   Specify the configurations of the sensor used with the tracker
        %   as trackingSensorConfigurations. You can specify the
        %   SensorConfigurations during construction as a Name,value pair
        %   or set it after construction. There are no default values for
        %   the SensorConfigurations. You must specify them before using
        %   the tracker. When specifying the configurations, the following
        %   properties must be provided with conventions defined below.
        %
        % SensorIndex               - Unique identifier of the sensor
        %
        % IsValidTime               - A flag indicating if the sensor
        %                             should be used to update the tracks
        %
        % SensorTransformParameters - A p-element array of measurement
        %                             parameters with the same fields
        %                             allowed by measurement models like
        %                             cvmeas, cameas. The first measurement
        %                             parameter must describe the
        %                             transformation of sensor with respect
        %                             to the autonomous system. Subsequent
        %                             transformation describe the
        %                             transformation from autonomous system
        %                             to the tracking coordinate frame. If
        %                             only one transform is provided, it is
        %                             assumed that tracking is performed in
        %                             autonomous system's coordinate frame
        %
        % SensorLimits              - A 2-by-2 matrix where first row
        %                             describes the azimuth limits and the
        %                             second row describes the range limits
        %
        %   Default: {}
        SensorConfigurations;

        % StateLimits Calculated using Velocity, Acceleration and
        % TurnRateLimits.
        %
        % |Motion model           | StateLimits Convention  |
        % ---------------------------------------------------
        % 'constant-velocity'     | [minVx maxVx;minVy maxVy];
        % 'constant-acceleration' | [minVx maxVx;minVy maxVy;minAx maxAx;minAy maxAy]
        % 'constant-turnrate     | [minVx maxVx;minVy maxVy;minw maxw];
        %
        %  where the variables have the following definition
        % minVx, maxVx - minimum and maximum velocity of targets in
        %                x-direction of tracking coordinate system (m/s)
        % minAx, maxAx - minimum and maximum acceleration of targets in
        %                x-direction of tracking coordinate system (m/s^2)
        % minVy, maxVy - minimum and maximum velocity of targets in
        %                y-direction of tracking coordinate system (m/s)
        % minAy, maxAy - minimum and maximum acceleration of targets in
        %                y-direction of tracking coordinate system (m/s^2)
        % minw, maxw   - minimum and maximum turn-rate of targets in the
        %                tracking coordinate system (deg/s)
        %
        % Defaults: The default value for each motion model is shown below.
        %
        % Model: constant-velocity,     Default: [-30 30;-30 30];
        % Model: constant-acceleration, Default: [-30 30;-30 30;-5 5;-5 5];
        % Model: constant-turnrate,    Default: [-30 30;-30 30;-5 5];
        StateLimits;

        %StateParameters  Parameters used for track state frame transform
        %   Specify the parameters used for track frame transformation from
        %   source (this tracker) frame to a fuser frame.
        %   This property is tunable.
        %
        % Default: struct
        StateParameters;

        % TrackDistanceFcn A function which defines the distance between
        % each cell of the dynamic grid map and a track. Specify the
        % property as a function handle or a character vector or a string.
        % The function must support the following signature:
        %
        % function d = TrackDistanceFcn(dynamicMap, predictedTrack)
        %
        % dynamicMap is the current estimated dynamic map, specified as a
        % dynamicEvidentialGridMap.
        %
        % predictedTrack is a object of objectTrack class defining a
        % predicted track of an object.
        %
        % d must be a matrix of the same size as the GridSize. d(i,j)
        % represents the distance of i,j cell from the track.
        %
        % The AssignmentThreshold property of the tracker uses this
        % distance for gating.
        %
        % Default: 'trackerGridRFS.defaultTrackDistanceFcn'
        % The default distance function uses negative log-likelihood of
        % occupancy and velocity of a cell against the track's estimate.
        TrackDistanceFcn;

        % TrackInitializationFcn A function to initiate a track from a set
        % of dynamic grid cells. Specify the property as a function handle
        % or a character vector or a string. The function must support the
        % following signature:
        %
        % function track = TrackInitializationFcn(dynamicGridCells)
        %
        % track must be an object of type objectTrack or a struct with
        % similar fields. The size of the state must follow the convention
        % for the object state mentioned in the MotionModels property.
        %
        % dynamicGridCells is a struct defining a set of grid cells
        % initializing this object track. It has the following fields:
        %
        % Width             - Width of the cell
        % GridIndices       - Indices of the grid cells an N-by-2 array,
        %                     where N is the number of unassigned cells
        % State             - States of the grid cells as P-by-N array where P
        %                     is the number of state variables
        % StateCovariance   - Uncertainty in state-estimate of each cell
        %                     as a P-by-P-by-N array
        % OccupancyMass     - Occupancy belief mass of the cells as N-element array
        % FreeMass          - Free belief mass of the cells as N-element array
        %
        % Default: 'trackerGridRFS.defaultTrackInitialization'
        %
        % The default TrackInitializationFcn merges the Gaussian estimate
        % from each cell to describe the state of the object. The
        % orientation of the object is defined in the same direction as
        % it's mean velocity. With a defined orientation, length and width
        % of the object are extracted using the geometric properties of the
        % cells. The uncertainty in length, width and orientation estimate
        % is calculated using linear approximations.
        TrackInitializationFcn;

        % TrackUpdateFcn A function to update a track with its associated
        % set of dynamic grid cells. Specify the property as a function
        % handle or a character vector or a string. The function must
        % support the following signature:
        %
        % function updateTrack = TrackUpdateFcn(predictedTrack, dynamicGridCells)
        %
        % predictedTrack is the predicted track of an object provided as an
        % objectTrack
        %
        % dynamicGridCells is a struct defining set of dynamic grid cells
        % associated to the track. It has the same fields as defined in the
        % TrackInitializationFcn property description.
        %
        % updatedTrack must return the updated track of the object as a
        % struct or an objectTrack object
        %
        % The default TrackUpdateFcn estimates Gaussian state approximation
        % from the dynamic grid cells similar to the
        % TrackInitializationFcn and overwrites the state of the track with
        % the estimate from grid cells.
        %
        % Default: 'trackerGridRFS.defaultTrackUpdate'
        TrackUpdateFcn;

        %TrackerIndex Unique identifier of the tracker
        %   Specify the unique index associated with this tracker in a
        %   decentralized tracking architecture. This index is used as the
        %   SourceIndex in the tracks output, and serves in track-to-track
        %   fusion. You must define this property to a positive value to
        %   use the track outputs as inputs to a track fuser.
        %
        % Default: uint32(0)
        TrackerIndex;

        % TurnRateLimits Specify the minimum and maximum of turn-rate
        % (deg/s) in the tracking coordinate frame as a 2 element vector.
        % The first element defines the minimum turn-rate and the second
        % element defines the maximum turn-rate. This property is only
        % active when MotionModel is set to 'constant-turnrate'
        %
        % Default: [-5 5]
        TurnRateLimits;

        % UseGPU Specify if the estimation of dynamic occupancy map is
        % performed on GPU. Enabling GPU computation requires the Parallel
        % Computing Toolbox(TM).
        % 
        % Default: false
        UseGPU;

        % VelocityLimits Specify the minimum and maximum of velocity (m/s)
        % in tracking coordinate frames as a 2-by-2 matrix. The first row
        % corresponds to the velocity limits in X direction and second row
        % corresponds to the velocity limits in Y direction.
        %
        % Default: [-10 10;-10 10]
        VelocityLimits;

        pClassToUse;

        %pConstantTimeTol  A tolerance value for time interval check
        %   Defines a time tolerance that is used when comparing the
        %   sensor data timestamps with the time interval expected by the
        %   tracker. 
        %
        %   NOTES: the value of the tolerance should be:
        %     1. Large enough to avoid issues in floating point comparison
        %        (e.g., 1.00000001 > 1).
        %     2. Small enough so that no sensor detection in the next
        %        time frame can have a timestamp less than this tolerance
        %        value after the end of the current time frame.
        pConstantTimetol;

    end
end
