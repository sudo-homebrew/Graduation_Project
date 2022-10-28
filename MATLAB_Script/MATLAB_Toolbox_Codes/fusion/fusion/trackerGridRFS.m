classdef trackerGridRFS  < matlabshared.tracking.internal.fusion.AbstractTracker
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
    %   reset                   - Resets states of the trackerGridRFS
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
    
    %#codegen
    properties (Dependent)
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
        SensorConfigurations
    end
    
    properties (Dependent)
        % HasSensorConfigurationsInput Update sensor configurations with
        % time
        %   Set this property to true if you want to update the
        %   configurations of the sensor with time. When this flag is set
        %   to true, the tracker must be called with the configuration
        %   input as follows: ... = step(tracker,sensorData,config,time)
        %
        %   Default: false
        HasSensorConfigurationsInput;
    end
    
    properties (Dependent)
        % MaxNumSensors Maximum number of sensors that are attached to the
        % autonomous system.
        %
        % Default: 20
        MaxNumSensors;
    end
    
    
    % Grid properties
    properties (Dependent)
        % GridLength Dimension of the grid in x-direction of the local
        % coordinate.
        % Specify the length as a positive scalar value describing
        % the length of the 2-D grid.
        %
        % Default: 100
        GridLength;
        
        % GridWidth Dimension of the grid in y-direction of the local
        % coordinate.
        % Specify the width as a positive scalar value describing
        % the width of the 2-D grid.
        %
        % Default: 100
        GridWidth;
        
        % GridResolution Resolution of the grid.
        % Specify the resolution of the grid as a positive scalar
        % describing number of cells per meter of the grid in both x and y
        % direction.
        %
        % Default: 1
        GridResolution;
        
        %  GridOriginInLocal Location of the grid in local coordinates
        %  A vector defining the [X Y] location of the bottom-left
        %  corner of the grid, relative to the local frame.
        % Default: [-50 50]
        GridOriginInLocal;
    end
    
    
    properties (Nontunable)
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
        MotionModel = 'constant-velocity';
    end
    
    properties (Hidden, Constant)
        MotionModelSet = matlab.system.StringSet({'constant-velocity','constant-acceleration','constant-turnrate'});
        ClusteringSet = matlab.system.StringSet({'DBSCAN','Custom'});
    end
    
    % Particle filtering (map estimator) properties
    properties (Nontunable)
        % NumParticles Number of persistent particles on the grid
        % Specify number of particles as positive scalar. Higher number of
        % particles result in better estimation with an increased
        % computational cost.
        %
        % Default: 100000
        NumParticles = 1e5
        
        % NumBirthParticles Number of new born particles per step.
        % Specify number of new particles initialized on the grid per step.
        % The location of these particles is determined by the tracker
        % using the mismatch between predicted and updated occupancy belief
        % masses as well as the BirthProbability. A reasonable value is
        % approximately 10 percent of the persistent particles.
        %
        % Default: 10000
        NumBirthParticles = 1e4;
        
        % BirthProbability The probability of new born target in each grid
        % cell.
        % Specify the birth probability as a positive scalar between 0 and
        % 1. The birth probability controls the weights of new particles
        % generated in a grid-cell.
        %
        % Default: 0.01
        BirthProbability = 1e-2;    
    end
    
    properties
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
        DeathRate = 1e-3;
        
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
        FreeSpaceDiscountFactor = 0.8;
    end
    
    properties (Nontunable)
        % VelocityLimits Specify the minimum and maximum of velocity (m/s)
        % in tracking coordinate frames as a 2-by-2 matrix. The first row
        % corresponds to the velocity limits in X direction and second row
        % corresponds to the velocity limits in Y direction.
        %
        % Default: [-10 10;-10 10]
        VelocityLimits = [-10 10;-10 10];
        
        % AccelerationLimits Specify the minimum and maximum of
        % acceleration (m/s^2) in tracking coordinate frame as a 2-by-2
        % matrix. The first row corresponds to the acceleration limits in X
        % direction and second row corresponds to the acceleration limits
        % in Y direction. This property is only active when MotionModel is
        % set to 'constant-acceleration'
        %
        % Default: [-5 5;-5 5]
        AccelerationLimits = [-5 5;-5 5];
        
        % TurnRateLimits Specify the minimum and maximum of turn-rate
        % (deg/s) in the tracking coordinate frame as a 2 element vector.
        % The first element defines the minimum turn-rate and the second
        % element defines the maximum turn-rate. This property is only
        % active when MotionModel is set to 'constant-turnrate'
        %
        % Default: [-5 5]
        TurnRateLimits = [-5 5];
    end
    
    properties (Dependent, Access = protected)
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
        StateLimits
    end
    
    properties
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
        ProcessNoise = eye(2)
    end
    
    properties (Nontunable)
        % HasAdditiveProcessNoise A flag indicating if the process noise in
        % state transition is of additive nature.
        %
        % Default: false
        HasAdditiveProcessNoise (1, 1) logical = false;
    end
    
    properties (Access = protected, Dependent)
        % ObjectProcessNoise Process noise for the object state
        ObjectProcessNoise
    end
    
    properties (Nontunable, Access = protected)
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
        TrackDistanceFcn = 'trackerGridRFS.defaultTrackDistance'
    end
    
    % Object Extractor properties
    properties (Nontunable)
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
        TrackInitializationFcn = 'trackerGridRFS.defaultTrackInitialization'
        
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
        TrackUpdateFcn = 'trackerGridRFS.defaultTrackUpdate';
    end
    
    properties (Nontunable)
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
        Clustering = 'DBSCAN';
        
        % Epsilon Threshold for a neighborhood search query for DBSCAN.
        % Specify Epsilon as positive scalar or a n-element array, where n
        % is the number of state variables. It defines a radius around a
        % core point. When specified as an array, the ith element of
        % Epsilon determines the threshold for ith state. This property is
        % only active when Clustering is set to 'DBSCAN'
        %
        % Default: 5
        ClusteringThreshold = 5;
        
        % MinNumPoints Minimum number of required points in a cluster
        % Specify MinNumPoints as a positive integer used as a threshold to
        % determine whether a point is a core point in DBSCAN algorithm.
        % This property is only active when Clustering is set to 'DBSCAN'
        %
        % Default: 2
        MinNumCellsPerCluster = 2;
        
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
        CustomClusteringFcn = ''
    end
    
    % Track logics
    properties (Nontunable)
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
        AssignmentThreshold = 30
        
        % ConfirmationThreshold Specify the confirmation threshold as [M
        % N], where a track will be confirmed if it receives at least M out
        % of N updates.
        %
        % Default: [2 3]
        ConfirmationThreshold = [2 3]
        
        % DeletionThreshold Specify the deletion threshold as [P R], where a
        % track will be deleted if in the last R updates, at least P
        % times it was not assigned to any dynamic grid cell
        %
        % Default: [5 5]
        DeletionThreshold = [5 5]
    end
    
    % The standard tracker properties of a tracker
    properties (Nontunable)
        %TrackerIndex Unique identifier of the tracker
        %   Specify the unique index associated with this tracker in a
        %   decentralized tracking architecture. This index is used as the
        %   SourceIndex in the tracks output, and serves in track-to-track
        %   fusion. You must define this property to a positive value to
        %   use the track outputs as inputs to a track fuser.
        %
        % Default: uint32(0)
        TrackerIndex = uint32(0);
        
        % MaxNumTracks   Maximum number of tracks
        %   Set the maximum number of tracks the tracker can maintain as a
        %   positive real integer.
        MaxNumTracks = 100;
    end
    
    properties (Dependent)
        %StateParameters  Parameters used for track state frame transform
        %   Specify the parameters used for track frame transformation from
        %   source (this tracker) frame to a fuser frame.
        %   This property is tunable.
        %
        % Default: struct
        StateParameters
    end
    
    properties (Nontunable)
        % UseGPU Specify if the estimation of dynamic occupancy map is
        % performed on GPU. Enabling GPU computation requires the Parallel
        % Computing Toolbox(TM).
        % 
        % Default: false
        UseGPU (1, 1) logical = false;
    end
    
    properties (Hidden, Dependent)
        HasOcclusion;
    end
    
    % Depends on object extractor
    properties (SetAccess = protected, Dependent)
        % NumTracks The total number of tracks the trackerGridRFS is
        % maintaining. This is a read-only property.
        NumTracks
        % NumConfirmed The total number of confirmed tracks the
        % trackerGridRFS is maintaining. This is a read-only property.
        NumConfirmedTracks
    end
    
    properties(Constant, Access = protected)
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
        pConstantTimetol = 1e-5
    end
    
    % The real stuff
    properties (Access = {?matlab.unittest.TestCase,?trackerGridRFS})
        pStateParameters = struct;
        pClusterer
        pMeasurementProjector
        pDynamicMapEstimator
        pObjectExtractor
        pLastTime
        pIsFirstCall = true;
        pLastKnownFilterState;
    end
    
    properties (Nontunable, Access = protected)
        pClassToUse
    end    
    
    methods
        function obj = trackerGridRFS(varargin)
            % Create the projector
            createMeasurementProjector(obj);
            
            % Set properties
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    %% Get/set methods
    methods
        function val = get.SensorConfigurations(obj)
            val = obj.pMeasurementProjector.SensorConfigurations;
        end
        
        function set.SensorConfigurations(obj, val)
            if isstruct(val)
                value = cell(numel(val),1);
                for i = 1:numel(val)
                    sensorConfig = fusion.internal.getConfigurationsFromStruct(val(i));
                    value{i} = sensorConfig{1};
                end
            else
                value = val;
            end
            obj.pMeasurementProjector.SensorConfigurations = value;
        end
        
        function val = get.HasSensorConfigurationsInput(obj)
            val = obj.pMeasurementProjector.HasSensorConfigurationsInput;
        end
        
        function set.HasSensorConfigurationsInput(obj,val)
            obj.pMeasurementProjector.HasSensorConfigurationsInput = val;
        end
        
        function val = get.HasOcclusion(obj)
            val = obj.pMeasurementProjector.HasOcclusion;
        end
        
        function set.HasOcclusion(obj, val)
            obj.pMeasurementProjector.HasOcclusion = val;
        end
        
        function val = get.MaxNumSensors(obj)
            val = obj.pMeasurementProjector.MaxNumSensors;
        end
        
        function set.MaxNumSensors(obj, val)
            validateattributes(val,{'numeric'},{'scalar','positive','real','finite','nonsparse'},mfilename,'MaxNumSensors');
            obj.pMeasurementProjector.MaxNumSensors = val;
        end
        
        function set.GridLength(obj, val)
            validateattributes(val,{'numeric'},{'scalar','positive','real','finite','nonsparse'},mfilename,'GridLength');
            obj.pMeasurementProjector.GridLength = val;
        end
        
        function set.GridWidth(obj, val)
            validateattributes(val,{'numeric'},{'scalar','positive','real','finite','nonsparse'},mfilename,'GridWidth');
            obj.pMeasurementProjector.GridWidth = val;
        end
        
        function set.GridResolution(obj, val)
            validateattributes(val,{'numeric'},{'scalar','positive','real','finite','nonsparse'},mfilename,'GridResolution');
            obj.pMeasurementProjector.GridResolution = val;
        end
        
        function set.GridOriginInLocal(obj, val)
            validateattributes(val,{'numeric'},{'real','finite','nonsparse','numel',2},mfilename,'GridLength');
            obj.pMeasurementProjector.GridOriginInLocal = val(:)';
        end
        
        function val = get.GridLength(obj)
            val = obj.pMeasurementProjector.GridLength;
        end
        
        function val = get.GridWidth(obj)
            val = obj.pMeasurementProjector.GridWidth;
        end
        
        function val = get.GridResolution(obj)
            val = obj.pMeasurementProjector.GridResolution;
        end
        
        function val = get.GridOriginInLocal(obj)
            val = obj.pMeasurementProjector.GridOriginInLocal;
        end
        
        function val = get.NumTracks(obj)
            if coder.internal.is_defined(obj.pObjectExtractor)
                val = obj.pObjectExtractor.NumTracks;
            else
                val = 0;
            end
        end
        
        function val = get.NumConfirmedTracks(obj)
            if coder.internal.is_defined(obj.pObjectExtractor)
                val = obj.pObjectExtractor.NumConfirmedTracks;
            else
                val = 0;
            end
        end
        
        function set.TrackerIndex(obj, val)
            validateattributes(val,{'numeric'},{'scalar','integer'},mfilename,'TrackerIndex');
            obj.TrackerIndex = uint32(val);
        end
        
        function set.StateParameters(obj, val)
            setStateParameters(obj,val);
        end
        
        function val = get.StateParameters(obj)
            val = obj.pStateParameters;
        end
        
        function set.VelocityLimits(obj,val)
            validateattributes(val,{'single','double'},{'real','nonsparse','nonnan','size',[2 2]},mfilename,'VelocityLimits');
            validateattributes(val',{'single','double'},{'increasing'},mfilename,'VelocityLimits');
            obj.VelocityLimits = val;
        end
        
        function set.AccelerationLimits(obj,val)
            validateattributes(val,{'single','double'},{'real','nonsparse','nonnan','size',[2 2]},mfilename,'AccelerationLimits');
            validateattributes(val',{'single','double'},{'increasing'},mfilename,'AccelerationLimits');
            obj.AccelerationLimits = val;
        end
        
        function set.TurnRateLimits(obj, val)
            validateattributes(val,{'single','double'},{'real','nonsparse','nonnan','increasing','numel',2},mfilename,'TurnRateLimits');
            obj.TurnRateLimits = val(:)';
        end
        
        function set.NumParticles(obj, val)
            validateattributes(val,{'single','double'},{'real','finite','integer','scalar'},mfilename,'NumParticles');
            obj.NumParticles = val;
        end
        
        function set.NumBirthParticles(obj, val)
            validateattributes(val,{'single','double'},{'real','finite','integer','scalar'},mfilename,'NumBirthParticles');
            obj.NumBirthParticles = val;
        end
        
        function set.BirthProbability(obj, val)
            validateattributes(val,{'single','double'},{'real','finite','nonnan','nonsparse','scalar','>=',0,'<',1},mfilename,'BirthProbability');
            obj.BirthProbability = val;
        end
        
        function set.DeathRate(obj, val)
            validateattributes(val,{'single','double'},{'real','finite','nonnan','nonsparse','scalar','<',1},mfilename,'DeathRate');
            obj.DeathRate = val;
            if coder.target('MATLAB')
                setEstimatorProperty(obj, 'DeathRate', val);
            else
                coder.internal.defer_inference('setEstimatorProperty',obj,'DeathRate',val);
            end
        end
        
        function set.FreeSpaceDiscountFactor(obj, val)
            validateattributes(val,{'single','double'},{'real','finite','nonnan','nonsparse','scalar','<=',1},mfilename,'FreeSpaceDiscountFactor');
            obj.FreeSpaceDiscountFactor = val;
            if coder.target('MATLAB')
                setEstimatorProperty(obj, 'FreeSpaceDiscountFactor', val);
            else
                coder.internal.defer_inference('setEstimatorProperty',obj,'FreeSpaceDiscountFactor',val);
            end
        end
        
        function set.ClusteringThreshold(obj, val)
            validateattributes(val,{'single','double'},{'real','finite','nonnan','nonsparse','scalar'},mfilename,'ClusteringThreshold');
            obj.ClusteringThreshold = val;
        end
        
        function set.MinNumCellsPerCluster(obj, val)
            validateattributes(val,{'single','double'},{'real','scalar','integer'},mfilename,'MinNumCellsPerCluster');
            obj.MinNumCellsPerCluster = val;
        end
        
        function set.AssignmentThreshold(obj, val)
            validateattributes(val,{'single','double'},{'real','finite','nonnan','nonsparse','scalar'},mfilename,'AssignmentThreshold');
            obj.AssignmentThreshold = val;
        end
        
        function set.ConfirmationThreshold(obj, val)
            validateattributes(val,{'single','double'},{'real','finite','nondecreasing','integer'},mfilename,'ConfirmationThreshold');
            obj.ConfirmationThreshold = val;
        end
        
        function set.DeletionThreshold(obj, val)
            validateattributes(val,{'single','double'},{'real','finite','nondecreasing','integer'},mfilename,'DeletionThreshold');
            obj.DeletionThreshold = val;
        end
        
        function set.ProcessNoise(obj, val)
            % If process noise is defined and verified, add a check against
            % input size
            matlabshared.tracking.internal.isSymmetricPositiveSemiDefinite('ProcessNoise',val);
            obj.ProcessNoise = val;
            if coder.target('MATLAB')
                setAndValidateProcessNoise(obj);
            else
                coder.internal.defer_inference('setAndValidateProcessNoise',obj);
            end
        end
        
        % Process noise for objects.
        function val = get.ObjectProcessNoise(obj)
            % In additive case, the process noise for theta, L and W is
            % assumed to be zero.
            if obj.HasAdditiveProcessNoise
                val = blkdiag(obj.ProcessNoise,zeros(3,class(obj.ProcessNoise)));
            else
                % Non-additive is same as point. Jacobians are calculated to
                % use zero process noise for theta, L and W.
                val = obj.ProcessNoise;
            end
        end
    end
    
    methods (Access = protected)
        function setEstimatorProperty(obj, propName, propVal)
            if isLocked(obj) && coder.internal.is_defined(obj.pDynamicMapEstimator)
                obj.pDynamicMapEstimator.(propName) = propVal;
            end
        end
        
        function setAndValidateProcessNoise(obj)
            if isLocked(obj) && coder.internal.is_defined(obj.pObjectExtractor)
                validateProcessNoiseSize(obj);
                obj.pObjectExtractor.ProcessNoise = obj.ObjectProcessNoise;
                obj.pDynamicMapEstimator.ProcessNoise = obj.ProcessNoise;
            end
        end
        
        function setExtractorStateParameters(obj,val)
            if isLocked(obj) && coder.internal.is_defined(obj.pObjectExtractor)
                obj.pObjectExtractor.StateParameters = val;
            end
        end
    end
    
    methods (Access = protected)
        function setupImpl(obj, varargin)
            % Get class from sensor data
            sensorData = varargin{1};
            
            % Define class to use
            if ~coder.internal.is_defined(obj.pClassToUse)
                coder.internal.assert(numel(sensorData) > 0,'fusion:GridTracker:UndefinedSampleData');
                obj.pClassToUse = class(sensorData(1).Measurement);
            end
            
            % Setup projector
            measMap = setupProjector(obj, varargin{:});
            
            % Setup filter
            createDynamicMapEstimator(obj);
            initialize(obj.pDynamicMapEstimator,measMap);
            
            % Setup object extractor
            createObjectExtractor(obj, varargin{:});
            
            % Set last time
            obj.pLastTime = cast(-eps, obj.pClassToUse);
            
            % Capture state of the filter
            obj.pLastKnownFilterState = captureState(obj.pDynamicMapEstimator);
        end
        
        function measMap = setupProjector(obj, varargin)
            % All properties are already synced. set UseGPU and ClassToUse.
            % The RFS filter is initialized in setup, so this projector
            % needs to be setup right now
            obj.pMeasurementProjector.UseGPU = obj.UseGPU;
            obj.pMeasurementProjector.ClassToUse = obj.pClassToUse;
            
            setup(obj.pMeasurementProjector,varargin{:});
            
            measMap = obj.pMeasurementProjector.pMeasurementEvidenceMap;
        end
    end
    
    methods
        function predictedTracks = predictTracksToTime(obj,track,time,varargin)
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
            
            % Check that the tracker has locked
            coder.internal.assert(isLocked(obj),'fusion:GridTracker:PredictBeforeUpdate')
            narginchk(3,5);
            
            validateattributes(time, {'numeric'}, ...
                {'real', 'finite', 'nonsparse', 'scalar', '>', obj.pLastTime}, ...
                'predictTracksToTime', 'time');
            
            if ischar(track) || isstring(track)
                type = validatestring(track,{'all','confirmed','tentative'},...
                    'predictTracksToTime','category');
                % All is -1, confirmed is -2 and tentative is -3
                id = -(find(strcmpi(type,{'all','confirmed','tentative'})));
            else
                validateattributes(track, {'numeric'}, ...
                    {'real', 'positive', 'scalar', 'integer'}, ...
                    'predictTracksToTime', 'trackID');
                id = track;
            end
            
            withCovariance = false;
            if nargin > 3
                validateattributes(varargin{2},{'logical'},{'scalar'},'trackerPHD');
                withCovariance = varargin{2};
            end
            
            dT = time - obj.pLastTime;
            predictedTracks = obj.pObjectExtractor.predictTracksToTime(dT,id,withCovariance);
        end
        
        function showDynamicMap(tracker, varargin)
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
            coder.internal.assert(isLocked(tracker),'fusion:GridTracker:mustUpdateBeforeCall','showDynamicMap');

            % Get handle to the dynamic map
            dynamicMap = tracker.pDynamicMapEstimator.DynamicMap;
            
            % Divert call to dynamic map
            show(dynamicMap, varargin{:});
        end
        
        function map = predictMapToTime(obj, time, varargin)
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
            
            coder.internal.assert(isLocked(obj),'fusion:GridTracker:mustUpdateBeforeCall','predictMapToTime');
            
            validateattributes(time, {'numeric'}, ...
                {'real', 'finite', 'nonsparse', 'scalar', '>=', obj.pLastTime}, ...
                'predictMapToTime', 'time');
            
            % Sync with last known filter state
            syncWithState(obj.pDynamicMapEstimator,obj.pLastKnownFilterState);
            
            withStateAndCovariance = true;
            if nargin > 2
                narginchk(4,4);
                validatestring(varargin{1},{'WithStateAndCovariance'},'predictMapToTime');
                validateattributes(varargin{2},{'numeric','logical'},{'binary'},'predictMapToTime','WithStateAndCovariance');
                withStateAndCovariance = varargin{2};
            end
            
            % dT value
            dT = time - obj.pLastTime;
            
            % Predict the filter to produce an outputable map.
            map = predictForOutput(obj.pDynamicMapEstimator,dT,withStateAndCovariance);
        end
    end
    
    methods (Access = protected)
        function f = getClusteringFcn(obj)
            % f = getClusteringFcn(obj) captures both DBSCAN and
            % CustomClusteringFcn using a single function handle
            
            if strcmpi(obj.Clustering,'DBSCAN')
                dbscan = fusion.internal.DBSCAN('MinNumPoints',obj.MinNumCellsPerCluster,...
                    'Epsilon',obj.ClusteringThreshold);
                switch obj.MotionModel
                    case 'constant-velocity'
                        idx = [1 3];
                    case 'constant-turnrate'
                        idx = [1 3];
                    case 'constant-acceleration'
                        idx = [1 4];
                end
                f = @(x)clusterCells(dbscan,idx,x);
            else
                f = obj.CustomClusteringFcn;
            end
        end
    end
    
    methods (Access = protected)
        function validatePropertiesImpl(obj)
            validateProcessNoiseSize(obj);
        end
        
        function validateProcessNoiseSize(obj)
            [additive, nonadditive] = getExpectedProcessNoiseSize(obj);
            if obj.HasAdditiveProcessNoise
                coder.internal.assert(size(obj.ProcessNoise,1) == additive,'fusion:GridTracker:IncorrectProcessNoiseSize',additive);
            else
                coder.internal.assert(size(obj.ProcessNoise,1) == nonadditive,'fusion:GridTracker:IncorrectProcessNoiseSize',nonadditive);
            end
        end
        
        function validateInputsImpl(obj, sensorData, varargin)
            validateattributes(sensorData,{'struct'},{},mfilename,'sensorData',1);
            expFields = {'Time','SensorIndex','Measurement','MeasurementParameters'};
            if coder.target('MATLAB')
                isAvail = isfield(sensorData,expFields);
            else
                isAvail = false(numel(expFields),1);
                for i = 1:numel(expFields)
                    isAvail(i) = isfield(sensorData,expFields{i});
                end
            end

            if any(~isAvail)
                idx = find(~isAvail,1,'first');
                missingFields = expFields{idx};
                coder.internal.assert(false,'fusion:GridTracker:InvalidSensorData',missingFields);
            end
            
            if obj.HasSensorConfigurationsInput
                configs = varargin{1};
                validateattributes(configs, {'struct','cell'}, {'vector'},'step','config',2);
                if iscell(configs)
                    coder.internal.assert(isstruct(configs{1}) || isa(configs{1},'fusion.internal.AbstractTrackingSensorConfiguration'),'fusion:GridTracker:invalidConfigType',class(configs{1}));
                else
                    coder.internal.assert(isstruct(configs(1)) && isfield(configs(1),'SensorIndex'),'fusion:GridTracker:invalidConfigType',class(configs(1)));
                end
                validateattributes(varargin{2},{'numeric'},{'scalar','real','finite','nonsparse','nonnegative'},'step','time',4);
            else
                validateattributes(varargin{1},{'numeric'},{'scalar','real','finite','nonsparse','nonnegative'},'step','time',3);
            end
        end
        
        function [ad, nonad] = getExpectedProcessNoiseSize(obj)
            switch obj.MotionModel
                case 'constant-velocity'
                    ad = 4;
                    nonad = 2;
                case 'constant-acceleration'
                    ad = 6;
                    nonad = 2;
                case 'constant-turnrate'
                    ad = 5;
                    nonad = 3;
            end
        end
    end
    
    methods (Access = protected)
        function numIn = getNumInputsImpl(obj)
            numIn = 2 + double(obj.HasSensorConfigurationsInput);
        end
    end
    
    methods (Access = {?trackerGridRFS,?matlab.unittest.TestCase})
        function createMeasurementProjector(obj)
            obj.pMeasurementProjector = fusion.internal.GridSensorDataProjector;
        end
        
        function createDynamicMapEstimator(obj)
            switch obj.MotionModel
                case 'constant-velocity'
                    estimator = initcvestimator(obj);
                case 'constant-acceleration'
                    estimator = initcaestimator(obj);
                case 'constant-turnrate'
                    estimator = initctestimator(obj);
            end
            obj.pDynamicMapEstimator = estimator;
        end
        
        function createObjectExtractor(obj, varargin)
            switch obj.MotionModel
                case 'constant-velocity'
                    objExtractor = initcvextractor(obj);
                case 'constant-acceleration'
                    objExtractor = initcaextractor(obj);
                case 'constant-turnrate'
                    objExtractor = initctextractor(obj);
            end
            obj.pObjectExtractor = objExtractor;
        end
        
        function extractor = initcvextractor(obj)
            extractor = fusion.internal.GridObjectExtractor(...
                'ExtractorIndex',obj.TrackerIndex,...
                'StateParameters',obj.pStateParameters,...
                'MaxNumTracks',obj.MaxNumTracks,...
                'StateTransitionFcn',@fusion.internal.GridMeasurementBuiltins.constvelrect,...
                'StateTransitionJacobianFcn',@fusion.internal.GridMeasurementBuiltins.constvelrectjac,...
                'ProcessNoise',obj.ObjectProcessNoise,...
                'HasAdditiveProcessNoise',obj.HasAdditiveProcessNoise,...
                'TrackInitializationFcn',obj.TrackInitializationFcn,...
                'ConfirmationThreshold',obj.ConfirmationThreshold,...
                'DeletionThreshold',obj.DeletionThreshold,...
                'TrackUpdateFcn',obj.TrackUpdateFcn,...
                'TrackDistanceFcn',obj.TrackDistanceFcn,...
                'AssignmentThreshold',obj.AssignmentThreshold,...
                'ClusteringFcn',getClusteringFcn(obj));
        end
        
        function extractor = initcaextractor(obj)
            extractor = fusion.internal.GridObjectExtractor(...
                'ExtractorIndex',obj.TrackerIndex,...
                'StateParameters',obj.pStateParameters,...
                'MaxNumTracks',obj.MaxNumTracks,...
                'StateTransitionFcn',@fusion.internal.GridMeasurementBuiltins.constaccrect,...
                'StateTransitionJacobianFcn',@fusion.internal.GridMeasurementBuiltins.constaccrectjac,...
                'ProcessNoise',obj.ObjectProcessNoise,...
                'HasAdditiveProcessNoise',obj.HasAdditiveProcessNoise,...
                'TrackInitializationFcn',obj.TrackInitializationFcn,...
                'TrackUpdateFcn',obj.TrackUpdateFcn,...
                'TrackDistanceFcn',obj.TrackDistanceFcn,...
                'AssignmentThreshold',obj.AssignmentThreshold,...
                'ClusteringFcn',getClusteringFcn(obj));
        end
        
        function extractor = initctextractor(obj)
            extractor = fusion.internal.GridObjectExtractor(...
                'ExtractorIndex',obj.TrackerIndex,...
                'StateParameters',obj.pStateParameters,...
                'MaxNumTracks',obj.MaxNumTracks,...
                'StateTransitionFcn',@fusion.internal.GridMeasurementBuiltins.constturnrect,...
                'StateTransitionJacobianFcn',@fusion.internal.GridMeasurementBuiltins.constturnrectjac,...
                'ProcessNoise',obj.ObjectProcessNoise,...
                'HasAdditiveProcessNoise',obj.HasAdditiveProcessNoise,...
                'TrackInitializationFcn',obj.TrackInitializationFcn,...
                'TrackUpdateFcn',obj.TrackUpdateFcn,...
                'TrackDistanceFcn',obj.TrackDistanceFcn,...
                'AssignmentThreshold',obj.AssignmentThreshold,...
                'ClusteringFcn',getClusteringFcn(obj));
        end
        
        function estimator = initcvestimator(obj)
            estimator = fusion.internal.DynamicMapRFSFilter(...
                'UseGPU',obj.UseGPU,...
                'ClassToUse',obj.pClassToUse,...
                'NumParticles',obj.NumParticles,...
                'NumBirthParticles',obj.NumBirthParticles,...
                'NumStateVariables',4,...
                'DeathRate',obj.DeathRate,...
                'BirthProbability',obj.BirthProbability,...
                'FreeSpaceDiscountFactor',obj.FreeSpaceDiscountFactor,...
                'ParticleSamplingFcn',@fusion.internal.GridMeasurementBuiltins.cvParticleSamplingFcn,...
                'ParticlePositionFcn',@fusion.internal.GridMeasurementBuiltins.cvParticlePositionFcn,...
                'StateTransitionFcn',@constvel,...
                'StateLimits',obj.StateLimits,...
                'HasAdditiveProcessNoise',obj.HasAdditiveProcessNoise,...
                'ProcessNoise',obj.ProcessNoise);
        end
        
        function estimator = initcaestimator(obj)
            estimator = fusion.internal.DynamicMapRFSFilter(...
                'UseGPU',obj.UseGPU,...
                'ClassToUse',obj.pClassToUse,...
                'NumParticles',obj.NumParticles,...
                'NumBirthParticles',obj.NumBirthParticles,...
                'NumStateVariables',6,...
                'DeathRate',obj.DeathRate,...
                'BirthProbability',obj.BirthProbability,...
                'FreeSpaceDiscountFactor',obj.FreeSpaceDiscountFactor,...
                'ParticleSamplingFcn',@fusion.internal.GridMeasurementBuiltins.caParticleSamplingFcn,...
                'ParticlePositionFcn',@fusion.internal.GridMeasurementBuiltins.caParticlePositionFcn,...
                'StateTransitionFcn',@constacc,...
                'StateLimits',obj.StateLimits,...
                'HasAdditiveProcessNoise',obj.HasAdditiveProcessNoise,...
                'ProcessNoise',obj.ProcessNoise);
        end
        
        function estimator = initctestimator(obj)
            estimator = fusion.internal.DynamicMapRFSFilter(...
                'UseGPU',obj.UseGPU,...
                'ClassToUse',obj.pClassToUse,...
                'NumParticles',obj.NumParticles,...
                'NumBirthParticles',obj.NumBirthParticles,...
                'NumStateVariables',5,...
                'DeathRate',obj.DeathRate,...
                'BirthProbability',obj.BirthProbability,...
                'FreeSpaceDiscountFactor',obj.FreeSpaceDiscountFactor,...
                'ParticleSamplingFcn',@fusion.internal.GridMeasurementBuiltins.ctParticleSamplingFcn,...
                'ParticlePositionFcn',@fusion.internal.GridMeasurementBuiltins.ctParticlePositionFcn,...
                'StateTransitionFcn',@constturn,...
                'StateLimits',obj.StateLimits,...
                'HasAdditiveProcessNoise',obj.HasAdditiveProcessNoise,...
                'ProcessNoise',obj.ProcessNoise);
        end
    end
    
    methods
        function val = get.StateLimits(obj)
            switch obj.MotionModel
                case 'constant-velocity'
                    val = obj.VelocityLimits;
                case 'constant-acceleration'
                    val = [obj.VelocityLimits;obj.AccelerationLimits];
                case 'constant-turnrate'
                    val = [obj.VelocityLimits;obj.TurnRateLimits];
            end
        end
    end
    
    methods (Access = protected)
        function [confTracks, tentTracks, allTracks, map] = stepImpl(obj, varargin)
            time = varargin{end};
            
            % Validate time stamps
            validateTimeStamps(obj, varargin{:});
            
            measMap = obj.pMeasurementProjector(varargin{:});
            
            % Bring filter to the last step
            syncWithState(obj.pDynamicMapEstimator, obj.pLastKnownFilterState)
            
            if ~obj.pIsFirstCall
                % Predict the filter to current time
                dT = time - obj.pLastTime;
                
                % For association with grid cells, position parameters must
                % be updated before prediction
                obj.pDynamicMapEstimator.ParticlePositionParameters = measMap.PositionParameters;
                
                % Predict the filter
                predict(obj.pDynamicMapEstimator, dT);
                
                % Correct the filter
                correct(obj.pDynamicMapEstimator, measMap);
            else
                reinitialize(obj.pDynamicMapEstimator, measMap);
            end

            % Get map from the filter
            map = obj.pDynamicMapEstimator.DynamicMap;
            
            % Then use the dynamic map to extract tracks
            [confTracks, tentTracks, allTracks] = obj.pObjectExtractor(map, time);
            
            obj.pIsFirstCall = false;
            
            % Update time
            obj.pLastTime = cast(time,obj.pClassToUse);
            
            % Capture filter state
            obj.pLastKnownFilterState = captureState(obj.pDynamicMapEstimator);
        end
    end
    
    methods (Access = protected)
        function tf = isInactivePropertyImpl(obj,prop)
            tf = false;
            tf = tf || (strcmpi(obj.MotionModel,'constant-velocity') && any(strcmpi(prop,{'AccelerationLimits','TurnRateLimits'})));
            tf = tf || (strcmpi(obj.MotionModel,'constant-acceleration') && any(strcmpi(prop,{'TurnRateLimits'})));
            tf = tf || (strcmpi(obj.MotionModel,'constant-turnrate') && any(strcmpi(prop,{'AccelerationLimits'})));
            tf = tf || (strcmpi(obj.Clustering,'dbscan') && strcmpi(prop,'CustomClusteringFcn'));
            tf = tf || (strcmpi(obj.Clustering,'Custom') && any(strcmpi(prop,{'ClusteringThreshold','MinNumCellsPerCluster'})));
        end
        
        function validateTimeStamps(obj, varargin)
            % Validate tracker update time
            time = varargin{end};
            coder.internal.assert(time > obj.pLastTime,'fusion:GridTracker:TimeMustIncrease','step');
            
            % Validate sensor time-stamps
            sensorData = varargin{1};
            if coder.target('MATLAB')
                sensorTimes = vertcat(sensorData.Time);
            else
                sensorTimes = zeros(numel(sensorData),1,obj.pClassToUse);
                for i = 1:numel(sensorData)
                    sensorTimes(i) = sensorData(i).Time;
                end
            end
            coder.internal.assert(all(sensorTimes > obj.pLastTime & sensorTimes <= time + obj.pConstantTimetol),'fusion:GridTracker:DetectionTimeMismatch','step');
        end
        
        function setStateParameters(obj,val)
            validateattributes(val,{'struct'},{},mfilename,'StateParameters');
            obj.pStateParameters  = val;
            if coder.target('MATLAB')
                setExtractorStateParameters(obj, val);
            else
                coder.internal.defer_inference('setExtractorStateParameters',obj,val);
            end
        end
    end
    
    methods (Access = protected)
        function groups = getPropertyGroups(~)
            idGroup = matlab.mixin.util.PropertyGroup(...
                {'TrackerIndex','SensorConfigurations','HasSensorConfigurationsInput',...
                'StateParameters','MaxNumSensors','MaxNumTracks'}, 'Tracker Configuration');
            
            gridGroup = matlab.mixin.util.PropertyGroup(...
                {'GridLength','GridWidth','GridResolution','GridOriginInLocal'},...
                'Grid definition');
            
            pfGroup = matlab.mixin.util.PropertyGroup(...
                {'MotionModel',...
                'VelocityLimits',...
                'AccelerationLimits',...
                'TurnRateLimits',...
                'ProcessNoise',...
                'HasAdditiveProcessNoise',...
                'NumParticles',...
                'NumBirthParticles',...
                'BirthProbability', ...
                'DeathRate', ...
                'FreeSpaceDiscountFactor'},...
                'Particle filtering');
            
            clusteringGroup = matlab.mixin.util.PropertyGroup(...
                {'Clustering', 'CustomClusteringFcn','ClusteringThreshold', 'MinNumCellsPerCluster','TrackInitializationFcn'},...
                'Track initialization');
            
            objGroup = matlab.mixin.util.PropertyGroup(...
                {'TrackUpdateFcn','AssignmentThreshold','ConfirmationThreshold','DeletionThreshold','NumTracks','NumConfirmedTracks'},...
                'Track management');
            
            groups = [idGroup gridGroup pfGroup clusteringGroup objGroup];
        end
    end
    
    methods (Access = protected)
        function s = saveObjectImpl(obj)
            s = saveObjectImpl@matlab.System(obj);
            s.pStateParameters = obj.pStateParameters;
            s.pIsFirstCall = obj.pIsFirstCall;
            s.pMeasurementProjector = clone(obj.pMeasurementProjector);
            if isLocked(obj)
                if coder.internal.is_defined(obj.pClusterer)
                    s.pClusterer = obj.pClusterer;
                end
                s.pClassToUse = obj.pClassToUse;
                s.pDynamicMapEstimator = clone(obj.pDynamicMapEstimator);
                s.pObjectExtractor = clone(obj.pObjectExtractor);
                s.pLastTime = obj.pLastTime;
                s.pLastKnownFilterState = obj.pLastKnownFilterState;
            end
        end
        
        function loadObjectImpl(obj, s, wasLocked)
            loadObjectImpl@matlab.System(obj, s, wasLocked)
            obj.pStateParameters = s.pStateParameters;
            obj.pIsFirstCall = s.pIsFirstCall;
            obj.pMeasurementProjector = s.pMeasurementProjector;
            if wasLocked
                if isfield(s,'pClusterer')
                    obj.pClusterer = s.pClusterer;
                end
                obj.pClassToUse = s.pClassToUse;
                obj.pObjectExtractor = s.pObjectExtractor;
                obj.pDynamicMapEstimator = s.pDynamicMapEstimator;
                obj.pLastTime = s.pLastTime;
                if isfield(s,'pLastKnownFilterState')
                    obj.pLastKnownFilterState = s.pLastKnownFilterState;
                else
                    obj.pLastKnownFilterState = captureState(obj.pDynamicMapEstimator);
                end
            end
        end
        
        function newObj = cloneImpl(obj)
            newObj = cloneImpl@matlab.System(obj);
            newObj.pStateParameters = obj.pStateParameters;
            newObj.pIsFirstCall = obj.pIsFirstCall;
            newObj.pMeasurementProjector = clone(obj.pMeasurementProjector);
            if coder.internal.is_defined(obj.pClusterer)
                newObj.pClusterer = clone(obj.pClusterer);
            end
            if coder.internal.is_defined(obj.pObjectExtractor) % Happens after setup
                newObj.pClassToUse = obj.pClassToUse;
                newObj.pLastTime = obj.pLastTime;
                newObj.pObjectExtractor = clone(obj.pObjectExtractor);
            end
            if coder.internal.is_defined(obj.pDynamicMapEstimator)
                newObj.pDynamicMapEstimator = clone(obj.pDynamicMapEstimator);
                newObj.pLastKnownFilterState = obj.pLastKnownFilterState;
            end
        end
        
        function releaseImpl(obj)
            % Release resources
            release(obj.pMeasurementProjector);
            release(obj.pObjectExtractor);
            releaseImpl@matlab.System(obj);
        end
        
        function resetImpl(obj)
            % Reset the tracker to its original state
            reset(obj.pMeasurementProjector);
            reset(obj.pDynamicMapEstimator);
            reset(obj.pObjectExtractor);
            obj.pLastTime = cast(-eps,'like',obj.pLastTime);
            obj.pIsFirstCall = true;
        end
    end
    
    methods (Static)
        function tf = isAllowedInSystemBlock()
            tf = false;
        end
    end
    
    methods (Static)
        function track = defaultTrackInitialization(gridCells)
            track = fusion.internal.GridObjectExtractor.defaultTrackInitialization(gridCells);
        end
        function track = defaultTrackUpdate(track, gridCells)
            track = fusion.internal.GridObjectExtractor.defaultTrackUpdate(track, gridCells);
        end
    end
    methods (Static, Access = {?trackerGridRFS,?fusion.internal.GridObjectExtractor})
        function d = defaultTrackDistance(map, track)
            d = fusion.internal.GridObjectExtractor.defaultTrackDistance(map,track);
        end
    end    
end

% Local function for calling DBSCAN
function idx = clusterCells(dbscan,posIdx,dynamicCells)

% DBSCAN only supports double-precision inputs
x = cast(dynamicCells.States(posIdx,:),'double');

% Gather to ensure x is on CPU
idx = dbscan.cluster(gather(x'));

end

