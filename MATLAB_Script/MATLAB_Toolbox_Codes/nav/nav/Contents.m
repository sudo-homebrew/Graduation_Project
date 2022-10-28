% Navigation Toolbox
% Version 2.2 (R2022a) 13-Nov-2021
%
% Mapping
%   binaryOccupancyMap         - Create an occupancy grid map with binary values
%   exportOccupancyMap3D       - Export 3-D occupancy map as an octree or binary tree file
%   importOccupancyMap3D       - Import an octree file as a 3-D occupancy map
%   occupancyMap               - Create an occupancy grid map with probabilistic values
%   occupancyMap3D             - Create a 3D occupancy map
%   mapClutter                 - Generate occupancy map with randomly-placed obstacles
%   mapLayer                   - Create map layer for N-dimensional data
%   mapMaze                    - Generate complicated paths with straight passage, turns, T-junctions
%   multiLayerMap              - Manage multiple map layers
%
% Localization
%   likelihoodFieldSensorModel              - Create a likelihood field range sensor model
%   odometryMotionModel                     - Create an odometry motion model         
%   monteCarloLocalization                  - Localize a robot with range sensor data and map
%   stateEstimatorPF                        - Create a particle filter state estimator                  
%   lidarScan                               - Create object for storing 2D LiDAR scan
%   matchScans                              - Estimate pose between two laser scans
%   matchScansGrid                          - Estimate pose between two laser scans using grid-based search
%   matchScansLine                          - Estimate pose between two laser scans using line features
%   transformScan                           - Transform laser scan based on relative pose
%   wheelEncoderOdometryUnicycle            - Compute unicycle odometry using wheel encoder ticks
%   wheelEncoderOdometryBicycle             - Compute bicycle odometry using wheel encoder ticks
%   wheelEncoderOdometryDifferentialDrive   - Compute differential-drive vehicle odometry using wheel encoder ticks
%   wheelEncoderOdometryAckermann           - Compute Ackermann vehicle odometry using wheel encoder ticks
%
% Path Planning
%   plannerControlRRT          - Create control-based rapidly-exploring random tree planner
%   plannerRRT                 - Create geometric rapidly-exploring random tree path planner 
%   plannerRRTStar             - Create optimal rapidly-exploring random tree path planner
%   plannerBiRRT               - Create bidirectional RRT planner for geometric planning
%   plannerHybridAStar         - Create a hybrid A star path planner
%   plannerAStarGrid           - Create grid based A star path planner
%   plannerPRM                 - Create Probabilistic Roadmap planner for geometric planning
%   optimizePath               - Optimize path while maintaining safe distance from obstacle
%   optimizePathOptions        - Create optimization options for optimizePath function
%   navPath                    - Planned path
%   navPathControl             - Path representing control-based kinematic trajectory
%   pathmetrics                - Calculate path metrics
%   mobileRobotPropagator      - State propagator for wheeled robotic systems
%   stateSpaceSE2              - SE(2) state space
%   stateSpaceSE3              - SE(3) state space
%   stateSpaceDubins           - State space representation for Dubins vehicles
%   stateSpaceReedsShepp       - State space representation for Reeds-Shepp vehicles
%   dubinsConnection           - Connect two poses with a Dubins path
%   dubinsPathSegment          - Create a Dubins path segment
%   reedsSheppConnection       - Connect two poses with a Reeds-Shepp path
%   reedsSheppPathSegment      - Create a Reeds-Shepp path segment
%   referencePathFrenet        - Smooth reference path fit to waypoints
%   validatorOccupancyMap      - State validation for grid maps
%   validatorOccupancyMap3D    - State validation for 3D grid maps
%   validatorVehicleCostmap    - State validation for vehicle costmaps
%   trajectoryOptimalFrenet    - Find optimal trajectory for a given reference path
%   trajectoryGeneratorFrenet  - Generate trajectories using reference path
%   createPlanningTemplate     - Create example implementation for path planning interface
%   plannerBenchmark           - Benchmark path planners using generated metrics
%
% Collision Checking
%   dynamicCapsuleList         - Dynamic obstacle list using capsules in SE2
%   dynamicCapsuleList3D       - Dynamic obstacle list using capsules in SE3
%
% Control
%   controllerPurePursuit      - Create a controller to follow a set of waypoints
%   controllerVFH              - Avoid obstacles using vector field histogram 
%
% SLAM
%   lidarSLAM                  - Lidar-based simultaneous localization and mapping
%   poseGraph                  - Create a 2D pose graph representation
%   poseGraph3D                - Create a 3D pose graph representation
%   poseGraphSolverOptions     - Create solver options for pose graph optimization
%   optimizePoseGraph          - Refine poses in a pose graph to best satisfy constraints
%   buildMap                   - Create an occupancy grid map from laser scans and poses
%   slamMapBuilder             - App for mapping unknown environment using lidar-based SLAM algorithm
%   trimLoopClosures           - Optimize pose graph and remove bad loop closures
%   ekfSLAM                    - Implement Simultaneous Localization And Mapping using Extended Kalman Filter
%   factorGraph                - Create a factor graph object
%   factorGraphSolverOptions   - Create solver options for optimizing factor graph
%   factorTwoPoseSE2           - Create a factor between two SE(2) poses
%   factorTwoPoseSE3           - Create a factor between two SE(3) poses
%   factorPoseSE3Prior         - Create a prior factor object for pose SE3
%   factorIMUBiasPrior         - Create a prior factor for IMU biases
%   factorVelocity3Prior       - Create a prior factor for velocity in 3D
%   factorIMU                  - Create an IMU factor
%   factorGPS                  - Create a GPS factor
%   importFactorGraph          - Create factor graph from g2o log file
%
% Coordinate Transformations
%   axang2quat         - Convert axis-angle rotation representation to quaternion
%   axang2rotm         - Convert axis-angle rotation representation to rotation matrix
%   axang2tform        - Convert axis-angle rotation representation to a homogeneous transform
%   cart2hom           - Convert Cartesian coordinates to homogeneous coordinates
%   eul2quat           - Convert Euler angles to quaternion
%   eul2rotm           - Convert Euler angles to rotation matrix
%   eul2tform          - Convert Euler angles to homogeneous transformation
%   hom2cart           - Convert homogeneous coordinates to Cartesian coordinates
%   quat2axang         - Convert quaternion to axis-angle rotation representation
%   quat2eul           - Convert quaternion to Euler angles
%   quat2rotm          - Convert quaternion to rotation matrix
%   quat2tform         - Convert quaternion to homogeneous transformation
%   rotm2axang         - Convert rotation matrix to axis-angle representation
%   rotm2eul           - Convert rotation matrix to Euler angles
%   rotm2quat          - Convert rotation matrix to quaternion
%   rotm2tform         - Convert rotation matrix to homogeneous transform
%   tform2axang        - Extract axis-angle rotation from homogeneous transformation
%   tform2eul          - Extract Euler angles from homogeneous transformation
%   tform2quat         - Extract quaternion from homogeneous transformation
%   tform2rotm         - Extract rotation matrix from homogeneous transformation
%   tform2trvec        - Extract translation vector from homogeneous transformation
%   trvec2tform        - Convert translation vector to homogeneous transformation
%   quaternion         - Create a quaternion
%   randrot            - Uniformly distributed random rotations
%
% Lat/Lon to Local Conversion
%   lla2enu            - Transform geodetic coordinates to local East-North-Up coordinates
%   lla2ned            - Transform geodetic coordinates to local North-East-Down coordinates
%   enu2lla            - Transform local East-North-Up coordinates to geodetic coordinates
%   ned2lla            - Transform local North-East-Down coordinates to geodetic coordinates
%
% Angular Operations
%   angdiff            - Calculate difference between two angles
%
% Trajectory Generation
%   waypointTrajectory                - Trajectory from position, time, orientation and velocity values
%   kinematicTrajectory               - Trajectory from translational and rotational rates
%
% Sensor Models for Localization
%   accelparams                       - Accelerometer sensor parameters
%   gyroparams                        - Gyroscope sensor parameters
%   magparams                         - Magnetometer sensor parameters
%   imuSensor                         - Simulate IMU comprising accelerometer, gyroscope, and/or magnetometer
%   gpsSensor                         - Simulate GPS position, velocity, groundspeed, and course
%   gnssSensor                        - Simulate GNSS position, velocity, and satellites
%   altimeterSensor                   - Simulate altimeter
%   insSensor                         - INS/GPS position, velocity, and orientation emulator 
%   rangeSensor                       - Simulate range-bearing sensor readings
%   wheelEncoderUnicycle              - Simulate wheel encoder sensor readings for unicycle vehicle
%   wheelEncoderBicycle               - Simulate wheel encoder sensor readings for bicycle vehicle
%   wheelEncoderDifferentialDrive     - Simulate wheel encoder sensor readings for differential drive vehicle
%   wheelEncoderAckermann             - Simulate wheel encoder sensor readings for Ackermann vehicle
%   allanvar                          - Allan variance
%   magcal                            - Magnetometer calibration
%   transformMotion                   - Convert motion quantities between two frames
%   rinexinfo                         - Get information about a RINEX file
%   rinexread                         - Read data from RINEX file
%   gnssconstellation                 - Satellite locations at specified time
%   lookangles                        - Satellite look angles from receiver and satellite positions
%   pseudoranges                      - Calculate pseudoranges between GNSS receiver and satellites
%   receiverposition                  - Estimate GNSS receiver position and velocity
%
% Sensor Fusion for Localization
%   ecompass                          - Accelerometer and magnetometer fusion for orientation
%   imufilter                         - Accelerometer and gyroscope fusion for orientation
%   ahrsfilter                        - Accelerometer, gyroscope, magnetometer fusion for orientation with Kalman filter
%   complementaryFilter               - Accelerometer, gyroscope, magnetometer fusion for orientation with complementary filter
%   ahrs10filter                      - Accelerometer, gyroscope, magnetometer, altimeter fusion for pose
%   insfilter                         - IMU and GPS fusion for orientation and position (pose)
%   insfilterMARG                     - Pose estimation filter
%   insfilterNonholonomic             - Pose estimation filter with nonholonomic constraints
%   insfilterAsync                    - Asynchronous pose estimation filter
%   insfilterErrorState               - Error-state pose estimation filter
%   tunerconfig                       - Configuration of fusion filter tuner
%   tunernoise                        - Noise structure for fusion filter tuner
%
% Inertial Sensor Fusion Filter Design
%   insEKF                                  - Extended Kalman filter for inertial navigation 
%   insOptions                              - Filter design options for insEKF
%   insAccelerometer                        - Accelerometer model for insEKF
%   insGyroscope                            - Gyroscope model for insEKF
%   insMagnetometer                         - Magnetometer model for insEKF
%   insGPS                                  - GPS model for insEKF
%   insMotionOrientation                    - 3D Orientation-only motion model for insEKF
%   insMotionPose                           - 3D Pose motion model for insEKF
%
% Kinematic Models
%   bicycleKinematics                 - Create bicycle vehicle model
%
% Utilities
%   plotTransforms     - Plot 3-D transforms described by translations and rotations
%   rateControl        - Execute loop at a fixed frequency
%   timescope          - Streaming display of time-domain signals
%   skyplot            - Visualize azimuth and elevation data on a sky plot
%   poseplot           - 3-D pose plot
%
% Sensor Connectivity
%   gpsdev              - Connect to a GPS receiver connected to the host computer
%   nmeaParser          - Parse data from NMEA sentences
%   extractNMEASentence - Extracts NMEA sentence and returns split string that can be used in custom functions
%
% <a href="matlab:demo('toolbox','Navigation')">View examples</a> for Navigation Toolbox.

% Copyright 2018-2021 The MathWorks, Inc.
