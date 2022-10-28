% Sensor Fusion and Tracking Toolbox
% Version 2.3 (R2022a) 13-Nov-2021
%
% Multi-Target Trackers
%   trackerGNN                             - Global nearest neighbor multi-target tracker
%   trackerJPDA                            - Joint probabilistic data association multi-target tracker
%   trackerTOMHT                           - Multi-hypothesis multi-target tracker (track oriented)
%   trackerPHD                             - Multi-target probability hypothesis density tracker
%   trackerGridRFS                         - Multi-target 2-D grid-based tracker
%   dynamicEvidentialGridMap               - Dynamic map estimate from trackerGridRFS
%   objectDetection                        - Report for a single object detection
%   getTrackPositions                      - Track positions and position covariances
%   getTrackVelocities                     - Track velocities and velocity covariances
%   partitionDetections                    - Partition detections based on distance
%
% Multi-Target Tracker Components
%   trackHistoryLogic                      - History-based track confirmation and deletion logic
%   trackScoreLogic                        - Score-based track confirmation and deletion logic
%   trackBranchHistory                     - Create and manage track branches
%   clusterTrackBranches                   - Cluster track-oriented multi-hypothesis history
%   compatibleTrackBranches                - Formulate global hypotheses from clusters
%   pruneTrackBranches                     - Prune track branches with low likelihood
%   jpdaEvents                             - Generate feasible joint events for trackerJPDA
%   trackingSensorConfiguration            - Configuration of the sensor used with trackerPHD
%
% Data Assignment
%   matchpairs                             - Global nearest neighbor 2-D assignment based on Duff-Koster
%   assignmunkres                          - Global nearest neighbor 2-D assignment based on Munkres
%   assignauction                          - Global nearest neighbor 2-D assignment based on Auction
%   assignjv                               - Global nearest neighbor 2-D assignment based on Jonker-Volgenant
%   assignkbest                            - Murty's K-best solutions for 2-D assignment
%   assignsd                               - S-D assignment using Lagrangian relaxation
%   assignkbestsd                          - K-best solutions to S-D assignment using Lagrangian Relaxation
%   assignTOMHT                            - Track-oriented multi-hypotheses tracking (TOMHT) assignment
%
% Tracking Filters
%   trackingABF                            - Alpha-beta-gamma filter
%   trackingKF                             - Linear Kalman filter 
%   trackingEKF                            - Extended Kalman filter
%   trackingUKF                            - Unscented Kalman filter
%   trackingCKF                            - Cubature Kalman filter 
%   trackingPF                             - Particle filter 
%   trackingMSCEKF                         - Extended Kalman filter in modified spherical coordinates
%   trackingGSF                            - Gaussian-sum filter
%   trackingIMM                            - Interacting Multiple Model filter
%   ggiwphd                                - Gamma Gaussian Inverse Wishart PHD filter
%   gmphd                                  - Gaussian mixture PHD filter
%
% Filter Initialization Functions for Trackers
%   initcvabf                              - Constant velocity trackingABF initialization
%   initcaabf                              - Constant acceleration trackingABF initialization
%   initcvkf                               - Constant velocity trackingKF initialization
%   initcakf                               - Constant acceleration trackingKF initialization
%   initcvekf                              - Constant velocity trackingEKF initialization
%   initcaekf                              - Constant acceleration trackingEKF initialization
%   initctekf                              - Constant turn rate trackingEKF initialization
%   initcvukf                              - Constant velocity trackingUKF initialization
%   initcaukf                              - Constant acceleration trackingUKF initialization
%   initctukf                              - Constant turn rate trackingUKF initialization
%   initcvckf                              - Constant velocity trackingCKF initialization
%   initcackf                              - Constant acceleration trackingCKF initialization
%   initctckf                              - Constant turn rate trackingCKF initialization
%   initcvpf                               - Constant velocity trackingPF initialization
%   initcapf                               - Constant acceleration trackingPF initialization
%   initctpf                               - Constant turn rate trackingPF initialization
%   initsingerekf                          - Singer acceleration trackingEKF initialization
%   initcvmscekf                           - Constant velocity trackingMSCEKF initialization
%   initrpekf                              - Constant velocity range-parameterized EKF initialization
%   initapekf                              - Constant velocity angle-parameterized EKF initialization
%   initekfimm                             - trackingIMM initialization 
%
% Filter Initialization Functions for PHD Tracker
%   initcvggiwphd                          - Constant velocity ggiwphd initialization
%   initcaggiwphd                          - Constant acceleration ggiwphd initialization
%   initctggiwphd                          - Constant turn rate ggiwphd initialization
%   initcvgmphd                            - Constant velocity gmphd initialization
%   initcagmphd                            - Constant acceleration gmphd initialization
%   initctgmphd                            - Constant turn-rate gmphd initialization
%   initctrectgmphd                        - Constant turn-rate rectangular target gmphd initialization
%
% Motion and Measurement Models for Tracking Filters
%   constvel                               - Constant velocity (CV) motion model
%   constveljac                            - Jacobian of the constant velocity (CV) motion model
%   cvmeas                                 - Measurement based on the constant velocity (CV) motion model
%   cvmeasjac                              - Jacobian of the measurement using the constant velocity (CV) model
%   constacc                               - Constant acceleration (CA) motion model
%   constaccjac                            - Jacobian of the constant acceleration (CA) motion model
%   cameas                                 - Measurement based on the constant acceleration (CA) motion model
%   cameasjac                              - Jacobian of the measurement using the constant acceleration (CA) model
%   constturn                              - Constant turn rate (CT) motion model
%   constturnjac                           - Jacobian of the constant turn rate (CT) model
%   ctmeas                                 - Measurement based on the constant turn rate (CT) motion model
%   ctmeasjac                              - Jacobian of the measurement using the constant turn rate (CT) model
%   constvelmsc                            - Constant velocity (CV) motion model in MSC frame
%   constvelmscjac                         - Jacobian of the constant velocity (CV) motion model in MSC frame
%   cvmeasmsc                              - Measurement based on the constant velocity (CV) motion model in MSC frame
%   cvmeasmscjac                           - Jacobian of the measurement using the constant velocity (CV) model in MSC frame
%   singer                                 - Singer acceleration motion model
%   singerjac                              - Jacobian of the Singer acceleration model
%   singermeas                             - Measurement based on Singer acceleration motion model
%   singermeasjac                          - Jacobian of the measurement using the Singer acceleration model
%   singerProcessNoise                     - Process noise matrix for Singer acceleration model
%   ctrect                                 - Constant turn-rate rectangular target model
%   ctrectjac                              - Jacobian of the constant turn-rate rectangular target model
%   ctrectmeas                             - Measurement based on the constant turn-rate rectangular target model
%   ctrectmeasjac                          - Jacobian of the measurement using constant turn-rate rectangular target model
%   ctrectcorners                          - Corner measurements of the constant turn-rate rectangular target model
%   switchimm                              - Model conversion function for trackingIMM object
%
% State, Detection, and Track Fusion
%   staticDetectionFuser                   - Static fusion of synchronous sensor detections
%   triangulateLOS                         - Triangulate multiple line-of-sight(LOS) detections
%   trackFuser                             - A track-to-track fuser
%   fuserSourceConfiguration               - Configuration of a source used with track fuser
%   objectTrack                            - Report for single object track
%   fusexcov                               - Covariance fusion using cross-covariance
%   fusecovint                             - Covariance fusion using covariance intersection
%   fusecovunion                           - Covariance fusion using covariance union
%   mergeDetections                        - Merge detections into clusters
%
% Tracking System-of-System Architecture
%   trackingArchitecture                   - Tracking system-of-system architecture
%   trackingArchitecture/addTracker        - Add a tracker to a tracking architecture
%   trackingArchitecture/addTrackFuser     - Add a track-to-track fusion algorithm to a tracking architecture
%   trackingArchitecture/summary           - View a summary of the tracking architecture in a tabular form
%   trackingArchitecture/show              - Plot the tracking architecture in a figure
%   fusion.trackingArchitecture.Tracker    - Interface definition for trackingArchitecture tracker
%   fusion.trackingArchitecture.TrackFuser - Interface definition for trackingArchitecture track fuser
%
% Track Visualization and Metrics
%   theaterPlot                            - Plot ground truth, detections and tracks
%   theaterPlot/platformPlotter            - Plot platforms (target or sensor platform)
%   theaterPlot/detectionPlotter           - Plot object detections and velocities
%   theaterPlot/trackPlotter               - Plot tracks, track uncertainties, and history trails
%   theaterPlot/orientationPlotter         - Plot object orientations
%   theaterPlot/trajectoryPlotter          - Plot object trajectory
%   theaterPlot/coveragePlotter            - Plot sensors and emitters coverage areas and beams
%   trackingGlobeViewer                    - Virtual globe for tracking scenario visualization
%   coverageConfig                         - Return sensors and emitters coverage configurations
%   trackAssignmentMetrics                 - Track establishment, maintenance, and deletion metrics
%   trackErrorMetrics                      - Track error and NEES
%   trackOSPAMetric                        - Optimal subpattern assignment (OSPA) metric
%   trackGOSPAMetric                       - Generalized optimal subpattern assignment (GOSPA) metric
%
% Scenario Generation
%   trackingScenarioDesigner               - App for designing tracking scenarios and generating detections
%   trackingScenario                       - Generate objects and trajectories in a tracking scenario
%   trackingScenario/platform              - Create a platform (target or sensor platform)
%   trackingScenario/groundSurface         - Create a ground surface
%   trackingScenario/advance               - Move simulation forward by one time interval
%   trackingScenario/record                - Record entire simulation
%   trackingScenario/platformProfiles      - Retrieve physical attributes for each platform
%   trackingScenario/platformPoses         - Retrieve poses (i.e. positions and orientations) of platforms
%   trackingScenario/emit                  - Collect emissions from all the emitters in the scenario
%   trackingScenario/propagate             - Propagate emissions in the scenario
%   trackingScenario/detect                - Collect detections from all the sensors in the scenario
%   trackingScenario/lidarDetect           - Collect point clouds from all the lidar sensors in the scenario
%   trackingScenario/coverageConfig        - Return sensors and emitters coverage configurations in the scenario
%   trackingScenario/perturb               - Perturb the tracking scenario
%   trackingScenario/clone                 - Create a copy of the tracking scenario
%   trackingScenarioRecording              - A tracking scenario recording
%   objectDetectionDelay                   - Simulate out-of-sequence measurements (OOSM)
%   monteCarloRun                          - Perform Monte Carlo realizations of a tracking scenario
%   emissionsInBody                        - Transform emissions to the body frame of a platform
%   extendedObjectMesh                     - Mesh representation of an extended object for lidar simulation
%   tracking.scenario.airplaneMesh         - Pre-built mesh for a airplane for lidar simulation
%
% Trajectory Generation
%   waypointTrajectory                     - Trajectory from position, time, orientation and velocity values
%   kinematicTrajectory                    - Trajectory from translational and rotational rates
%   geoTrajectory                          - Trajectory from geodetic coordinates, time, local orientation and velocity values
%
% Sensor Models for Localization
%   accelparams                            - Accelerometer sensor parameters
%   gyroparams                             - Gyroscope sensor parameters
%   magparams                              - Magnetometer sensor parameters
%   imuSensor                              - Simulate IMU comprising accelerometer, gyroscope, and/or magnetometer
%   gpsSensor                              - Simulate GPS position, velocity, groundspeed, and course
%   altimeterSensor                        - Simulate altimeter
%   allanvar                               - Allan variance
%   magcal                                 - Magnetometer calibration
%   transformMotion                        - Convert motion quantities between two frames
%
% Sensor Fusion for Localization
%   ecompass                               - Accelerometer and magnetometer fusion for orientation
%   imufilter                              - Accelerometer and gyroscope fusion for orientation
%   ahrsfilter                             - Accelerometer, gyroscope, magnetometer fusion for orientation with Kalman filter
%   complementaryFilter                    - Accelerometer, gyroscope, magnetometer fusion for orientation with complementary filter
%   ahrs10filter                           - Accelerometer, gyroscope, magnetometer, altimeter fusion for pose
%   insfilter                              - IMU and GPS fusion for orientation and position (pose)
%   insfilterMARG                          - Pose estimation filter
%   insfilterNonholonomic                  - Pose estimation filter with nonholonomic constraints
%   insfilterAsync                         - Asynchronous pose estimation filter
%   insfilterErrorState                    - Error-state pose estimation filter
%   tunerconfig                            - Configuration of fusion filter tuner
%   tunernoise                             - Noise structure for fusion filter tuner
%   timescope                              - Streaming display of time-domain signals
%   poseplot                               - 3-D pose plot
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
% Sensor Models for Tracking
%   fusionRadarSensor                      - Scanning radar detection generator
%   radarEmitter                           - Radar signals and interferences generator
%   rcsSignature                           - Radar cross-section (RCS) signature
%   radarChannel                           - Atmospheric propagation and reflections of radar signals
%   radarEmission                          - Report for a single radar signal emission
%   monostaticLidarSensor                  - Monostatic lidar point cloud generator
%   sonarSensor                            - Active or passive sonar detections generator
%   sonarEmitter                           - Acoustic signals and interferences generator
%   tsSignature                            - Sonar target strength (TS) signature
%   underwaterChannel                      - Underwater propagation and reflection of sonar signals
%   sonarEmission                          - Report for a single acoustic signal emission
%   irSensor                               - Infrared (IR) detections generator
%   irSignature                            - Infrared (IR) platform signature
%   insSensor                              - INS/GPS position, velocity, and orientation emulator 
%
% Broadcast System Models
%   adsbCategory                           - Enumeration of Automatic Dependent Surveillance - Broadcast categories
%   adsbTransponder                        - Automatic Dependent Surveillance - Broadcast transponder model
%   adsbReceiver                           - Automatic Dependent Surveillance - Broadcast receiver model
%
% Lat/Lon to Local Conversion
%   lla2enu                                - Transform geodetic coordinates to local East-North-Up coordinates
%   lla2ned                                - Transform geodetic coordinates to local North-East-Down coordinates
%   enu2lla                                - Transform local East-North-Up coordinates to geodetic coordinates
%   ned2lla                                - Transform local North-East-Down coordinates to geodetic coordinates
%
% Rotations
%   quaternion                             - Quaternion arrays from parts or other rotation representations
%   quaternion/rotmat                      - Convert quaternions to rotation matrices
%   quaternion/euler                       - Convert quaternions to Euler or Tait-Bryan angles (radians)
%   quaternion/eulerd                      - Convert quaternions to Euler or Tait-Bryan angles (degrees)
%   quaternion/rotvec                      - Convert quaternions to rotation vectors (radians)
%   quaternion/rotvecd                     - Convert quaternions to rotation vectors (degrees)
%   quaternion/compact                     - Arrays from quaternions
%   quaternion/parts                       - Extract four parts of a quaternion
%   quaternion/rotateframe                 - Rotate coordinate frame using quaternions
%   quaternion/rotatepoint                 - Rotate 3D points using quaternions
%   quaternion/norm                        - Quaternion norm
%   quaternion/normalize                   - Convert to a unit quaternion
%   quaternion/dist                        - Distance between two quaternions
%   quaternion/slerp                       - Spherical linear interpolation of quaternions
%   quaternion/meanrot                     - Average rotation of quaternions
%   randrot                                - Uniformly distributed random rotations
%
% Examples
%   <a href="matlab:demo('toolbox','fusion')">Examples</a>                              - Index of Sensor Fusion and Tracking Toolbox examples

% Copyright 2021 The MathWorks, Inc.
