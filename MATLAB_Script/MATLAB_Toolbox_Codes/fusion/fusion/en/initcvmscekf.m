%INITCVMSCEKF  Constant velocity trackingMSCEKF initialization
%   trackingMSCEKF can be used with trackers for tracking targets with 
%   angle-only measurements from a single observer. 
%
%   MSCEKF = INITCVMSCEKF(Detection) initializes a trackingMSCEKF (MSC-EKF)
%   based on information provided in Detection. It assumes a target range
%   of 3e4 units and range-covariance as 1e10 units^2.
%
%   MSCEKF = INITCVMSCEKF(Detection,rangeEstimation) allows specifying the
%   range information to the filter. rangeEstimation is a 2-element vector,
%   where first element specifies the range of the target and second
%   element specifies the standard deviation in range.
%
%   Detection must be an objectDetection. The detection must specify the
%   measurement parameters, as a scalar or array of struct with the
%   following fields. Each struct specifies the transformation to move from
%   sensor to scenario frame.
%     Frame           - 'spherical' or 'rectangular' or enum with same value.
%                        First measurement parameter must specify Frame as 
%                        'spherical' or enum with same value.
%     HasRange        - a logical scalar.      must be set to false.
%     Orientation     - a 3-by-3 orthonormal 
%                       orientation matrix.    Default: eye(3)
%     HasElevation    - a logical scalar.      Default: true, elevation is measured
%     IsParentToChild - a logical scalar.      Default: false, orientation defines a rotation from child to parent frame.
%   Notes:
%   ------
%   1. You can use INITCVMSCEKF as the FilterInitializationFcn property.
%   2. The function configures the filter with process noise 
%      assuming a unit target acceleration standard deviation.
%   3. The function configures the covariance of the state in MSC
%      frame by using a linear transformation of covariance in
%      Cartesian frame.
%   4. The function initializes the ObserverInput of the trackingMSCEKF
%      with zero observer acceleration in all directions. You must use
%      the setTrackFilterProperties function of the trackers to update the
%      ObserverInput.
%
%   Example 1: Initialize a trackingMSCEKF using a detection.
%   -------------------------------------------------------
%   % Create an angle-only detection
%   detection = objectDetection(0,[30;20],'MeasurementParameters',...
%   struct('Frame','Spherical','HasRange',false));
%   filter = INITCVMSCEKF(detection);
%
%   Example 2: Initialize a trackingMSCEKF with a detection from rotating sensor.
%   -----------------------------------------------------------------------
%   % Create measurement parameters for subsequent rotation.
%   measParamSensorToPlat = struct('Frame','Spherical','HasRange',false,...
%   'Orientation',rotmat(quaternion([0 0 30],'rotvecd'),'frame'))
%   measParamPlatToScenario = struct('Frame','Rectangular','HasRange',false,...
%   'Orientation',rotmat(quaternion([30 0 0],'rotvecd'),'frame'))
%   measParam = [measParamSensorToPlat;measParamPlatToScenario];
%   detection = objectDetection(0,[30;20],'MeasurementParameters',measParam);
%   % Initialize a filter
%   filter = INITCVMSCEKF(detection);
%   % Check that filter's measurement is same as detection
%   cvmeasmsc(filter.State,measParam)
%
%   Example 3: Tracking a constant velocity target using trackerGNN
%   ---------------------------------------------------------------
%   % Consider a scenario when the target is moving at a constant velocity
%   % along and the observer is moving at a constant acceleration
%   % Define target's initial state using a constant velocity model.
%   tgtState = [2000;-3;500;-5;0;0];
%   % Define observer's initial state using a constant acceleration model.
%   observerState = [0;2;0;490;-10;0.2;0;0;0];
%   
%   % Create a trackerGNN object to use with INITCVMSCEKF with some prior
%   % information about range and range-covariance.
%   range = 1000;
%   rangeStdDev = 1e3;
%   rangeEstimate = [range rangeStdDev];
%   tracker = trackerGNN('FilterInitializationFcn',@(det)INITCVMSCEKF(det,rangeEstimate));
%     
%   % Simulate synthetic data by using measurement models.
%   % Get az and el information using cvmeas function.
%   syntheticParams = struct('Frame','Spherical','HasRange',false,...
%   'OriginPosition',observerState(1:3:end));
%   meas = cvmeas(tgtState,syntheticParams);
%     
%   % Create an angle-only objectDetection to simulate synthetic detection
%   detection = objectDetection(0,meas,'MeasurementParameters',...
%   struct('Frame','Spherical','HasRange',false),'MeasurementNoise',0.033*eye(2));
%     
%   % Create trackPlotter and platformPlotter to visualize the scenario.
%   tp = theaterPlot('XLimits',[0 2500],'YLimits',[0 1000]);
%   targetPlotter = platformPlotter(tp,'DisplayName','Target','MarkerFaceColor','k');
%   observerPlotter = platformPlotter(tp,'DisplayName', 'Observer','MarkerFaceColor','r');
%   trkPlotter = trackPlotter(tp,'DisplayName','Track','MarkerFaceColor','g','HistoryDepth',50);
%   tgtTrajPlotter = trajectoryPlotter(tp,'DisplayName','Target Trajectory','Color','k');
%   obsTrajPlotter = trajectoryPlotter(tp,'DisplayName','Observer Trajectory','Color','r');
%     
%   % Run the tracker
%   time = 0; dT = 0.1;
%   tgtPoses = [];
%   obsPoses = [];
%     while time < 50
%         [confTracks,tentTracks,allTracks] = tracker(detection,time);
%         for i = 1:numel(allTracks)
%             setTrackFilterProperties(tracker,allTracks(i).TrackID,'ObserverInput',observerState(3:3:end));
%         end
%         % Update synthetic detection.
%         observerState = constacc(observerState,dT);
%         tgtState = constvel(tgtState,dT);
%         syntheticParams.OriginPosition = observerState(1:3:end);
%         detection.Measurement = cvmeas(tgtState,syntheticParams);
%         time = time + dT;
%         detection.Time = time;
% 
%         % Update plots
%         tgtPoses = [tgtPoses;tgtState(1:2:end)'];
%         obsPoses = [obsPoses;observerState(1:3:end)'];
%         targetPlotter.plotPlatform(tgtState(1:2:end)');
%         observerPlotter.plotPlatform(observerState(1:3:end)');
%         tgtTrajPlotter.plotTrajectory({tgtPoses});
%         obsTrajPlotter.plotTrajectory({obsPoses});
%         % Plot the first track as there are no false alarms, this should be
%         % the target.
%         % Get positions from the MSC state of the track.
%         cartState = cvmeasmsc(allTracks(i).State,'rectangular') + observerState(1:3:end);
%         trkPlotter.plotTrack(cartState');
%     end
%
%   See also: trackingMSCEKF, objectDetection, constvelmsc, constvelmscjac,
%   cvmeasmsc, cvmeasmscjac

    
% Copyright 2018-2021 The MathWorks, Inc.

