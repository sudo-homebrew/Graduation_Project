classdef trackErrorMetrics< matlab.System
%trackErrorMetrics Track error and NEES
%   trackErrorMetrics returns a System object capable of providing
%   quantitative comparison of tracks against known "truth" trajectories.  
%
%   TEM = trackErrorMetrics('Name',value,...) returns a track
%   error metric object, TEM, with each specified property name 
%   set to the specified value. You can specify additional name-value 
%   pair arguments in any order as (Name1,Value1,...,NameN, ValueN).
%   See the list of properties below.
%
%   trackErrorMetrics properties:
%      ErrorFunctionFormat  - Selects error functions and format
%                             one of 'built-in' or 'custom'
%                             Default:  'built-in'
%
%   trackErrorMetrics 'built-in' error function properties:
%      MotionModel           - Specifies desired motion model
%                              one of 'constvel' 'constacc' 'constturn' or 'singer'
%                              Default:  'constvel'
%
%   trackErrorMetrics 'custom' error function properties:
%      EstimationErrorLabels  - Array of string labels for each output
%      EstimationErrorFcn     - Handle to an estimation error function
%      TrackIdentifierFcn     - Handle to a function returning unique track identifiers
%      TruthIdentifierFcn     - Handle to a function returning unique truth identifiers
%
%   Step method syntax: 
%
%      [OUT1, OUT2, ...] = step(TEM, TRACKS, TRACKIDS, TRUTHS, TRUTHIDS) 
%      returns metrics in the output over the current track-to-truth
%      assignments. TRACKIDS and TRUTHIDS are each a vector whose corresponding
%      elements match the track and truth identifiers found in TRACKS and 
%      TRUTHS, respectively.
%
%      When ErrorFunctionFormat is 'built-in', the number of outputs 
%      depends on the setting of the MotionModel property:
%
%                |   OUT1  |   OUT2  |   OUT3      |  OUT4    |   OUT5   |   OUT6
%     -----------+---------+---------+-------------+----------+----------+-------------
%     'constvel' | posRMSE | velRMSE | posANEES    | velANEES |   N/A    |   N/A
%     'constacc' | posRMSE | velRMSE | accRMSE     | posANEES | velANEES | accANEES
%     'constturn'| posRMSE | velRMSE | yawRateRMSE | posANEES | velANEES | yawRateANEES
%     'singer'   | posRMSE | velRMSE | accRMSE     | posANEES | velANEES | accANEES
%
%      where pos, vel, acc, and yawRate correspond to the position,
%      velocity, acceleration and yaw rate; and RMSE is the root
%      mean squared error, and ANEES is the average normalized estimation
%      error squared.
%
%      When ErrorFunctionFormat is 'custom', the number of outputs
%      corresponds to the number of elements listed in the EstimationErrorLabels
%      property, and should match the number of outputs in the EstimationErrorFcn.
%      The results of the estimation errors are averaged arithmetically over
%      all track-to-truth assignments.
%
%   System objects may be called directly like a function instead of using
%   the step method. For example, y = step(obj) and y = obj() are
%   equivalent. 
%
%   trackErrorMetrics methods:
%     <a href="matlab:help trackErrorMetrics/stepImpl">step</a>                 - See above description for use of this method
%     currentTrackMetrics    - Returns metrics for most recent tracks
%     currentTruthMetrics    - Returns metrics for most recent truths
%     cumulativeTrackMetrics - Returns cumulative metrics for all previous tracks
%     cumulativeTruthMetrics - Returns cumulative metrics for all previous truths
%     release                - Allows property value and input characteristics changes
%     clone                  - Creates a copy of the trackErrorMetrics
%     isLocked               - Locked status (logical)
%     <a href="matlab:help matlab.System/reset   ">reset</a>                  - Resets state of cumulative metrics
%
%   % EXAMPLE: Examine the results of a system that tracked two targets
%    
%   % load pre-recorded data
%   load trackmetricex tracklog truthlog
%
%   % construct an object to analyze assignment and error metrics
%   tam = trackAssignmentMetrics;
%   tem = trackErrorMetrics
% 
%   % create output variables
%   posRMSE = zeros(numel(tracklog),1);
%   velRMSE = zeros(numel(tracklog),1);
%   posANEES = zeros(numel(tracklog),1);
%   velANEES = zeros(numel(tracklog),1);
%
%   for i=1:numel(tracklog)
%       % extract the tracker and ground truth at the i-th tracker update
%       tracks = tracklog{i};
%       truths = truthlog{i};
% 
%       % analyze and retrieve the current track-to-truth assignment
%       [trackAM,truthAM] = tam(tracks, truths);
%       [trackIDs,truthIDs] = currentAssignment(tam);
%
%       % analyze instantaneous error metrics over all tracks and truths
%       [posRMSE(i),velRMSE(i),posANEES(i),velANEES(i)] = tem(tracks,trackIDs,truths,truthIDs);
%   end 
%
%   subplot(2,2,1)
%   plot(posRMSE)
%   title('Position Error');
%   xlabel('tracker update');
%   ylabel('RMSE (m)');
% 
%   subplot(2,2,2)
%   plot(velRMSE)
%   title('Velocity Error');
%   xlabel('tracker update');
%   ylabel('RMSE (m/s)');
%
%   subplot(2,2,3)
%   plot(posANEES)
%   title('Position Error');
%   xlabel('tracker update');
%   ylabel('ANEES');
% 
%   subplot(2,2,4)
%   plot(velANEES)
%   title('Velocity Error');
%   xlabel('tracker update');
%   ylabel('ANEES');
% 
%   % show cumulative error metrics for each individual recorded track
%   cumulativeTrackMetrics(tem)
% 
%   % show cumulative error metrics for each individual recorded truth object
%   cumulativeTruthMetrics(tem)
%
%   See also: trackAssignmentMetrics, trackOSPAMetric, trackGOSPAMetric,
%   trackerGNN, trackerTOMHT.

 
%   Copyright 2018 The MathWorks, Inc.

    methods
        function out=trackErrorMetrics
        end

        function out=cumulativeTrackMetrics(~) %#ok<STOUT>
            %cumulativeTrackMetrics  return cumulative metrics for all previous tracks
            %   trackMetricsTable(TEM) returns a table of cumulative metrics for
            %   every track identifier over all previous calls to step().
            %   When using the 'built-in' ErrorFunctionFormat, the table 
            %   will have columns that depend on the setting of the 'MotionModel'
            %   property:
            %      'constvel'  - posRMSE, velRMSE, posANEES, velANEES
            %      'constacc'  - posRMSE, velRMSE, accRMSE, posANEES, velANEES, accANEES
            %      'constturn' - posRMSE, velRMSE, yawRateRMSE, posANEES, velANEES, yawRateANEES
            %      'singer'    - posRMSE, velRMSE, accRMSE, posANEES, velANEES, accANEES
            %   where pos, vel, acc, and yawRate refer to the error in position,
            %   velocity, acceleration and turn (yaw rate).  RMSE denotes the
            %   root mean squared error, and ANEES denotes averaged normalized
            %   estimation error.
            %
            %   Otherwise the table contains the arithmetically averaged values
            %   of the custom metrics when ErrorFunctionFormat is 'custom'.
            %
            %   All metrics reported in this table are cumulative from the last
            %   invocation of the reset or release method.
        end

        function out=cumulativeTruthMetrics(~) %#ok<STOUT>
            %cumulativeTruthMetrics  return cumulative metrics for all previous truths
            %   cumulativeTruthMetrics(TEM) returns a table of cumulative metrics for
            %   every truth identifier over all previous calls to step().
            %   When using the 'built-in' ErrorFunctionFormat, the table 
            %   will have columns that depend on the setting of the 'MotionModel'
            %   property:
            %      'constvel'  - posRMSE, velRMSE, posANEES, velANEES
            %      'constacc'  - posRMSE, velRMSE, accRMSE, posANEES, velANEES, accANEES
            %      'constturn' - posRMSE, velRMSE, yawRateRMSE, posANEES, velANEES, yawRateANEES
            %      'singer'    - posRMSE, velRMSE, accRMSE, posANEES, velANEES, accANEES
            %   where pos, vel, acc, and yawRate refer to the error in position,
            %   velocity, acceleration and turn (yaw rate).  RMSE denotes the
            %   root mean squared error, and ANEES denotes averaged normalized
            %   estimation error.
            %
            %   Otherwise the table contains the arithmetically averaged values
            %   of the custom metrics when ErrorFunctionFormat is 'custom'.
            %
            %   All metrics reported in this table are cumulative from the last
            %   invocation of the reset or release method.
        end

        function out=currentTrackMetrics(~) %#ok<STOUT>
            %currentTrackMetrics  return metrics for most recent tracks
            %   currentTrackMetrics(TEM) returns a table of metrics for every
            %   track identifier provided in the most recent call to step().
            %   When using the 'built-in' ErrorFunctionFormat, the table 
            %   will have columns that depend on the setting of the 'MotionModel'
            %   property:
            %      'constvel'  - posRMSE, velRMSE, posANEES, velANEES
            %      'constacc'  - posRMSE, velRMSE, accRMSE, posANEES, velANEES, accANEES
            %      'constturn' - posRMSE, velRMSE, yawRateRMSE, posANEES, velANEES, yawRateANEES
            %      'singer'    - posRMSE, velRMSE, accRMSE, posANEES, velANEES, accANEES
            %   where pos, vel, acc, and yawRate refer to the error in position,
            %   velocity, acceleration and turn (yaw rate).  RMSE denotes the
            %   root mean squared error, and ANEES denotes averaged normalized
            %   estimation error.
            %
            %   Otherwise the table contains the arithmetically averaged values
            %   of the custom metrics when ErrorFunctionFormat is 'custom'.
        end

        function out=currentTruthMetrics(~) %#ok<STOUT>
            %currentTruthMetrics  return metrics for most recent truths
            %   truthMetricsTable(TEM) returns a table of metrics for every
            %   truth identifier provided in the most recent call to step().
            %   When using the 'built-in' ErrorFunctionFormat, the table 
            %   will have columns that depend on the setting of the 'MotionModel'
            %   property:
            %      'constvel'  - posRMSE, velRMSE, posANEES, velANEES
            %      'constacc'  - posRMSE, velRMSE, accRMSE, posANEES, velANEES, accANEES
            %      'constturn' - posRMSE, velRMSE, yawRateRMSE, posANEES, velANEES, yawRateANEES
            %      'singer'    - posRMSE, velRMSE, accRMSE, posANEES, velANEES, accANEES
            %   where pos, vel, acc, and yawRate refer to the error in position,
            %   velocity, acceleration and turn (yaw rate).  RMSE denotes the
            %   root mean squared error, and ANEES denotes averaged normalized
            %   estimation error.
            %
            %   Otherwise the table contains the arithmetically averaged values
            %   of the custom metrics when ErrorFunctionFormat is 'custom'.
        end

        function out=isInactivePropertyImpl(~) %#ok<STOUT>
        end

        function out=loadObjectImpl(~) %#ok<STOUT>
        end

        function out=resetImpl(~) %#ok<STOUT>
        end

        function out=saveObjectImpl(~) %#ok<STOUT>
        end

        function out=setupImpl(~) %#ok<STOUT>
        end

        function out=stepImpl(~) %#ok<STOUT>
            %stepImpl  step implementation for trackAssignmentMetrics
            %   [POSRMSE,VELRMSE,POSANEES,VELANEES] = step(TEM,TRACKS,TRACKIDS,TRUTHS,TRUTHIDS)
            %   when ErrorFunctionFormat is 'builtin' returns the root mean squared
            %   error in position, POSRMSE, and velocity, VELRMSE;
            %   and the averaged normalized estimation error squared in position, 
            %   POSANEES, and velocity, VELANEES, over the current track-to-truth
            %   assignments. TRACKIDS and TRUTHIDS are each a vector whose corresponding
            %   elements match the track and truth identifiers found in TRACKS and 
            %   TRUTHS, respectively.  TEM is the trackErrorMetrics object.
            %
            %   [OUT1, OUT2, ...] = step(TEM, TRACKS, TRACKIDS, TRUTHS, TRUTHIDS)
            %   returns the errors specified in the EstimationErrorFcn and
            %   EstimationErrorLabels averaged arithmetically over all
            %   track-to-truth assignments.
        end

    end
    properties
        %ErrorFunctionFormat  Selects custom error functions and format
        %   Selects the error function format.  Select one of:
        %      'built-in' - enables the MotionModel property, which can be used 
        %                   as a convenient interface when using TRACKS reported 
        %                   by a built-in multi-object tracker, and TRUTHS reported
        %                   by the platformPoses method of a trackingScenario object.
        %      'custom'   - enables EstimationErrorLabels, EstimationErrorFcn,
        %                   TruthIdentifierFcn, TrackIdentifierFcns.  These
        %                   properties can be used to construct error functions
        %                   for arbitrary TRACKS and TRUTHS input arrays.
        %
        %Default:  'built-in'
        ErrorFunctionFormat;

        %EstimationErrorFcn  A handle to an distance error function
        %   Specifies the function for determining the estimation errors
        %   of truths to tracks.  This function is called in a loop for
        %   each entry in the track-to-truth assignment specified in the
        %   TRACKIDS and TRUTHIDS parameters passed in the step method.
        %
        %   The function must have the following syntax: 
        %       [OUT1, OUT2, ..., OUTN] = ESTIMATIONERROR(TRACK, TRUTH)
        %
        %       OUT1, OUT2, ..., OUTN are scalar results that have a one-to-one
        %       correspondence with the string labels in the EstimationErrorLabels
        %       property.  trackErrorMetrics averages each output
        %       _arithmetically_ when reporting across tracks or truths.
        %       
        %       TRACK is an element of the TRACKS array passed to the step method,
        %
        %       TRUTH is an element of the TRUTHS array passed to the step method.
        %
        %       The default estimation error function assumes TRACKS and TRUTHS
        %       are each an array of struct, or an array of object.  
        EstimationErrorFcn;

        %EstimationErrorLabels  Array of string labels for each output of the EstimationErrorFcn
        %   Specifies the labels for each output of the EstimationErrorFcn
        %   property.  The number of labels must correspond to the number
        %   of outputs of the EstimationErrorFcn.  The labels may be an array
        %   of strings or a cell array of character vectors.
        %
        %Default: {'posMSE'}
        EstimationErrorLabels;

        %MotionModel  Specifies desired motion model
        %   Select the motion model used by the TRACKS input by choosing one of
        %   'constvel', 'constacc', 'constturn', or 'singer'.  These expect the
        %   'State' field to have a column vector as follows:
        %
        %      'constvel'  - position     in elements [1 3 5]
        %                    velocity     in elements [2 4 6]
        %
        %      'constacc'  - position     in elements [1 4 7]
        %                    velocity     in elements [2 5 8]
        %                    acceleration in elements [3 6 9]
        %
        %      'constturn' - position     in elements [1 3 6]
        %                    velocity     in elements [2 4 7]
        %                    yaw rate     in element  [5]
        %
        %      'singer'    - position     in elements [1 4 7]
        %                    velocity     in elements [2 5 8]
        %                    acceleration in elements [3 6 9]
        %
        %   Each option above also expects the 'StateCovariance' field to have
        %   position and velocity information in the rows and columns
        %   corresponding to the 'State' property positional input selector.
        %
        %   The motion model governs the outputs to the step function.
        %
        %Default:  'constvel'
        MotionModel;

        %TrackIdentifierFcn  Handle to a function returning unique track identifiers
        %   Specifies the track identifiers for the TRACK input of the step
        %   function.  The track identifiers should be unique string or numeric
        %   values.
        %
        %   The function must have the following syntax
        %      TRACKID = TRACKIDENTIFIER(TRACK)
        %
        %   TRACKID is a numeric array of the same size as TRACK.
        %   TRACK is the array passed into the step method.
        %
        %   The default identification function handle, @defaultTrackIdentifier
        %   assumes TRACK is an array with a 'TrackID' fieldname or property.  
        TrackIdentifierFcn;

        %TruthIdentifierFcn  Handle to a function returning unique truth identifiers
        %   Specifies the truth object identifiers for the TRUTH input of the
        %   step function.  The truth identifiers should be unique string or
        %   numeric values.
        %
        %   The function must have the following syntax
        %      TRUTHID = TRUTHIDENTIFIER(TRUTH)
        %
        %   TRUTHID is a numeric array of the same size as TRUTH.
        %   TRUTH is the array passed into the step method.
        %
        %   The default identification function assumes TRUTH is an array of
        %   struct or class with an 'PlatformID' fieldname or property.  
        TruthIdentifierFcn;

    end
end
