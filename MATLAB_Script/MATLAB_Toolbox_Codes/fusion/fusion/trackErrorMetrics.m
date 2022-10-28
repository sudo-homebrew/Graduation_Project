classdef trackErrorMetrics < matlab.System
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
%     reset                  - Resets state of cumulative metrics
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

properties (Nontunable,Dependent)
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
    ErrorFunctionFormat
    
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
    MotionModel

    %EstimationErrorLabels  Array of string labels for each output of the EstimationErrorFcn
    %   Specifies the labels for each output of the EstimationErrorFcn
    %   property.  The number of labels must correspond to the number
    %   of outputs of the EstimationErrorFcn.  The labels may be an array
    %   of strings or a cell array of character vectors.
    %
    %Default: {'posMSE'}
    EstimationErrorLabels
    
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
    EstimationErrorFcn    
    
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
    TrackIdentifierFcn
    
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
    TruthIdentifierFcn    
end

properties (Access = private)
    pTEM
end

properties (Hidden,Constant)
    ErrorFunctionFormatSet = matlab.system.StringSet({'built-in','custom'});
    MotionModelSet = matlab.system.StringSet({'constvel','constacc','constturn','singer'});
end

methods
    function obj = trackErrorMetrics(varargin)
        obj.pTEM = fusion.internal.metrics.ErrorMetrics;
        % Support name-value pair arguments when constructing object
        setProperties(obj,nargin,varargin{:});
    end
    
    function v = get.TruthIdentifierFcn(obj)
        v = obj.pTEM.TruthIdentifierFcn;
    end
    
    function v = get.TrackIdentifierFcn(obj)
        v = obj.pTEM.TrackIdentifierFcn;
    end
    
    function v = get.EstimationErrorFcn(obj)
        v = obj.pTEM.EstimationErrorFcn;
    end

    function v = get.EstimationErrorLabels(obj)
        v = obj.pTEM.EstimationErrorLabels;
    end
    
    function v = get.ErrorFunctionFormat(obj)
        v = obj.pTEM.ErrorFunctionFormat;
    end
    
    function v = get.MotionModel(obj)
        v = obj.pTEM.MotionModel;
    end
    
    function set.EstimationErrorLabels(obj, value)
        if isstring(value) || iscellstr(value)
            obj.pTEM.EstimationErrorLabels = value;
            setBuiltinErrorFunction(obj);
        else
           error(message('fusion:trackErrorMetrics:InvalidEstimationErrorLabels')); 
        end
    end
    
    function set.ErrorFunctionFormat(obj, value)
        obj.pTEM.ErrorFunctionFormat = value;
        setBuiltinErrorFunction(obj);
    end
    
    function set.MotionModel(obj, v)
        obj.pTEM.MotionModel = v;
    end
    
    function set.TruthIdentifierFcn(obj,v)
        if ~isa(v, 'function_handle')
            error(message('fusion:trackErrorMetrics:MustBeFunctionHandle','TruthIdentifierFcn'));
        end
        obj.pTEM.TruthIdentifierFcn = v;
    end
    
    function set.TrackIdentifierFcn(obj,v)
        if ~isa(v, 'function_handle')
            error(message('fusion:trackErrorMetrics:MustBeFunctionHandle','TrackIdentifierFcn'));
        end
        obj.pTEM.TrackIdentifierFcn = v;
    end
    
    function set.EstimationErrorFcn(obj,v)
        if ~isa(v, 'function_handle')
            error(message('fusion:trackErrorMetrics:MustBeFunctionHandle','EstimationErrorFcn'));
        end
        obj.pTEM.EstimationErrorFcn = v;
    end
    
    function tm = currentTrackMetrics(obj)
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
        tm = currentTrackMetricsTable(obj.pTEM);
    end
        
    function tm = currentTruthMetrics(obj)
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
        tm = currentTruthMetricsTable(obj.pTEM);
    end
    
    function ctm = cumulativeTrackMetrics(obj)
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
        ctm = cumulativeTrackMetricsTable(obj.pTEM);
    end
        
    function ctm = cumulativeTruthMetrics(obj)
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
        ctm = cumulativeTruthMetricsTable(obj.pTEM);
    end
end

methods (Hidden)
    function varargout = cumulativeMetrics(obj)
        varargout = cumulativeMetrics(obj.pTEM);
    end
end

methods (Access = protected)
    function varargout = stepImpl(obj, tracks, trackIDs, truths, truthIDs)
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
        
        validateTracks(obj, tracks);
        validateTruths(obj, truths);
        analyze(obj.pTEM, tracks, trackIDs, truths, truthIDs);
        varargout = lastMetrics(obj.pTEM);
    end
    
    function setupImpl(obj)
        setBuiltinErrorFunction(obj);
    end
    
    function resetImpl(obj)
        resetMetrics(obj.pTEM);
    end

    function flag = isInactivePropertyImpl(obj, prop)
        if strcmp(obj.ErrorFunctionFormat,'built-in')
            flag = ismember(prop,{'EstimationErrorLabels','EstimationErrorFcn','TrackIdentifierFcn','TruthIdentifierFcn'});
        else
            flag = ismember(prop,{'MotionModel'});
        end
    end
    
    function s = saveObjectImpl(obj)
        s.ErrorFunctionFormat = obj.ErrorFunctionFormat;
        s.MotionModel = obj.MotionModel;
        s.EstimationErrorFcn = obj.EstimationErrorFcn;
        s.TrackIdentifierFcn = obj.TrackIdentifierFcn;
        s.TruthIdentifierFcn = obj.TruthIdentifierFcn;
        if isLocked(obj)
            s.History = saveErrorMetrics(obj.pTEM);
        end
    end
    
    function loadObjectImpl(obj, s, wasLocked)
        obj.ErrorFunctionFormat = s.ErrorFunctionFormat;
        obj.MotionModel = s.MotionModel;
        obj.EstimationErrorFcn = s.EstimationErrorFcn;
        obj.TrackIdentifierFcn = s.TrackIdentifierFcn;
        obj.TruthIdentifierFcn = s.TruthIdentifierFcn;
        if wasLocked
            loadErrorMetrics(obj.pTEM, s.History);
        end
    end        
end

methods(Static, Hidden)
    function flag = isAllowedInSystemBlock
        flag = false;
    end
end

methods (Access = private)
    function setBuiltinErrorFunction(obj)
        if strcmp(obj.ErrorFunctionFormat,'built-in')
            setBuiltinErrorFunction(obj.pTEM);
        end
    end
    
    function validateTracks(obj, tracks)
        if strcmp(obj.ErrorFunctionFormat,'built-in')
            if ~isempty(tracks)
                cond = isa(tracks,'objectTrack') || (isstruct(tracks) && ...
                    all(isfield(tracks,{'TrackID','State','StateCovariance'})));
                if ~cond
                    error(message('fusion:trackErrorMetrics:InvalidBuiltInTrackStructure'));
                end
                
                nActual = numel(tracks(1).State);
                iModel = strcmp(obj.MotionModel,{'constvel','constacc','constturn', 'singer'});
                nExpected = [6 9 7 9];
                if ~isequal(nActual,nExpected(iModel)) || ~iscolumn(tracks(1).State)
                    error(message('fusion:trackErrorMetrics:InvalidState',obj.MotionModel));
                end
            end
        end
    end
    
    function validateTruths(obj, truths)
        if strcmp(obj.ErrorFunctionFormat,'built-in')
            if ~isempty(truths)
                if isstruct(truths)
                    if ~all(isfield(truths,{'PlatformID','Position','Velocity'}))
                       error(message('fusion:trackErrorMetrics:InvalidBuiltInTruthStructure'));
                    end
                    if ~isrow(truths(1).Position) || ~isrow(truths(1).Velocity)
                       error(message('fusion:trackErrorMetrics:InvalidTruthState'));
                    end
                elseif ~isa(truths, 'fusion.scenario.Platform')
                    error(message('fusion:trackErrorMetrics:InvalidBuiltInTruthPlatforms'));
                end 
            end
        end
    end
end
end
