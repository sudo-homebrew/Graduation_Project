classdef trackAssignmentMetrics < matlab.System
%trackAssignmentMetrics Track establishment, maintenance, and deletion metrics
%   trackAssignmentMetrics returns a System object capable of providing
%   quantitative comparison of a multiple object tracking system against
%   known "truth" objects, by automatic assignment of the tracks to the
%   known truths provided at each time step.  
%
%   An assignment distance metric is used to determine when a track may be
%   assigned to a truth object when it falls within a known threshold.  A
%   divergence distance metric is used to determine when a previously
%   assigned track may be re-assigned to a different truth object when it
%   exceeds a corresponding threshold.
%
%   TAM = trackAssignmentMetrics('Name',value,...) returns a track
%   assignment metric object, TAM, with each specified property name 
%   set to the specified value. You can specify additional name-value 
%   pair arguments in any order as (Name1,Value1,...,NameN, ValueN).
%   See the list of properties below.
%
%   trackAssignmentMetrics properties:
%      AssignmentThreshold    - Maximum permitted assignment distance
%      DivergenceThreshold    - Maximum permitted divergence distance 
%      DistanceFunctionFormat - Selects error functions and format
%                               one of 'built-in' (default) or 'custom'
%
%   trackAssignmentMetrics 'built-in' DistanceFunctionFormat properties:
%      MotionModel        - Specifies desired motion model.  One of:
%                              'constvel'  - constant velocity (default)
%                              'constacc'  - constant acceleration
%                              'constturn' - constant turn
%                              'singer'    - Singer acceleration
%      AssignmentDistance - Specifies the physical quantity for assignment
%                           One of:
%                              'posnees'   - the normalized estimation
%                                            error squared (NEES) in track
%                                            position  (default)
%                              'velnees'   - track velocity NEES
%                              'posabserr' - track position absolute error
%                              'velabserr' - track velocity absolute error
%      DivergenceDistance - Specifies the physical quantity for divergence
%                           One of:
%                              'posnees'   - the normalized estimation
%                                            error squared (NEES) in track
%                                            position  (default)
%                              'velnees'   - track velocity NEES
%                              'posabserr' - track position absolute error
%                              'velabserr' - track velocity absolute error
%
%   trackAssignmentMetrics 'custom' DistanceFunctionFormat properties:
%      AssignmentDistanceFcn   - Handle to an assignment distance function
%      DivergenceDistanceFcn   - Handle to a divergence distance function
%      IsInsideCoverageAreaFcn - Handle to a function that indicates when
%                                a truth object is detectable
%      TrackIdentifierFcn      - Handle to a function returning unique
%                                track identifiers
%      TruthIdentifierFcn      - Handle to a function returning unique
%                                truth identifiers
%      InvalidTrackIdentifier  - Track identifier for invalid assignment
%      InvalidTruthIdentifier  - Truth identifier for invalid assignment
%
%   Step method syntax: click on <a href="matlab:help trackAssignmentMetrics/stepImpl">step</a> for more details.
%
%      [TRACKSUMMARY, TRUTHSUMMARY] = step(TAM, TRACKS, TRUTHS) 
%      returns structures, TRACKSUMMARY and TRUTHSUMMARY,
%      that contain cumulative metrics across all tracks and truths
%      updated by all previous calls to step.  
%
%   System objects may be called directly like a function instead of using
%   the step method. For example, y = step(obj) and y = obj() are
%   equivalent. 
%
%   trackAssignmentMetrics methods:
%     <a href="matlab:help trackAssignmentMetrics/stepImpl">step</a>              - See above description for use of this method
%     currentAssignment - Returns Current mapping of track IDs to truth IDs
%     trackMetricsTable - Returns metrics for all cumulative tracks
%     truthMetricsTable - Returns metrics for all cumulative truths
%     release           - Allows property value and input characteristics changes
%     clone             - Creates a copy of the trackAssignmentMetrics
%     isLocked          - Locked status (logical)
%     reset             - Resets states of the trackAssignmentMetrics
%
%   
%   % EXAMPLE: Examine the results of a system that tracked two targets
%    
%   % load pre-recorded data
%   load trackmetricex tracklog truthlog
%
%   % construct an object to analyze assignment metrics
%   tam = trackAssignmentMetrics;
% 
%   for i=1:numel(tracklog)
%       % extract the tracker and ground truth at the i-th tracker update
%       tracks = tracklog{i};
%       truths = truthlog{i};
% 
%       % extract summary of assignment metrics against tracks and truths
%       [trackAM,truthAM] = tam(tracks, truths)
% 
%       % display the current track-to-truth assignment
%       [trackIDs,truthIDs] = currentAssignment(tam);
%       for j=1:numel(trackIDs)
%           fprintf('Track %i is assigned to Truth %i\n',trackIDs(j),truthIDs(j));
%       end
%   end 
% 
%   % show cumulative metrics for each individual recorded track
%   trackMetricsTable(tam)
% 
%   % show cumulative metrics for each individual recorded truth object
%   truthMetricsTable(tam)
%
%   See also: trackErrorMetrics, trackOSPAMetric, trackGOSPAMetric,
%   constvel, constacc, constturn, singer, trackingScenario.

%   References:
%   (1) S. B. Colegrove, S. J. Davey, B. Cheung. A Tracker Assessment Tool
%       for Comparing Tracker Performance.  DSTO-TR-1694 (2005).
%       DSTO Information Sciences Laboratory.
%
%   (2) Paul Votruba, Rich Nisley, Ron Rothrock, Brett Zombro ed. Single 
%       Integrated Air Picture (SIAP) Metrics Implementation.  
%       SIAP SE TF TR 2001-003 (2001).  Single Integrated Air Picture 
%       System Engineering Task Force. 

%   Copyright 2018 The MathWorks, Inc.

properties (Dependent, Nontunable)
    %DistanceFunctionFormat  Selects custom error functions and format
    %   Selects the distance function format.  Select one of:
    %      'built-in' - enables the MotionModel, AssignmentDistance, and
    %                   DivergenceDistance properties, which can be used 
    %                   as a convenient interface when using TRACKS
    %                   reported by any built-in multi-object tracker, and
    %                   TRUTHS reported by the platformPoses method of a
    %                   trackingScenario object.
    %      'custom'   - enables custom properties:  AssignmentDistanceFcn,
    %                   DivergenceDistanceFcn, IsInsideCoverageAreaFcn, 
    %                   TruthIdentifierFcn, TrackIdentifierFcns.  These
    %                   properties can be used to construct acceptance or
    %                   divergence distances, coverage areas, and identifiers
    %                   for arbitrary TRACKS and TRUTHS input arrays.
    %
    %Default:  'built-in'
    DistanceFunctionFormat
    
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
    
    %AssignmentThreshold  Maximum permitted assignment distance
    %   Specifies the maximum permitted assignment distance between a newly
    %   encountered or divergent track and a truth object, beyond which the
    %   assignment between the track and truth cannot take place.
    %
    %   Express AssignmentThreshold as a non-negative scalar value.  This
    %   threshold is typically expressed in units of NEES.
    %
    %Default:  1
    AssignmentThreshold

    %DivergenceThreshold  Maximum permitted divergence distance
    %   Specifies the maximum permitted divergence distance between a track
    %   state and the state of an assigned truth object, beyond which the
    %   track becomes eligible for reassignment to a different truth
    %   object.
    %
    %   Express DivergenceThreshold as a non-negative scalar value.  This
    %   threshold is typically expressed in units of NEES.
    %
    %Default:  2
    DivergenceThreshold
    
    %AssignmentDistance  Specifies the physical quantity for assignment
    %   Specifies the physical quantity for assignment when using the
    %   'built-in' DistanceFunctionFormat.  Whenever a new track is detected
    %   or a track becomes divergent, the track is compared against all
    %   truths passed to the current step method via this function seeking
    %   the closest truth that is within the threshold defined by the
    %   AssignmentThreshold property.  
    %   Set AssignmentDistance to of:
    %      'posnees'   - the normalized estimation error squared (NEES)
    %                    in track position  (default)
    %      'velnees'   - the NEES in track velocity
    %      'posabserr' - the absolute error in track position
    %      'velabserr' - the absolute error in track velocity
    %
    %Default:  'posnees'
    AssignmentDistance

    %DivergenceDistance  Specifies the physical quantity for divergence
    %   Specifies the physical quantity for divergence when using the
    %   'built-in' DistanceFunctionFormat.  Whenever a track was
    %   previously assigned to a truth, the distance between them is
    %   compared by this function on subsequent invocations to the step
    %   method.  Any track whose divergence distance to its respective
    %   truth assignment that exceeds the DivergenceThreshold property is
    %   considered divergent and may be reassigned to a new truth.
    %   Set DivergenceDistance to one of:
    %      'posnees'   - the normalized estimation error squared (NEES)
    %                    in track position  (default)
    %      'velnees'   - the NEES in track velocity
    %      'posabserr' - the absolute error in track position
    %      'velabserr' - the absolute error in track velocity
    %
    %Default:  'posnees'
    DivergenceDistance
    
    %AssignmentDistanceFcn  A handle to an assignment distance function
    %   Specifies the function for determining the assignment distance of
    %   truths to tracks.  Whenever a new track is detected or a track
    %   becomes divergent, the track is compared against all truths passed
    %   to the current step method via this function seeking the closest
    %   truth that is within the threshold defined by the
    %   AssignmentThreshold property.
    %
    %   The function must have the following syntax: 
    %       DISTANCE = ASSIGNMENTDISTANCE(TRACK, TRUTH)
    %
    %   DISTANCE is a non-negative assignment distance, typically expressed
    %            in units of NEES.
    %   TRACK is an element of the TRACKS array passed to the step method,
    %   TRUTH is an element of the TRUTHS array passed to the step method.
    AssignmentDistanceFcn
        
    %DivergenceDistanceFcn  A handle to an divergence distance function
    %   Specifies the function for determining the divergence distance of
    %   truths to tracks.  Whenever a track was previously assigned to a
    %   truth, the distance between them is compared by this function on
    %   subsequent invocations to the step method.  Any track whose
    %   divergence distance to its respective truth assignment that exceeds
    %   the DivergenceThreshold property is considered divergent and may be
    %   reassigned to a new truth.
    %
    %   The function must have the following syntax: 
    %       DISTANCE = DIVERGENCEDISTANCE(TRACK, TRUTH)
    %
    %   DISTANCE is a non-negative divergence distance, typically expressed
    %            in units of NEES.
    %   TRACK is an element of the TRACKS array passed to the step method,
    %   TRUTH is an element of the TRUTHS array passed to the step method.
    DivergenceDistanceFcn
        
    %IsInsideCoverageAreaFcn  Handle to a function that determines when a truth object is detectable.
    %   Specifies the function for determining when a truth object is
    %   inside the coverage area of the sensors in a tracking system.
    %   
    %   The function must have the following syntax:
    %       STATUS = ISINSIDECOVERAGEAREA(TRUTH)
    %   
    %   STATUS is a logical array the same size as the TRUTH input.  STATUS
    %   is true when the truth objects specified by TRUTH are within the
    %   coverage area.
    %   TRUTH is the array of truth objects expected to be passed in on
    %   each invocation to step.
    %
    %   The default coverage area function returns true for all truth
    %   input.
    IsInsideCoverageAreaFcn
    
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
    %   The default identification function assumes TRACK is an array of
    %   struct or class with a "TrackID" fieldname or property.  
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
    
    %InvalidTrackIdentifier  Track identifier for invalid assignment
    %   Specifies the value to report for a track with an invalid assignment
    %   This must evaluate to a numeric value or a string and be of the same
    %   class as returned by the TrackIdentifierFcn.
    %
    %Default:  NaN
    InvalidTrackIdentifier
    
    %InvalidTruthIdentifier  Truth identifier for invalid assignment
    %   Specifies the value to report for a truth with an invalid assignment
    %   This must evaluate to a numeric value or a string and be of the same
    %   class as returned by the TruthIdentifierFcn.
    %   
    %Default:  NaN
    InvalidTruthIdentifier
end

properties (Access = private)
    pTAM
end

properties (Hidden,Constant)
    DistanceFunctionFormatSet = matlab.system.StringSet({'built-in','custom'});
    AssignmentDistanceSet = matlab.system.StringSet({'posnees','velnees','posabserr','velabserr'});
    DivergenceDistanceSet = matlab.system.StringSet({'posnees','velnees','posabserr','velabserr'});
    MotionModelSet = matlab.system.StringSet({'constvel','constacc','constturn','singer'});
end


methods
    function obj = trackAssignmentMetrics(varargin)
        obj.pTAM = fusion.internal.metrics.AssignmentMetrics;
        % Support name-value pair arguments when constructing object
        setProperties(obj,nargin,varargin{:})
    end
    function v = get.AssignmentDistance(obj)
        v = obj.pTAM.AssignmentDistance;
    end
    function v = get.DivergenceDistance(obj)
        v = obj.pTAM.DivergenceDistance;
    end
    function v = get.DistanceFunctionFormat(obj)
        v = obj.pTAM.DistanceFunctionFormat;
    end
    function v = get.MotionModel(obj)
        v = obj.pTAM.MotionModel;
    end
    
    function v = get.AssignmentDistanceFcn(obj)
        v = obj.pTAM.AssignmentDistanceFcn;
    end
    function v = get.AssignmentThreshold(obj)
        v = obj.pTAM.AssignmentThreshold;
    end
    function v = get.DivergenceDistanceFcn(obj)
        v = obj.pTAM.DivergenceDistanceFcn;
    end
    function v = get.DivergenceThreshold(obj)
        v = obj.pTAM.DivergenceThreshold;
    end
    function v = get.IsInsideCoverageAreaFcn(obj)
        v = obj.pTAM.IsInsideCoverageAreaFcn;
    end
    function v = get.TruthIdentifierFcn(obj)
        v = obj.pTAM.TruthIdentifierFcn;
    end
    function v = get.TrackIdentifierFcn(obj)
        v = obj.pTAM.TrackIdentifierFcn;
    end
    function v = get.InvalidTrackIdentifier(obj)
        v = obj.pTAM.InvalidTrackID;
    end
    function v = get.InvalidTruthIdentifier(obj)
        v = obj.pTAM.InvalidTruthID;
    end

    function set.AssignmentDistance(obj,v)
        % validated by AssignmentDistanceSet
        obj.pTAM.AssignmentDistance = v;
    end
    function set.DivergenceDistance(obj,v)
        % validated by DivergenceDistanceSet
        obj.pTAM.DivergenceDistance = v;
    end
    function set.DistanceFunctionFormat(obj,v)
        % validated by DistanceFunctionFormatSet
        obj.pTAM.DistanceFunctionFormat = v;
    end
    function set.MotionModel(obj,v)
        % validated by MotionModelSet
        obj.pTAM.MotionModel = v;
    end
    function set.AssignmentThreshold(obj,v)
        if ~isscalar(v) || ~isreal(v) || v < 0
            error(message('fusion:trackAssignmentMetrics:MustBeNonNegative','AssignmentThreshold'));
        end
        obj.pTAM.AssignmentThreshold = v;
    end
    function set.DivergenceThreshold(obj,v)
        if ~isscalar(v) || ~isreal(v) || v < 0
            error(message('fusion:trackAssignmentMetrics:MustBeNonNegative','DivergenceThreshold'));
        end
        obj.pTAM.DivergenceThreshold = v;
    end
    function set.AssignmentDistanceFcn(obj,v)
        if ~isa(v, 'function_handle')
            error(message('fusion:trackAssignmentMetrics:MustBeFunctionHandle','AssignmentDistanceFcn'));
        end
        obj.pTAM.AssignmentDistanceFcn = v;
    end
    function set.DivergenceDistanceFcn(obj,v)
        if ~isa(v, 'function_handle')
            error(message('fusion:trackAssignmentMetrics:MustBeFunctionHandle','DivergenceDistanceFcn'));
        end
        obj.pTAM.DivergenceDistanceFcn = v;
    end
    function set.IsInsideCoverageAreaFcn(obj,v)
        if ~isa(v, 'function_handle')
            error(message('fusion:trackAssignmentMetrics:MustBeFunctionHandle','IsInsideCoverageAreaFcn'));
        end
        obj.pTAM.IsInsideCoverageAreaFcn = v;
    end
    function set.TruthIdentifierFcn(obj,v)
        if ~isa(v, 'function_handle')
            error(message('fusion:trackAssignmentMetrics:MustBeFunctionHandle','TruthIdentifierFcn'));
        end
        obj.pTAM.TruthIdentifierFcn = v;
    end
    function set.TrackIdentifierFcn(obj,v)
        if ~isa(v, 'function_handle')
            error(message('fusion:trackAssignmentMetrics:MustBeFunctionHandle','TrackIdentifierFcn'));
        end
        obj.pTAM.TrackIdentifierFcn = v;
    end
    function set.InvalidTrackIdentifier(obj,v)
        if ~isscalar(v) || ~isnumeric(v) && ~isstring(v)
            error(message('fusion:trackAssignmentMetrics:MustBeScalar','InvalidTrackIdentifier'));
        end
        obj.pTAM.InvalidTrackID = v;
    end
    function set.InvalidTruthIdentifier(obj,v)
        if ~isscalar(v) || ~isnumeric(v) && ~isstring(v)
            error(message('fusion:trackAssignmentMetrics:MustBeScalar','InvalidTruthIdentifier'));
        end
        obj.pTAM.InvalidTruthID = v;
    end
    
    function [trackIDs, truthIDs] = currentAssignment(obj)
        %currentAssignment  Returns Current mapping of track IDs to truth IDs
        %   [trackIDs, truthIDs] = currentAssignment(TAM) returns the
        %   assessment of track to truth assignment in the most recent
        %   update of the track assignment metrics, TAM. The assignment is
        %   returned in a vector of track identifiers, trackIDs, and truth
        %   identifiers, truthIDs, where the assignments are expressed in
        %   the corresponding elements of the vector.
        [trackIDs, truthIDs] = currentAssignment(obj.pTAM);
    end
    
    function trackMetrics = trackMetricsTable(obj)
        %trackMetricsTable  return metrics for every individual track
        %   trackMetricsTable(TAM) returns a table of metrics for every
        %   individual track:
        %      TrackID          - the unique track identifier
        %      AssignedTruthID  - the unique truth identifier
        %                         if the track is not assigned to any truth or the
        %                         track was not reported in the last update, then
        %                         value of AssignedTruthID is NaN.
        %      Surviving        - true if the track was reported in the last update
        %      TotalLength      - the number of updates this track was reported
        %      DeletionStatus   - true if this track was previously assigned to a
        %                         truth that was deleted while inside its coverage
        %                         area.  
        %      DeletionLength   - the number of updates the track was following a
        %                         deleted truth
        %      DivergenceStatus - true when the divergence distance between this 
        %                         track and its corresponding truth exceeded the
        %                         divergence threshold
        %      DivergenceCount  - the number of times this track entered a
        %                         divergent state
        %      DivergenceLength - the number of updates this track was in a
        %                         divergent state
        %      RedundancyStatus - true if this track is assigned to a truth already
        %                         associated with another track
        %      RedundancyCount  - the number of times this track entered a redundant
        %                         state
        %      RedundancyLength - the number of updates while in a redundant state
        %      FalseTrackStatus - true if the track was not assigned to any truth
        %      FalseTrackLength - the number of updates the track was unassigned
        %      SwapCount        - the number of times the track was assigned to a
        %                         new truth object.
        trackMetrics = trackMetricsTable(obj.pTAM);
    end
        
    function truthMetrics = truthMetricsTable(obj)
        %truthMetricsTable  return metrics for every individual truth
        %   truthMetricsTable(TAM) returns a table of metrics for every individual
        %   truth object:
        %      TruthID              - the unique identifier of the truth
        %      AssociatedTrackID    - the unique identifier of the associated track
        %      DeletionStatus       - false if the truth was reported in the last
        %                             update
        %      TotalLength          - the total number of updates this truth was
        %                             reported
        %      BreakStatus          - true when an established truth no longer has
        %                             any track assigned to it
        %      BreakCount           - the number of times this truth object entered
        %                             a broken state
        %      BreakLength          - the number of updates this truth object was
        %                             in a broken state
        %      InCoverageArea       - true when the truth object is inside the
        %                             coverage area
        %      EstablishmentStatus  - true if the truth was associated with any track
        %      EstablishmentLength  - the number of updates before this truth was
        %                             associated to any track while inside
        %                             the coverage area
        truthMetrics = truthMetricsTable(obj.pTAM);
    end
    
end

methods (Access = protected)
    
    function resetImpl(obj)
        reset(obj.pTAM);
    end
    
    function setupImpl(obj)
        setBuiltinDistanceFunctions(obj);
    end
    
    function setBuiltinDistanceFunctions(obj)
        if strcmp(obj.DistanceFunctionFormat,'built-in')
            setBuiltinDistanceFunctions(obj.pTAM);
        end
    end

    function [trackSummary, truthSummary] = stepImpl(obj, tracks, truths)
        %stepImpl  step implementation for trackAssignmentMetrics
        %   [TRACKSUMMARY, TRUTHSUMMARY] = step(TAM, TRACKS, TRUTHS)
        %   returns structures, TRACKSUMMARY and TRUTHSUMMARY, that contain 
        %   cumulative metrics across all tracks and truths updated by all 
        %   previous calls to the step method of the trackAssignmentMetrics
        %   object, TAM.
        %
        %   TRACKS is an array of objects or an array of struct. If the
        %   DistanceFunctionFormat is 'built-in', then the TRACKS input
        %   must contain 'State', 'StateCovariance' and 'TrackID'
        %   information. The track outputs from built-in trackers, such as
        %   trackerGNN, are compatible with the TRACKS input.
        %
        %   TRUTHS is an array of struct containing information for each
        %   truth.  When using a trackingScenario, TRUTHS may be taken
        %   directly from the platformPoses method.
        %
        %   TRACKSUMMARY is a struct containing the following cumulative
        %   metrics over all tracks encountered since the last call to
        %   the reset method:
        %
        %      TotalNumTracks         - The total number of unique track
        %                               identifiers encountered
        %
        %      NumFalseTracks         - The number of tracks never assigned
        %                               to any truth
        %
        %      MaxSwapCount           - The maximum and total number of track
        %      TotalSwapCount           swaps of each track.  A track swap
        %                               occurs whenever a track is assigned to
        %                               a different truth.
        %
        %      MaxDivergenceCount     - The maximum and total number of divergences.
        %      TotalDivergenceCount     A track is divergent when the result of
        %                               the divergence distance function is
        %                               greater than the divergence threshold.
        %
        %      MaxDivergenceLength    - The maximum and total number of updates
        %      TotalDivergenceLength    each track was in a divergent state
        %
        %      MaxRedundancyCount     - The maximum and total number of additional 
        %      TotalRedundancyCount     tracks assigned to the same truth
        %
        %      MaxRedundancyLength    - The maximum and total number of updates
        %      TotalRedundancyLength    each track was in a redundant state
        %
        %   TRUTHSUMMARY is a struct containing the following cumulative
        %   metrics over all truths encountered since the last call to the
        %   reset method:
        %
        %      TotalNumTruths           - The total number of unique truth
        %                                 identifiers encoutered
        %
        %      NumMissingTruths         - The total number of truths never
        %                                 established with any track
        %
        %      MaxEstablishmentLength   - The maximum and total number of
        %      TotalEstablishmentLength   updates before a truth was associated 
        %                                 with any track while inside the
        %                                 coverage area.  The lengths of 
        %                                 missing truths do not count
        %                                 toward this summary metric.
        %
        %      MaxBreakCount            - The maximum and total number of
        %      TotalBreakCount            times each truth was unassociated
        %                                 by any track after being
        %                                 established.
        %
        %      MaxBreakLength           - The maximum and total number of 
        %      TotalBreakLength           times each truth was in a broken state
        validateTracks(obj, tracks);
        validateTruths(obj, truths);
        
        obj.pTAM.analyze(tracks, truths);
        
        trackSummary = trackMetricsSummary(obj.pTAM);
        truthSummary = truthMetricsSummary(obj.pTAM);
    end
    
    function flag = isInactivePropertyImpl(obj, prop)
        if strcmp(obj.DistanceFunctionFormat,'built-in')
            flag = ismember(prop,{'AssignmentDistanceFcn','DivergenceDistanceFcn','IsInsideCoverageAreaFcn','TrackIdentifierFcn','TruthIdentifierFcn','InvalidTrackIdentifier','InvalidTruthIdentifier'});
        else
            flag = ismember(prop,{'MotionModel','AssignmentDistance','DivergenceDistance'});
        end
    end
    
    function loadObjectImpl(obj, s, wasLocked)
        obj.DistanceFunctionFormat    = s.DistanceFunctionFormat;
        obj.MotionModel               = s.MotionModel;
        obj.AssignmentThreshold       = s.AssignmentThreshold;
        obj.DivergenceThreshold       = s.DivergenceThreshold;
        obj.AssignmentDistance        = s.AssignmentDistance;
        obj.DivergenceDistance        = s.DivergenceDistance;
        obj.AssignmentDistanceFcn     = s.AssignmentDistanceFcn;
        obj.DivergenceDistanceFcn     = s.DivergenceDistanceFcn;
        obj.IsInsideCoverageAreaFcn   = s.IsInsideCoverageAreaFcn;
        obj.TrackIdentifierFcn        = s.TrackIdentifierFcn;
        obj.TruthIdentifierFcn        = s.TruthIdentifierFcn;
        obj.InvalidTrackIdentifier    = s.InvalidTrackIdentifier;
        obj.InvalidTruthIdentifier    = s.InvalidTruthIdentifier;
        if wasLocked
            loadHistory(obj.pTAM, s.History);
        end
    end
    
    function s = saveObjectImpl(obj)
        s.DistanceFunctionFormat    = obj.DistanceFunctionFormat;
        s.MotionModel               = obj.MotionModel;
        s.AssignmentThreshold       = obj.AssignmentThreshold;
        s.DivergenceThreshold       = obj.DivergenceThreshold;
        s.AssignmentDistance        = obj.AssignmentDistance;
        s.DivergenceDistance        = obj.DivergenceDistance;
        s.AssignmentDistanceFcn     = obj.AssignmentDistanceFcn;
        s.DivergenceDistanceFcn     = obj.DivergenceDistanceFcn;
        s.IsInsideCoverageAreaFcn   = obj.IsInsideCoverageAreaFcn;
        s.TrackIdentifierFcn        = obj.TrackIdentifierFcn;
        s.TruthIdentifierFcn        = obj.TruthIdentifierFcn;
        s.InvalidTrackIdentifier    = obj.InvalidTrackIdentifier;
        s.InvalidTruthIdentifier    = obj.InvalidTruthIdentifier;
        if isLocked(obj)
            s.History = saveHistory(obj.pTAM);
        end
    end
end

methods (Static, Hidden)
    function flag = isAllowedInSystemBlock
        flag = false;
    end
end

methods (Access = private)
    function validateTracks(obj, tracks)
        if strcmp(obj.DistanceFunctionFormat,'built-in')
            if ~isempty(tracks)
                cond = isa(tracks,'objectTrack') || (isstruct(tracks) && ...
                    all(isfield(tracks,{'TrackID','State'})));
                if ~cond
                    error(message('fusion:trackAssignmentMetrics:InvalidBuiltInTrackStructure'));
                end
                if isstruct(tracks) && ~isfield(tracks,'StateCovariance') && ...
                        (ismember(obj.AssignmentDistance,{'posnees','velnees'}) || ...
                         ismember(obj.DivergenceDistance,{'posnees','velnees'}))
                    error(message('fusion:trackAssignmentMetrics:InvalidBuiltInTrackStructureCovarianceRequired'));
                end
                n = numel(tracks(1).State);
                x = strcmp(obj.MotionModel,{'constvel','constacc','constturn','singer'});
                y = [6 9 7 9];
                if ~isequal(n,y(x)) || ~iscolumn(tracks(1).State)
                    error(message('fusion:trackAssignmentMetrics:InvalidState',obj.MotionModel));
                end
            end
        end
    end
    
    function validateTruths(obj, truths)
        if strcmp(obj.DistanceFunctionFormat,'built-in')
            if ~isempty(truths)
                if isstruct(truths)
                    if ~all(isfield(truths,{'PlatformID','Position','Velocity'}))
                       error(message('fusion:trackAssignmentMetrics:InvalidBuiltInTruthStructure'));
                    end
                    if ~isrow(truths(1).Position) || ~isrow(truths(1).Velocity)
                        error(message('fusion:trackAssignmentMetrics:InvalidTruthState'));
                    end
                else
                    error(message('fusion:trackAssignmentMetrics:InvalidBuiltInTruthPlatforms'));
                end
            end
        end
    end
end
end
